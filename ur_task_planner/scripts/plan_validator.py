"""Validation of the JSON plans produced by the LLM.

This module has no ROS dependency so it can be unit tested on its own.
"""
import copy

# Parameters each skill takes. A step must contain exactly these.
SKILL_PARAMS = {
    'home': (),
    'pick': ('object',),
    'place': ('object', 'zone'),
    'move_above': ('object',),
    'move_to_zone': ('zone',),
    'open_gripper': (),
    'close_gripper': (),
}

LOCATION_TABLE = 'table'
LOCATION_GRIPPER = 'gripper'
LOCATION_UNKNOWN = 'unknown'


class PlanValidationError(Exception):
    """Raised when a plan must not be executed."""


class SceneState:
    """What the robot is holding and where every object currently is."""

    def __init__(self, holding=None, locations=None):
        self.holding = holding or None
        # object id -> 'table' | 'gripper' | 'unknown' | zone id
        self.locations = dict(locations or {})

    @classmethod
    def from_dict(cls, data):
        """Build the state from the JSON published on /scene_state."""
        if not isinstance(data, dict):
            raise ValueError('scene state must be a JSON object')
        locations = data.get('objects', {})
        if not isinstance(locations, dict):
            raise ValueError('"objects" must be a JSON object')
        return cls(holding=data.get('holding'), locations=locations)

    def occupant(self, zone):
        for obj, location in self.locations.items():
            if location == zone:
                return obj
        return None


class PlanValidator:

    def __init__(self, allowed_objects, allowed_zones, allowed_skills=None, max_steps=20):
        self.allowed_skills = list(allowed_skills) if allowed_skills else list(SKILL_PARAMS)
        unknown = [s for s in self.allowed_skills if s not in SKILL_PARAMS]
        if unknown:
            raise ValueError(f'skills without a parameter definition: {unknown}')
        self.allowed_objects = list(allowed_objects)
        self.allowed_zones = list(allowed_zones)
        self.max_steps = max_steps

    def validate(self, plan_data, state=None):
        """Return the list of steps to execute, or raise PlanValidationError.

        Every returned step is a dict with the keys 'skill', 'object' and 'zone'
        (empty string when the skill does not take that parameter).
        """
        if not isinstance(plan_data, dict):
            raise PlanValidationError('plan must be a JSON object with a "plan" list')
        plan = plan_data.get('plan')
        if not isinstance(plan, list):
            raise PlanValidationError('"plan" must be a list of steps')
        if not plan:
            reason = plan_data.get('reason')
            detail = f': {reason}' if isinstance(reason, str) and reason else ''
            raise PlanValidationError(f'plan is empty{detail}')
        if len(plan) > self.max_steps:
            raise PlanValidationError(
                f'plan has {len(plan)} steps, at most {self.max_steps} are allowed')

        steps = [self._check_step(i, step) for i, step in enumerate(plan, start=1)]
        self._check_sequence(steps, state)
        return steps

    def _check_step(self, index, step):
        where = f'step {index}'
        if not isinstance(step, dict):
            raise PlanValidationError(f'{where}: must be a JSON object')

        skill = step.get('skill')
        if not isinstance(skill, str) or skill not in self.allowed_skills:
            raise PlanValidationError(f'{where}: skill {skill!r} is not allowed')

        expected = SKILL_PARAMS[skill]
        for key in step:
            if key == 'skill':
                continue
            if key not in ('object', 'zone'):
                raise PlanValidationError(f'{where}: unknown field {key!r}')
            if key not in expected:
                raise PlanValidationError(f'{where}: skill {skill!r} does not take {key!r}')

        checked = {'skill': skill, 'object': '', 'zone': ''}
        for key in expected:
            value = step.get(key)
            if not isinstance(value, str) or not value:
                raise PlanValidationError(f'{where}: skill {skill!r} requires {key!r}')
            allowed = self.allowed_objects if key == 'object' else self.allowed_zones
            if value not in allowed:
                raise PlanValidationError(f'{where}: {key} {value!r} does not exist')
            checked[key] = value
        return checked

    def _check_sequence(self, steps, state):
        """Simulate the plan to catch steps that cannot succeed."""
        state = copy.deepcopy(state) if state is not None else SceneState()
        for index, step in enumerate(steps, start=1):
            where = f'step {index}'
            skill, obj, zone = step['skill'], step['object'], step['zone']

            if skill in ('pick', 'move_above') and \
                    state.locations.get(obj) == LOCATION_UNKNOWN:
                raise PlanValidationError(
                    f'{where}: the position of {obj!r} is unknown, it was dropped')

            if skill == 'pick':
                if state.holding:
                    raise PlanValidationError(
                        f'{where}: cannot pick {obj!r} while holding {state.holding!r}')
                state.holding = obj
                state.locations[obj] = LOCATION_GRIPPER
            elif skill == 'place':
                if state.holding != obj:
                    held = repr(state.holding) if state.holding else 'nothing'
                    raise PlanValidationError(
                        f'{where}: cannot place {obj!r}, the gripper holds {held}')
                occupant = state.occupant(zone)
                if occupant:
                    raise PlanValidationError(
                        f'{where}: {zone!r} is already occupied by {occupant!r}')
                state.holding = None
                state.locations[obj] = zone
            elif skill == 'move_above':
                if state.holding == obj:
                    raise PlanValidationError(
                        f'{where}: cannot move above {obj!r}, it is in the gripper')
            elif skill in ('open_gripper', 'close_gripper'):
                if state.holding:
                    raise PlanValidationError(
                        f'{where}: {skill!r} is not allowed while holding '
                        f'{state.holding!r}, use "place"')
