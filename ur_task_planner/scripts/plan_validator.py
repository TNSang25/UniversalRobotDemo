"""Validation of the JSON plans produced by the LLM.

This module has no ROS dependency so it can be unit tested on its own.
"""
import copy
import math

from scene_geometry import overlaps_zone, radius

# Parameters each skill takes. A step must contain exactly these.
SKILL_PARAMS = {
    'home': (),
    'observe': (),
    'place_on_table': ('object', 'position'),
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

    def __init__(self, holding=None, locations=None, positions=None, observed=False, revision=-1):
        self.holding = holding or None
        # object id -> 'table' | 'gripper' | 'unknown' | zone id
        self.locations = dict(locations or {})
        self.positions = copy.deepcopy(positions or {})
        self.observed = observed
        self.revision = revision

    @classmethod
    def from_dict(cls, data):
        """Build the state from the JSON published on /scene_state."""
        if not isinstance(data, dict):
            raise ValueError('scene state must be a JSON object')
        locations = data.get('objects', {})
        if not isinstance(locations, dict):
            raise ValueError('"objects" must be a JSON object')
        if any(not isinstance(k, str) or not isinstance(v, str) for k, v in locations.items()):
            raise ValueError('object locations must be strings')
        holding = data.get('holding')
        if holding is not None and not isinstance(holding, str):
            raise ValueError('holding must be a string or null')
        positions = data.get('positions', {})
        if not isinstance(positions, dict) or any(
                not isinstance(p, list) or len(p) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in p) for p in positions.values()):
            raise ValueError('positions must map object ids to finite XYZ lists')
        revision = data.get('revision', -1)
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < -1:
            raise ValueError('revision must be a nonnegative integer')
        return cls(holding=holding, locations=locations, positions=positions,
                   observed=data.get('observed') is True, revision=revision)

    def occupants(self, zone, workspace=None):
        result = []
        for obj, location in self.locations.items():
            if location in (LOCATION_UNKNOWN, LOCATION_GRIPPER):
                continue
            if workspace is not None and obj in self.positions and obj in workspace.sizes:
                x, y = self.positions[obj][:2]
                if overlaps_zone(x, y, radius(workspace.sizes[obj]), workspace.zones[zone]):
                    result.append(obj)
            elif location == zone:
                result.append(obj)
        return result

    def occupant(self, zone):
        for obj, location in self.locations.items():
            if location == zone:
                return obj
        return None


class PlanValidator:

    def __init__(self, allowed_objects, allowed_zones, allowed_skills=None, max_steps=20, workspace=None):
        self.allowed_skills = list(allowed_skills) if allowed_skills else list(SKILL_PARAMS)
        unknown = [s for s in self.allowed_skills if s not in SKILL_PARAMS]
        if unknown:
            raise ValueError(f'skills without a parameter definition: {unknown}')
        self.allowed_objects = list(allowed_objects)
        self.allowed_zones = list(allowed_zones)
        self.max_steps = max_steps
        self.workspace = workspace

    def validate(self, plan_data, state=None):
        """Return the list of steps to execute, or raise PlanValidationError.

        Every returned step has 'skill', 'object' and 'zone'; place_on_table also has 'position'
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
            if key not in ('object', 'zone', 'position'):
                raise PlanValidationError(f'{where}: unknown field {key!r}')
            if key not in expected:
                raise PlanValidationError(f'{where}: skill {skill!r} does not take {key!r}')

        checked = {'skill': skill, 'object': '', 'zone': ''}
        for key in expected:
            value = step.get(key)
            if key == 'position':
                if not isinstance(value, list) or len(value) != 2 or any(
                        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                        for v in value):
                    raise PlanValidationError(f'{where}: position must be two finite numbers [x, y]')
                checked[key] = list(value)
                continue
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
            elif skill in ('place', 'place_on_table'):
                if state.holding != obj:
                    held = repr(state.holding) if state.holding else 'nothing'
                    raise PlanValidationError(
                        f'{where}: cannot place {obj!r}, the gripper holds {held}')
                if skill == 'place_on_table':
                    if self.workspace is None:
                        raise PlanValidationError(f'{where}: table workspace is not configured')
                    try:
                        self.workspace.check(obj, step['position'], state.positions, state.locations)
                    except ValueError as error:
                        raise PlanValidationError(f'{where}: {error}') from error
                    state.holding = None
                    state.locations[obj] = LOCATION_TABLE
                    state.positions[obj] = [*step['position'],
                                            self.workspace.height + self.workspace.sizes[obj][2] / 2]
                    continue
                occupants = state.occupants(zone, self.workspace)
                occupant = occupants[0] if occupants else None
                if occupant:
                    raise PlanValidationError(
                        f'{where}: {zone!r} is already occupied by {occupant!r}')
                state.holding = None
                state.locations[obj] = zone
                if self.workspace is not None:
                    state.positions[obj] = list(self.workspace.zones[zone]['position'])
                    state.positions[obj][2] += self.workspace.sizes[obj][2] / 2
            elif skill == 'move_above':
                if state.holding == obj:
                    raise PlanValidationError(
                        f'{where}: cannot move above {obj!r}, it is in the gripper')
            elif skill in ('open_gripper', 'close_gripper'):
                if state.holding:
                    raise PlanValidationError(
                        f'{where}: {skill!r} is not allowed while holding '
                        f'{state.holding!r}, use "place"')
