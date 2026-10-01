import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from plan_validator import PlanValidationError, PlanValidator, SceneState  # noqa: E402

OBJECTS = ['red_cube', 'blue_cube', 'yellow_cube']
ZONES = ['zone_a', 'zone_b', 'zone_c']


@pytest.fixture
def validator():
    return PlanValidator(OBJECTS, ZONES)


@pytest.fixture
def state():
    return SceneState(locations={name: 'table' for name in OBJECTS})


def test_reference_plan_is_accepted(validator, state):
    steps = validator.validate({'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]}, state)
    assert steps == [
        {'skill': 'pick', 'object': 'red_cube', 'zone': ''},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'home', 'object': '', 'zone': ''},
    ]


def test_validation_does_not_modify_the_state(validator, state):
    validator.validate({'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
    ]}, state)
    assert state.holding is None
    assert state.locations['red_cube'] == 'table'


def test_primitive_skills_are_accepted(validator, state):
    steps = validator.validate({'plan': [
        {'skill': 'open_gripper'},
        {'skill': 'move_above', 'object': 'blue_cube'},
        {'skill': 'move_to_zone', 'zone': 'zone_c'},
        {'skill': 'close_gripper'},
    ]}, state)
    assert [s['skill'] for s in steps] == [
        'open_gripper', 'move_above', 'move_to_zone', 'close_gripper']


def test_two_objects_into_two_zones(validator, state):
    validator.validate({'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]}, state)


def test_zone_is_free_again_after_its_object_is_picked(validator):
    state = SceneState(locations={'red_cube': 'zone_a', 'blue_cube': 'table'})
    validator.validate({'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_a'},
    ]}, state)


def test_place_of_an_object_already_held(validator):
    state = SceneState(holding='red_cube', locations={'red_cube': 'gripper'})
    validator.validate({'plan': [
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
    ]}, state)


@pytest.mark.parametrize('plan_data, expected', [
    # structure
    ([{'skill': 'home'}], 'must be a JSON object'),
    ('home', 'must be a JSON object'),
    (None, 'must be a JSON object'),
    ({'steps': [{'skill': 'home'}]}, 'must be a list'),
    ({'plan': {'skill': 'home'}}, 'must be a list'),
    ({'plan': []}, 'plan is empty'),
    ({'plan': [], 'reason': 'no green cube'}, 'no green cube'),
    ({'plan': ['home']}, 'step 1: must be a JSON object'),
    ({'plan': [{'skill': 'home'}] * 21}, 'at most 20'),
    # skills
    ({'plan': [{'skill': 'throw', 'object': 'red_cube'}]}, "skill 'throw' is not allowed"),
    ({'plan': [{'object': 'red_cube'}]}, 'skill None is not allowed'),
    ({'plan': [{'skill': ['pick'], 'object': 'red_cube'}]}, 'is not allowed'),
    ({'plan': [{'skill': 'Pick', 'object': 'red_cube'}]}, "skill 'Pick' is not allowed"),
    # objects and zones
    ({'plan': [{'skill': 'pick', 'object': 'green_cube'}]}, "object 'green_cube' does not exist"),
    ({'plan': [{'skill': 'move_to_zone', 'zone': 'zone_d'}]}, "zone 'zone_d' does not exist"),
    ({'plan': [{'skill': 'pick', 'object': 'zone_a'}]}, "object 'zone_a' does not exist"),
    # parameters
    ({'plan': [{'skill': 'pick'}]}, "requires 'object'"),
    ({'plan': [{'skill': 'pick', 'object': ''}]}, "requires 'object'"),
    ({'plan': [{'skill': 'pick', 'object': None}]}, "requires 'object'"),
    ({'plan': [{'skill': 'pick', 'object': 5}]}, "requires 'object'"),
    ({'plan': [{'skill': 'move_to_zone'}]}, "requires 'zone'"),
    ({'plan': [{'skill': 'home', 'object': 'red_cube'}]}, "does not take 'object'"),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube', 'zone': 'zone_a'}]},
     "does not take 'zone'"),
    ({'plan': [{'skill': 'home', 'speed': 10}]}, "unknown field 'speed'"),
    # sequence
    ({'plan': [{'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'}]},
     'the gripper holds nothing'),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'place', 'object': 'red_cube'}]}, "requires 'zone'"),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_a'}]},
     "the gripper holds 'red_cube'"),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'pick', 'object': 'blue_cube'}]}, "while holding 'red_cube'"),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'open_gripper'}]}, 'use "place"'),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'move_above', 'object': 'red_cube'}]}, 'it is in the gripper'),
    ({'plan': [{'skill': 'pick', 'object': 'red_cube'},
               {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
               {'skill': 'pick', 'object': 'blue_cube'},
               {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_a'}]},
     "'zone_a' is already occupied by 'red_cube'"),
])
def test_invalid_plans_are_rejected(validator, state, plan_data, expected):
    with pytest.raises(PlanValidationError) as error:
        validator.validate(plan_data, state)
    assert expected in str(error.value)


def test_occupied_zone_from_scene_state(validator):
    state = SceneState(locations={'red_cube': 'zone_b', 'blue_cube': 'table'})
    with pytest.raises(PlanValidationError) as error:
        validator.validate({'plan': [
            {'skill': 'pick', 'object': 'blue_cube'},
            {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_b'},
        ]}, state)
    assert "'zone_b' is already occupied by 'red_cube'" in str(error.value)


def test_pick_while_holding_from_scene_state(validator):
    state = SceneState(holding='red_cube', locations={'red_cube': 'gripper'})
    with pytest.raises(PlanValidationError):
        validator.validate({'plan': [{'skill': 'pick', 'object': 'blue_cube'}]}, state)


@pytest.mark.parametrize('step', [
    {'skill': 'pick', 'object': 'red_cube'},
    {'skill': 'move_above', 'object': 'red_cube'},
])
def test_dropped_object_cannot_be_used(validator, step):
    state = SceneState(locations={'red_cube': 'unknown', 'blue_cube': 'table'})
    with pytest.raises(PlanValidationError) as error:
        validator.validate({'plan': [step]}, state)
    assert "the position of 'red_cube' is unknown" in str(error.value)
    validator.validate({'plan': [{'skill': 'pick', 'object': 'blue_cube'}]}, state)


def test_restricted_skill_list():
    validator = PlanValidator(OBJECTS, ZONES, allowed_skills=['home', 'pick', 'place'])
    with pytest.raises(PlanValidationError) as error:
        validator.validate({'plan': [{'skill': 'open_gripper'}]})
    assert 'is not allowed' in str(error.value)


def test_skill_without_definition_is_a_configuration_error():
    with pytest.raises(ValueError):
        PlanValidator(OBJECTS, ZONES, allowed_skills=['home', 'fly'])


def test_scene_state_from_dict():
    state = SceneState.from_dict(
        {'holding': None, 'objects': {'red_cube': 'zone_a', 'blue_cube': 'table'}})
    assert state.holding is None
    assert state.occupant('zone_a') == 'red_cube'
    assert state.occupant('zone_b') is None
    with pytest.raises(ValueError):
        SceneState.from_dict(['red_cube'])
