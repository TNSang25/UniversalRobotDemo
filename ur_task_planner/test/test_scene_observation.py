"""Measured poses, occupied-zone clearing, and LLM table placement constraints."""
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from cube_perception import detect_cubes, rotation_matrix
from plan_validator import PlanValidationError, PlanValidator, SceneState
from scene_geometry import TableWorkspace, overlaps_zone

OBJECTS = ['red_cube', 'blue_cube', 'yellow_cube']
ZONES = {f'zone_{letter}': {'position': [0.10, y, 0.81], 'size': [0.15, 0.15]}
         for letter, y in zip('abc', [0.15, 0.0, -0.15])}
SIZES = {name: [0.05, 0.05, 0.05] for name in OBJECTS}


def workspace():
    return TableWorkspace([-0.25, 0.015, -0.34, 0.34], [-0.35, 0, 0.81], ZONES, SIZES)


def occupied_state():
    return SceneState(locations={'red_cube': 'zone_b', 'blue_cube': 'zone_a', 'yellow_cube': 'zone_c'},
                      positions={'red_cube': [0.10, 0, 0.835], 'blue_cube': [0.10, 0.15, 0.835],
                                 'yellow_cube': [0.10, -0.15, 0.835]}, observed=True)


def test_llm_can_clear_target_when_all_zones_are_full():
    state = occupied_state()
    steps = PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place_on_table', 'object': 'red_cube', 'position': [-0.12, 0.25]},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]}, state)
    assert steps[1]['position'] == [-0.12, 0.25]
    assert state.locations['red_cube'] == 'zone_b'
    assert state.positions['red_cube'] == [0.10, 0, 0.835]


@pytest.mark.parametrize('position', [None, [0], [0, 1, 2], ['x', 1], [True, 0], [math.nan, 0], [math.inf, 0]])
def test_malformed_llm_coordinates_are_rejected(position):
    with pytest.raises(PlanValidationError, match='two finite numbers'):
        PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place_on_table', 'object': 'red_cube', 'position': position},
        ]}, occupied_state())


@pytest.mark.parametrize('position', [[-0.24, 0.0], [-0.12, 0.33], [0.0, 0.0], [0.10, 0.0], [-0.25, 0.0]])
def test_outside_or_zone_overlapping_positions_are_rejected(position):
    with pytest.raises(PlanValidationError):
        PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place_on_table', 'object': 'red_cube', 'position': position},
        ]}, occupied_state())


def test_second_displaced_cube_cannot_use_first_temporary_position():
    with pytest.raises(PlanValidationError, match='too close to red_cube'):
        PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place_on_table', 'object': 'red_cube', 'position': [-0.12, 0.25]},
            {'skill': 'pick', 'object': 'blue_cube'},
            {'skill': 'place_on_table', 'object': 'blue_cube', 'position': [-0.12, 0.25]},
        ]}, occupied_state())


def test_missing_observation_never_allows_table_clearance():
    state = occupied_state()
    state.locations['yellow_cube'] = 'unknown'
    with pytest.raises(PlanValidationError, match='unknown'):
        PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place_on_table', 'object': 'red_cube', 'position': [-0.12, 0.25]},
        ]}, state)


def test_keepout_and_reach_use_full_cube_footprint():
    w = workspace()
    w.bounds = [-0.5, 0.5, -0.4, 0.4]
    with pytest.raises(ValueError, match='keepout'):
        w.check('red_cube', [-0.35, 0], {}, {})
    with pytest.raises(ValueError, match='outside reach'):
        w.check('red_cube', [0.4, 0.3], {}, {})


def test_zone_occupancy_includes_cube_overhanging_zone_edge():
    assert overlaps_zone(0.005, 0, math.hypot(.05, .05)/2, ZONES['zone_b'])
    assert not overlaps_zone(-0.10, 0, math.hypot(.05, .05)/2, ZONES['zone_b'])


def synthetic_rgbd(yaw=0.3):
    # Downward optical camera: camera x = world x, camera y = -world y.
    rotation = rotation_matrix([1, 0, 0, 0])
    translation = np.array([0, 0, 1.6])
    intrinsic = [400, 0, 160, 0, 400, 120, 0, 0, 1]
    rgb = np.full((240, 320, 3), [130, 110, 90], dtype=np.uint8)
    depth = np.full((240, 320), .8)
    corners = np.array([[-.025, -.025], [.025, -.025], [.025, .025], [-.025, .025]])
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = corners @ rot.T + [-.10, .12]
    pixels = np.stack((corners[:, 0]*400/.75+160, -corners[:, 1]*400/.75+120), axis=-1)
    mask = np.zeros(depth.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(pixels).astype(np.int32), 1)
    rgb[mask == 1] = [255, 0, 0]
    depth[mask == 1] = .75
    return rgb, depth, intrinsic, rotation, translation


@pytest.mark.parametrize('yaw', [0, .3, -.6, 1.1])
def test_cube_pose_comes_from_rgb_depth_and_tf(yaw):
    args = synthetic_rgbd(yaw)
    found = detect_cubes(*args, SIZES)
    centre, measured_yaw = found['red_cube']
    assert centre == pytest.approx([-.10, .12, .825], abs=.003)
    assert abs(math.remainder(measured_yaw - yaw, math.pi/2)) < .08
    assert 'blue_cube' not in found


def test_invalid_depth_and_coloured_table_are_not_cubes():
    rgb, depth, k, rot, trans = synthetic_rgbd()
    depth[:] = np.nan
    assert detect_cubes(rgb, depth, k, rot, trans, SIZES) == {}
    depth[:] = .8
    assert detect_cubes(rgb, depth, k, rot, trans, SIZES) == {}


def test_scene_snapshot_keeps_measured_positions():
    state = SceneState.from_dict({'holding': None, 'observed': True,
                                 'objects': {'red_cube': 'zone_a'},
                                 'positions': {'red_cube': [.1, .15, .835]}})
    assert state.observed
    assert state.positions['red_cube'] == [.1, .15, .835]
    with pytest.raises(ValueError):
        SceneState.from_dict({'objects': {'red_cube': None}})


def test_cube_on_shared_zone_boundary_blocks_both_zones():
    state = occupied_state()
    state.locations['red_cube'] = 'zone_a'
    state.positions['red_cube'] = [.1, .075, .835]
    assert 'red_cube' in state.occupants('zone_a', workspace())
    assert 'red_cube' in state.occupants('zone_b', workspace())
    with pytest.raises(PlanValidationError, match='occupied'):
        PlanValidator(OBJECTS, ZONES, workspace=workspace()).validate({'plan': [
            {'skill': 'pick', 'object': 'yellow_cube'},
            {'skill': 'place', 'object': 'yellow_cube', 'zone': 'zone_b'},
        ]}, state)
