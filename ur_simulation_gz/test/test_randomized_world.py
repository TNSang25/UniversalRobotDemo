import importlib.util
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'depth_launch', PACKAGE / 'launch' / 'ur3e_rg2_table_depth.launch.py'
)
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)


def make_world():
    return ET.parse(PACKAGE / 'world' / 'ur_table_depth.sdf').getroot().find('world')


def cube_corners(cube):
    x, y, z, roll, pitch, yaw = map(float, cube.findtext('pose').split())
    assert z == pytest.approx(0.825)
    assert roll == pitch == 0
    assert -math.pi <= yaw <= math.pi
    size = list(map(float, cube.findtext('link/collision/geometry/box/size').split()))
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in (
        (-size[0] / 2, -size[1] / 2), (size[0] / 2, -size[1] / 2),
        (size[0] / 2, size[1] / 2), (-size[0] / 2, size[1] / 2),
    )]


def edges(polygon):
    return list(zip(polygon, polygon[1:] + polygon[:1]))


def point_edge_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                    / (dx * dx + dy * dy)))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def polygons_overlap(first, second):
    for a, b in edges(first) + edges(second):
        axis = (a[1] - b[1], b[0] - a[0])
        p = [x * axis[0] + y * axis[1] for x, y in first]
        q = [x * axis[0] + y * axis[1] for x, y in second]
        if max(p) < min(q) or max(q) < min(p):
            return False
    return True


def test_random_layout_geometry_across_seeds():
    # Check actual rotated square footprints, independently of the sampler's circles.
    xmin, xmax, ymin, ymax = LAUNCH.DEFAULT_CUBE_REGION
    for seed in range(200):
        world = make_world()
        LAUNCH.randomize_cube_poses(world, LAUNCH.DEFAULT_CUBE_REGION, seed)
        polygons = [cube_corners(world.find(f"model[@name='{name}']"))
                    for name in LAUNCH.CUBE_NAMES]
        for polygon in polygons:
            assert all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in polygon)
            assert min(point_edge_distance(LAUNCH.ROBOT_XY, a, b)
                       for a, b in edges(polygon)) >= 0.12 - 1e-9
        for first, second in itertools.combinations(polygons, 2):
            assert not polygons_overlap(first, second)
            gap = min(
                [point_edge_distance(point, a, b) for point in first for a, b in edges(second)]
                + [point_edge_distance(point, a, b) for point in second for a, b in edges(first)]
            )
            assert gap >= 0.015 - 1e-9


def test_seed_repeats_and_default_changes_layout():
    def poses(seed=None):
        world = make_world()
        LAUNCH.randomize_cube_poses(world, LAUNCH.DEFAULT_CUBE_REGION, seed)
        return [world.find(f"model[@name='{name}']/pose").text for name in LAUNCH.CUBE_NAMES]

    assert poses(42) == poses(42)
    assert poses(42) != poses(43)
    assert poses() != poses()


def test_randomization_preserves_other_models_and_source():
    source = (PACKAGE / 'world' / 'ur_table_depth.sdf').read_bytes()
    world = make_world()
    unchanged = {model.get('name'): ET.tostring(model) for model in world.findall('model')
                 if model.get('name') not in LAUNCH.CUBE_NAMES}
    LAUNCH.randomize_cube_poses(world, LAUNCH.DEFAULT_CUBE_REGION, 12)
    assert all(ET.tostring(world.find(f"model[@name='{name}']")) == data
               for name, data in unchanged.items())
    assert (PACKAGE / 'world' / 'ur_table_depth.sdf').read_bytes() == source


@pytest.mark.parametrize('bounds', [
    (-0.2, -0.3, -0.3, 0.3), (float('nan'), 0, -0.3, 0.3),
    (-0.6, 0, -0.3, 0.3), (-0.25, 0.1, -0.3, 0.3),
    (-0.25, 0, -0.5, 0.3), (-0.2, -0.19, -0.02, 0.02),
    (-0.2, -0.12, -0.04, 0.04),
])
def test_invalid_or_crowded_region_fails(bounds):
    with pytest.raises(ValueError):
        LAUNCH.randomize_cube_poses(make_world(), bounds, 42)


def test_zones_and_letters_shift_five_centimeters():
    old = ET.parse(PACKAGE / 'world' / 'ur_table.sdf').getroot().find('world')
    new = make_world()
    for name in ['tray_A', 'tray_B', 'tray_C', 'letter_A', 'letter_B', 'letter_C']:
        before = list(map(float, old.find(f"model[@name='{name}']/pose").text.split()))
        after = list(map(float, new.find(f"model[@name='{name}']/pose").text.split()))
        assert after[0] - before[0] == pytest.approx(0.05)
        assert after[1:] == before[1:]
