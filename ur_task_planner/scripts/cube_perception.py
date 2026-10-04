"""RGB-D cube detection for the coloured, unstacked Gazebo cubes.

No simulator pose service is used. RGB and depth must be registered.
"""
import math

import cv2
import numpy as np

HUE_RANGES = {
    'red_cube': ((0, 10), (170, 179)),
    'yellow_cube': ((20, 38),),
    'blue_cube': ((100, 135),),
    'green_cube': ((40, 85),),
    'pink_cube': ((140, 169),),
}


def rotation_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError('invalid TF quaternion')
    x, y, z, w = (v / norm for v in quaternion)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def detect_cubes(rgb, depth, intrinsic, rotation, translation, sizes, table_height=0.8):
    """Return id -> (XYZ centre, yaw). Ambiguous/occluded colours are omitted."""
    if rgb.shape[:2] != depth.shape or intrinsic[0] <= 0 or intrinsic[4] <= 0:
        raise ValueError('RGB/depth registration or camera intrinsics are invalid')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    v, u = np.indices(depth.shape)
    optical = np.stack(((u-intrinsic[2])*depth/intrinsic[0],
                        (v-intrinsic[5])*depth/intrinsic[4], depth), axis=-1)
    world = optical @ rotation.T + translation
    valid = np.isfinite(depth) & (depth > 0) & np.isfinite(world).all(axis=-1)
    found = {}
    for name, size in sizes.items():
        mask = np.zeros(depth.shape, dtype=np.uint8)
        for lo, hi in HUE_RANGES[name]:
            mask |= cv2.inRange(hsv, (lo, 90, 60), (hi, 255, 255))
        # The table, tray and labels are below a cube's top face.
        mask[~valid | (world[:, :, 2] < table_height + size[2] - 0.012)
             | (world[:, :, 2] > table_height + size[2] + 0.025)] = 0
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        components = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 50]
        if len(components) != 1:
            continue
        points = world[labels == components[0]]
        # Reject side-face pixels by keeping the upper, approximately planar face.
        top = np.percentile(points[:, 2], 80)
        points = points[abs(points[:, 2] - top) < 0.004]
        if len(points) < 40:
            continue
        (cx, cy), (sx, sy), angle = cv2.minAreaRect(points[:, :2].astype(np.float32))
        if not (0.65*size[0] <= sx <= 1.25*size[0]
                and 0.65*size[1] <= sy <= 1.25*size[1]):
            continue
        yaw = math.remainder(math.radians(angle), math.pi / 2)
        found[name] = ([float(cx), float(cy), float(np.median(points[:, 2])-size[2]/2)], yaw)
    return found
