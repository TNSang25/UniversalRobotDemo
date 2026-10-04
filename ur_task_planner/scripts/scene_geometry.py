"""Conservative footprint checks, in gazebo_world metres."""
import math


def radius(size):
    return math.hypot(size[0], size[1]) / 2.0


def overlaps_zone(x, y, r, zone):
    dx = max(abs(x - zone['position'][0]) - zone['size'][0] / 2, 0.0)
    dy = max(abs(y - zone['position'][1]) - zone['size'][1] / 2, 0.0)
    return math.hypot(dx, dy) <= r


class TableWorkspace:
    def __init__(self, bounds, base, zones, sizes, gap=0.015,
                 keepout=0.12, reach=0.56, height=0.8):
        self.bounds, self.base, self.zones, self.sizes = bounds, base, zones, sizes
        self.gap, self.keepout, self.reach, self.height = gap, keepout, reach, height

    def check(self, obj, position, positions, locations):
        x, y = position
        r = radius(self.sizes[obj])
        xmin, xmax, ymin, ymax = self.bounds
        if not (xmin + r <= x <= xmax - r and ymin + r <= y <= ymax - r):
            raise ValueError('temporary position footprint is outside table placement bounds')
        distance = math.hypot(x - self.base[0], y - self.base[1])
        if distance < self.keepout + r or distance > self.reach - r:
            raise ValueError('temporary position is inside robot keepout or outside reach')
        if any(overlaps_zone(x, y, r + self.gap, zone) for zone in self.zones.values()):
            raise ValueError('temporary position overlaps a zone')
        for name in self.sizes:
            if name == obj or locations.get(name) == 'gripper':
                continue
            if locations.get(name) == 'unknown' or name not in positions:
                raise ValueError(f'cannot check clearance: position of {name} is unknown')
            other = positions[name]
            if math.hypot(x - other[0], y - other[1]) < r + radius(self.sizes[name]) + self.gap:
                raise ValueError(f'temporary position is too close to {name}')
