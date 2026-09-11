"""Arena definition -- the single source of truth for the map.

The simulator (algorithm/simulator.py) and the ROS planner
(mdp_bringup/planner_core.py) both read from this module, so the grid,
obstacles and image positions can't drift between the offline sim and the
live run.

Coordinates: centimetres, origin at the bottom-left corner of the 2.0m x 2.0m
movement area, matching the course spec (Algorithms briefing) and the STM32
protocol (which also talks centimetres). Orientation theta is radians,
East = 0, counter-clockwise positive, in (-pi, pi].
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# --- Course constants (Algorithms briefing) ---------------------------------
ARENA_SIZE_CM = 200          # 2.0m x 2.0m
START_ZONE_CM = 40           # 40cm x 40cm at the bottom-left corner
ROBOT_W_CM = 19              # 19cm width (transverse)
ROBOT_H_CM = 23              # 23cm length (longitudinal)
TURNING_RADIUS_CM = 21.0     # calibrated 21.0cm physical turning radius (42cm diameter)
OBSTACLE_SIZE_CM = 10        # 10cm x 10cm footprint
IMAGE_RECOG_DIST_CM = 20     # ideal camera distance to an obstacle

# Time budget (B.2 scoring: "# images recognized within the time limit")
TIME_LIMIT_S = 360.0          # 6 minutes


@dataclass
class Obstacle:
    """A 10cm x 10cm block. The 'face' with the target image is one of
    N/S/E/W; the other three faces carry a bull's-eye marker."""
    id: int
    x: int               # centre x in cm
    y: int               # centre y in cm
    image_id: Optional[int] = None   # numeric class id (11-40), or None
    face: Optional[str] = None       # N/S/E/W, which side holds the target
    label: str = "marker"            # resolved label: 'marker' or the class id


@dataclass
class Config:
    """Robot configuration (pose)."""
    x: float
    y: float
    theta: float        # radians, East = 0

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    @classmethod
    def from_tuple(cls, t: Tuple[float, float, float]) -> "Config":
        return cls(t[0], t[1], t[2])


def default_arena() -> Dict:
    """The canonical starting arena: start zone + the 5 obstacles.

    Obstacle positions here are placeholders -- in the real run they come
    from the perception node (ObstacleDetection messages). Kept here so the
    simulator has something to plan against out of the box.
    """
    return {
        "arena_size": ARENA_SIZE_CM,
        "start_zone": START_ZONE_CM,
        "robot_w": ROBOT_W_CM,
        "robot_h": ROBOT_H_CM,
        "turning_radius": TURNING_RADIUS_CM,
        "obstacles": [
            Obstacle(id=1, x=60,  y=60,  image_id=11, face="N"),
            Obstacle(id=2, x=130, y=60,  image_id=22, face="E"),
            Obstacle(id=3, x=130, y=140, image_id=33, face="W"),
            Obstacle(id=4, x=60,  y=140, image_id=14, face="S"),
            Obstacle(id=5, x=95,  y=100, image_id=25, face="N"),
        ],
    }


def in_bounds(x: float, y: float, arena: Optional[Dict] = None,
              margin: float = 0.0) -> bool:
    """Is a point inside the movement area (with optional margin)?"""
    a = arena or default_arena()
    s = a["arena_size"]
    return (margin <= x <= s - margin) and (margin <= y <= s - margin)


def in_start_zone(x: float, y: float, arena: Optional[Dict] = None) -> bool:
    a = arena or default_arena()
    z = a["start_zone"]
    return 0.0 <= x <= z and 0.0 <= y <= z


def obstacle_footprint(ob: Obstacle, arena: Optional[Dict] = None
                      ) -> Tuple[float, float, float, float]:
    """Axis-aligned (x0, y0, x1, y1) of an obstacle's footprint in cm.

    The obstacle size is the module constant OBSTACLE_SIZE_CM (10 cm); the
    arena dict carries Obstacle objects, not a size, so the size comes from
    here regardless of which arena is passed.
    """
    half = OBSTACLE_SIZE_CM / 2.0
    return (ob.x - half, ob.y - half, ob.x + half, ob.y + half)


def point_collides_obstacle(px: float, py: float, ob: Obstacle,
                            arena: Optional[Dict] = None) -> bool:
    x0, y0, x1, y1 = obstacle_footprint(ob, arena)
    return x0 <= px <= x1 and y0 <= py <= y1


def collides_any(px: float, py: float, arena: Optional[Dict] = None) -> Optional[Obstacle]:
    a = arena or default_arena()
    # Robot footprint is 19x23cm; treat it as its centre point for the cheap
    # check here (the planner adds proper footprint clearance via the 30cm
    # padded planning footprint from the briefing).
    for ob in a["obstacles"]:
        if point_collides_obstacle(px, py, ob, a):
            return ob
    return None


# ----------------------------------------------------------------------------
# Oriented footprint collision.  The cheap point check above treats the robot
# as a point; the planner and simulator need the real 19x23cm body, so the
# robot is modelled as an oriented rectangle and tested against each 10x10
# obstacle with a separating-axis test.  This is the briefing's "30cm padded
# planning footprint" done properly (Minkowski sum of robot + obstacle) rather
# than an over-approximating inflation that rejects valid paths.
# ----------------------------------------------------------------------------
def _obb_collide(cx1: float, cy1: float, th1: float, w1: float, h1: float,
                 cx2: float, cy2: float, w2: float, h2: float) -> bool:
    """Separating-axis test between robot oriented rectangle (w1 width, h1 length along heading th1)
    and axis-aligned rectangle (w2 width in x, h2 height in y).
    """
    c, s = math.cos(th1), math.sin(th1)
    hw1, hh1 = w1 / 2.0, h1 / 2.0  # hw1 = half-width (transverse), hh1 = half-length (longitudinal)
    hw2, hh2 = w2 / 2.0, h2 / 2.0
    dx, dy = cx1 - cx2, cy1 - cy2
    # Axes to test: rect1's heading (c, s) and transverse (-s, c) + world axes (1, 0), (0, 1)
    axes = ((c, s), (-s, c), (1.0, 0.0), (0.0, 1.0))
    for ax, ay in axes:
        # Extent of rect1 (hh1 along heading (c, s), hw1 along transverse (-s, c))
        e1 = (hh1 * abs(ax * c + ay * s)
              + hw1 * abs(-ax * s + ay * c))
        # Extent of rect2 (axis-aligned) along this axis.
        e2 = hw2 * abs(ax) + hh2 * abs(ay)
        if abs(dx * ax + dy * ay) >= e1 + e2:
            return False  # a separating axis exists -> no overlap
    return True


def robot_corners(px: float, py: float, theta: float,
                  robot_w: float = ROBOT_W_CM,
                  robot_h: float = ROBOT_H_CM) -> List[Tuple[float, float]]:
    """Return the 4 world-space corners of the oriented robot chassis."""
    c, s = math.cos(theta), math.sin(theta)
    hh = robot_h / 2.0  # longitudinal (heading)
    hw = robot_w / 2.0  # transverse (side)
    return [
        (px + hh * c - hw * s, py + hh * s + hw * c),
        (px + hh * c + hw * s, py + hh * s - hw * c),
        (px - hh * c + hw * s, py - hh * s - hw * c),
        (px - hh * c - hw * s, py - hh * s + hw * c),
    ]


def robot_in_bounds(px: float, py: float, theta: float,
                    arena: Optional[Dict] = None,
                    robot_w: Optional[float] = None,
                    robot_h: Optional[float] = None,
                    margin: float = 0.0) -> bool:
    """Check if the entire oriented robot chassis stays within arena walls."""
    a = arena or default_arena()
    s = a.get("arena_size", ARENA_SIZE_CM)
    rw = robot_w if robot_w is not None else a.get("robot_w", ROBOT_W_CM)
    rh = robot_h if robot_h is not None else a.get("robot_h", ROBOT_H_CM)
    for cx, cy in robot_corners(px, py, theta, rw, rh):
        if not (margin <= cx <= s - margin and margin <= cy <= s - margin):
            return False
    return True


def robot_collides_obstacle(px: float, py: float, theta: float, ob: Obstacle,
                            robot_w: Optional[float] = None,
                            robot_h: Optional[float] = None,
                            arena: Optional[Dict] = None) -> bool:
    """Is the robot's oriented body (centre at (px,py), heading theta)
    overlapping this 10x10 obstacle?"""
    a = arena or default_arena()
    rw = robot_w if robot_w is not None else a["robot_w"]
    rh = robot_h if robot_h is not None else a["robot_h"]
    sz = OBSTACLE_SIZE_CM
    return _obb_collide(px, py, theta, rw, rh, ob.x, ob.y, sz, sz)


def robot_collides_any(px: float, py: float, theta: float,
                       arena: Optional[Dict] = None,
                       robot_w: Optional[float] = None,
                       robot_h: Optional[float] = None,
                       safety_margin: float = 4.0
                       ) -> Optional[Obstacle]:
    """First obstacle or wall boundary the robot's oriented body collides with (or None)."""
    a = arena or default_arena()
    rw = (robot_w if robot_w is not None else a.get("robot_w", ROBOT_W_CM)) + 2.0 * safety_margin
    rh = (robot_h if robot_h is not None else a.get("robot_h", ROBOT_H_CM)) + 2.0 * safety_margin

    # 1. Check arena boundaries
    if not robot_in_bounds(px, py, theta, a, rw, rh, margin=0.0):
        return Obstacle(id=-1, x=int(px), y=int(py), label="wall")

    # 2. Check each obstacle
    for ob in a["obstacles"]:
        if robot_collides_obstacle(px, py, theta, ob, rw, rh, a):
            return ob
    return None


def path_collides(poses, step: float = 2.0,
                  arena: Optional[Dict] = None,
                  robot_w: Optional[float] = None,
                  robot_h: Optional[float] = None,
                  safety_margin: float = 4.0
                  ) -> Optional[Obstacle]:
    """Sample a path (list of (x, y, theta) poses) and find the first
    obstacle or boundary wall the robot body runs into.
    """
    if not poses:
        return None
    a = arena or default_arena()
    p0 = poses[0]
    last_x, last_y = (p0.x, p0.y) if isinstance(p0, Config) else (p0[0], p0[1])
    n = len(poses)
    for i in range(n):
        p = poses[i]
        x, y, theta = (p.x, p.y, p.theta) if isinstance(p, Config) else (p[0], p[1], p[2])
        if i > 0:
            seg = math.hypot(x - last_x, y - last_y)
            k = max(1, int(math.ceil(seg / step)))
            for j in range(1, k + 1):
                t = j / k
                px = last_x + (x - last_x) * t
                py = last_y + (y - last_y) * t
                th = theta
                hit = robot_collides_any(px, py, th, a, robot_w, robot_h, safety_margin=safety_margin)
                if hit is not None:
                    return hit
        else:
            hit = robot_collides_any(x, y, theta, a, robot_w, robot_h, safety_margin=safety_margin)
            if hit is not None:
                return hit
        last_x, last_y = x, y
    return None


def image_positions(arena: Optional[Dict] = None) -> List[Tuple[int, Obstacle]]:
    """(obstacle_id, obstacle) pairs that carry a target image."""
    a = arena or default_arena()
    return [(ob.id, ob) for ob in a["obstacles"] if ob.image_id is not None]


def to_config(x: float, y: float, theta: float = 0.0) -> Config:
    theta = math.atan2(math.sin(theta), math.cos(theta))  # wrap to (-pi, pi]
    return Config(x, y, theta)