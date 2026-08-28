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
ROBOT_W_CM = 20
ROBOT_H_CM = 21
TURNING_RADIUS_CM = 25       # ~25cm, larger if moving faster
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
    # Robot footprint is 20x21cm; treat it as its centre point for the cheap
    # check here (the planner adds proper footprint clearance via the 30cm
    # padded planning footprint from the briefing).
    for ob in a["obstacles"]:
        if point_collides_obstacle(px, py, ob, a):
            return ob
    return None


def image_positions(arena: Optional[Dict] = None) -> List[Tuple[int, Obstacle]]:
    """(obstacle_id, obstacle) pairs that carry a target image."""
    a = arena or default_arena()
    return [(ob.id, ob) for ob in a["obstacles"] if ob.image_id is not None]


def to_config(x: float, y: float, theta: float = 0.0) -> Config:
    theta = math.atan2(math.sin(theta), math.cos(theta))  # wrap to (-pi, pi]
    return Config(x, y, theta)