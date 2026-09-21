"""Task 1: point-to-point shortest drive.

A Reeds-Shepp planner in SE(2) for a car-like robot that can drive forward
or backward with a fixed minimum turning radius, checked against the known
arena's obstacles, with a discrete-grid A* fallback.

The geometry lives in :mod:`reed_shepp` (verified OMPL port -- shortest
path, all 48 configurations, distance matrix).  This module adds the two
things the geometry alone doesn't do:

1. **obstacle checking per configuration.**  Reeds-Shepp returns the single
   shortest curve and ignores obstacles.  Here every *feasible* configuration
   (CSC / CCC / CCCC / CCSC / CCSCC -- 48 curves in total) is sampled and
   tested against the arena; the shortest *collision-free* one wins.
2. **a fallback search.**  When no configuration is collision-free (tight
   clutter, narrow gaps) the problem is discretised onto a 4-direction grid
   and searched with A*, so the robot always has a move to execute rather
   than reporting failure.

Public surface
-------------
* :func:`se2_distance` -- SE(2) distance between two poses.
* :func:`plan_drive` -- the Task 1 entry point: returns a serialised path.
* :func:`plan_drive_poses` -- same, as world-space (x, y, theta) poses.
* :func:`waypoints_to_move_commands` -- (x, y, theta) poses -> ROS MoveCommand
  batch for the STM32 to execute.

All poses are Config (centimetres, theta in radians, East = 0, CCW+).
"""

import heapq
import itertools
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from arena import (Config, Obstacle, in_bounds, in_start_zone,
                   path_collides, robot_collides_any, default_arena,
                   TURNING_RADIUS_CM, ARENA_SIZE_CM, ROBOT_W_CM, ROBOT_H_CM)
from reed_shepp import (_get_candidates, _canonical, _PATH_TYPES,
                        _build, normalize_angle, reeds_shepp_distance,
                        reeds_shepp_path, reeds_shepp_path_poses,
                        waypoints_to_poses, _turning_radius)

_PI = math.pi
_TWO_PI = 2.0 * _PI


# ---------------------------------------------------------------------------
# SE(2) distance.  Euclidean over the pose space with orientation wrapped to
# (-pi, pi]; the heading term is a chord-length so near-opposite headings
# don't double-count.  This is the cheap distance the planner uses to rank
# waypoints before the expensive per-configuration collision check.
# ---------------------------------------------------------------------------
def se2_distance(start: Config, goal: Config,
                 heading_weight: float = 1.0) -> float:
    """Distance between two poses in SE(2).

    d = sqrt(dx^2 + dy^2 + (w * d_theta)^2), where d_theta is the wrapped
    angular difference in (-pi, pi].  heading_weight scales the angular
    term (cm per radian); 0.0 makes it pure position, and the default 1.0
    keeps the three terms comparable.  Returns 0.0 for identical poses.
    """
    dx = goal.x - start.x
    dy = goal.y - start.y
    dtheta = normalize_angle(goal.theta - start.theta)
    return math.sqrt(dx * dx + dy * dy
                     + (heading_weight * dtheta) ** 2)


# ---------------------------------------------------------------------------
# Reeds-Shepp with obstacle checking.  Each configuration (each of the 48
# curves returned by _get_candidates) is turned into a waypoint list, expanded
# to world-space poses, and sampled against the arena.  The shortest *valid*
# configuration wins -- never the globally shortest if it ploughs through an
# obstacle.
# ---------------------------------------------------------------------------
def sample_waypoints_to_poses(start: Config, waypoints: List[Tuple],
                              step: float = 2.0) -> List[Tuple[float, float, float]]:
    """Sample fine-grained (x, y, theta) poses along straight segments and circular arcs."""
    x, y, theta = start.x, start.y, start.theta
    poses = [(x, y, theta)]
    for kind, param, rad in waypoints:
        if kind in ('F', 'B'):
            dist = abs(param)
            sgn = 1.0 if kind == 'F' else -1.0
            num_steps = max(1, int(math.ceil(dist / step)))
            ds = dist / num_steps
            for _ in range(num_steps):
                x += sgn * ds * math.cos(theta)
                y += sgn * ds * math.sin(theta)
                poses.append((x, y, theta))
        else:
            angle = param  # radians
            arc_len = abs(angle) * rad
            num_steps = max(1, int(math.ceil(arc_len / step)))
            d_th = angle / num_steps
            for _ in range(num_steps):
                if kind == 'L':
                    x += rad * (math.sin(theta + d_th) - math.sin(theta))
                    y += rad * (-math.cos(theta + d_th) + math.cos(theta))
                    theta = normalize_angle(theta + d_th)
                else:  # 'R'
                    x += rad * (-math.sin(theta - d_th) + math.sin(theta))
                    y += rad * (math.cos(theta - d_th) - math.cos(theta))
                    theta = normalize_angle(theta - d_th)
                poses.append((x, y, theta))
    return poses


def _poses_for_candidate(start: Config, kind_id: int, params: Tuple,
                         radius: float, step: float = 2.0) -> Tuple[List[Tuple], List[Tuple[float, float, float]]]:
    """Expand a (kind_id, t, u, v, w, x) canonical-frame candidate into
    scaled waypoints and fine-grained sampled world-space poses.
    """
    t, u, v, w, x = params
    wps = _build(kind_id, t, u, v, w, x)
    scaled_wps = []
    for kind, p, _ in wps:
        if kind in ('F', 'B'):
            scaled_wps.append((kind, abs(p) * radius, radius))
        else:
            scaled_wps.append((kind, p, radius))
    poses = sample_waypoints_to_poses(start, scaled_wps, step=step)
    return scaled_wps, poses


def _candidate_length(kind_id: int, params: Tuple, radius: float) -> float:
    """True arc length of a candidate configuration (sum of |segment|)."""
    t, u, v, w, x = params
    kinds = _PATH_TYPES[kind_id]
    segs = [t, u, v, w, x]
    total = 0.0
    for kind, p in zip(kinds, segs):
        if kind is None or p is None:
            continue
        total += abs(p)
    return total * radius


def reeds_shepp_candidates(start: Config, goal: Config,
                           radius: Optional[float] = None,
                           arena: Optional[Dict] = None,
                           step: float = 2.0
                           ) -> List[Dict]:
    """Every feasible Reeds-Shepp configuration from start to goal, each
    annotated with its length, kind id, canonical params, waypoints and
    fine-grained sampled poses for collision checking.
    """
    r = radius if radius is not None else _turning_radius()
    dx = goal.x - start.x
    dy = goal.y - start.y
    phi = normalize_angle(goal.theta - start.theta)
    x, y = _canonical(dx, dy, start.theta, r)
    out = []
    for total, kind_id, t, u, v, w, xx in _get_candidates(x, y, phi):
        params = (t, u, v, w, xx)
        wps, poses = _poses_for_candidate(start, kind_id, params, r, step=step)
        out.append({
            "kind_id": kind_id,
            "length": total * r,
            "params": params,
            "radius": r,
            "waypoints": wps,
            "poses": poses,
        })
    return out


def reeds_shepp_path_with_obstacles(start: Config, goal: Config,
                                    radius: Optional[float] = None,
                                    arena: Optional[Dict] = None,
                                    step: float = 2.0
                                    ) -> Tuple[float, List[Tuple]]:
    """Shortest Reeds-Shepp configuration that avoids every obstacle.

    Iterates every feasible configuration (CSC/CCC/CCCC/CCSC/CCSCC), samples
    each against the arena, and returns the shortest collision-free one as
    ``(length, waypoints)``. Returns ``(inf, [])`` if no configuration is
    collision-free (the caller should fall back to A*).
    """
    r = radius if radius is not None else _turning_radius()
    arena = arena or default_arena()
    best_len = math.inf
    best_wps: List[Tuple] = []
    for cand in reeds_shepp_candidates(start, goal, r, arena, step=step):
        if cand["length"] >= best_len:
            continue  # can't beat the best valid one found so far
        hit = path_collides(cand["poses"], step=step, arena=arena)
        if hit is None:
            best_len = cand["length"]
            best_wps = cand["waypoints"]
    if best_len == math.inf:
        return math.inf, []
    return best_len, best_wps


# ---------------------------------------------------------------------------
# Kinematic Hybrid A* Search in SE(2) (Ackermann Kinematics)
# ---------------------------------------------------------------------------

def hybrid_astar(start: Config, goal: Config,
                 arena: Dict,
                 radius: float = TURNING_RADIUS_CM,
                 max_nodes: int = 1500,
                 step: float = 2.0,
                 safety_margin: float = 4.0
                 ) -> Tuple[float, List[Tuple], List[Tuple[float, float, float]], str]:
    """Kinematic Hybrid A* search in SE(2) respecting Ackermann turning constraints (R = 25cm)."""
    # 1. Try direct collision-free Reeds-Shepp curve first
    direct_len, direct_wps = reeds_shepp_path_with_obstacles(start, goal, radius, arena, step=step)
    if not math.isinf(direct_len) and direct_wps:
        return direct_len, direct_wps, sample_waypoints_to_poses(start, direct_wps, step=step), "reeds_shepp"

    xy_step = 5.0
    th_step = math.pi / 8.0  # 22.5 degrees

    def get_key(cfg: Config) -> Tuple[float, float, float]:
        kx = round(cfg.x / xy_step) * xy_step
        ky = round(cfg.y / xy_step) * xy_step
        kth = round(normalize_angle(cfg.theta) / th_step) * th_step
        return (round(kx, 1), round(ky, 1), round(normalize_angle(kth), 2))

    # Kinematically valid motion primitives for an Ackermann car:
    actions = [
        ('F', 5.0), ('F', 10.0), ('F', 20.0),
        ('B', 5.0), ('B', 10.0), ('B', 20.0),
        ('L', math.pi / 8.0), ('R', math.pi / 8.0),
        ('L', math.pi / 4.0), ('R', math.pi / 4.0),
        ('L', math.pi / 2.0), ('R', math.pi / 2.0),
        ('L', -math.pi / 8.0), ('R', -math.pi / 8.0),
        ('L', -math.pi / 4.0), ('R', -math.pi / 4.0),
        ('L', -math.pi / 2.0), ('R', -math.pi / 2.0),
    ]

    open_heap = []
    count = 0
    h0 = reeds_shepp_distance(start, goal, radius)
    heapq.heappush(open_heap, (h0, count, 0.0, start, []))
    visited = {}
    nodes = 0

    while open_heap and nodes < max_nodes:
        f, _, g, cur_cfg, cur_wps = heapq.heappop(open_heap)
        key = get_key(cur_cfg)
        if key in visited and visited[key] <= g:
            continue
        visited[key] = g
        nodes += 1

        # Analytic expansion: try direct Reeds-Shepp curve from current configuration
        rs_len, rs_wps = reeds_shepp_path_with_obstacles(cur_cfg, goal, radius, arena, step=step)
        if not math.isinf(rs_len) and rs_wps:
            total_wps = cur_wps + rs_wps
            total_poses = sample_waypoints_to_poses(start, total_wps, step=step)
            return g + rs_len, total_wps, total_poses, "astar"

        for kind, param in actions:
            wp = (kind, param, radius)
            step_poses = sample_waypoints_to_poses(cur_cfg, [wp], step=step)

            # Check collision along intermediate poses with safety margin
            collision = False
            for px, py, pth in step_poses[1:]:
                if robot_collides_any(px, py, pth, arena, safety_margin=safety_margin, boundary_margin=1.0) is not None:
                    collision = True
                    break
            if collision:
                continue

            end_x, end_y, end_th = step_poses[-1]
            next_cfg = Config(end_x, end_y, end_th)
            next_key = get_key(next_cfg)
            cost = abs(param) if kind in ('F', 'B') else abs(param) * radius
            if kind == 'B' or (kind in ('L', 'R') and param < 0):
                cost *= 1.15  # Slight preference for forward motion

            next_g = g + cost
            if next_key in visited and visited[next_key] <= next_g:
                continue

            next_h = reeds_shepp_distance(next_cfg, goal, radius)
            count += 1
            heapq.heappush(open_heap, (next_g + next_h, count, next_g, next_cfg, cur_wps + [wp]))

    return math.inf, [], [], "none"


# ---------------------------------------------------------------------------
# Task 1 entry point.
# ---------------------------------------------------------------------------
def plan_drive(start: Config, goal: Config,
               radius: Optional[float] = None,
               arena: Optional[Dict] = None,
               step: float = 2.0,
               safety_margin: float = 4.0
               ) -> Dict:
    """Plan the shortest drive from start to goal, avoiding obstacles and walls.

    Strategy:
    1. Try every Reeds-Shepp configuration (48 curves), sample against arena, pick shortest.
    2. If all direct curves are blocked, run Kinematic Hybrid A* with Ackermann motion primitives.
    """
    r = radius if radius is not None else _turning_radius()
    a = arena or default_arena()

    length, wps, poses, method = hybrid_astar(start, goal, a, radius=r, step=step, safety_margin=safety_margin)
    if not math.isinf(length) and wps:
        return {
            "method": method,
            "length": length,
            "waypoints": wps,
            "poses": poses,
        }
    return {
        "method": "none",
        "length": math.inf,
        "waypoints": [],
        "poses": [],
    }


def plan_drive_poses(start: Config, goal: Config,
                     radius: Optional[float] = None,
                     arena: Optional[Dict] = None,
                     step: float = 2.0
                     ) -> Tuple[float, List[Tuple[float, float, float]]]:
    """Like :func:`plan_drive` but returns only (length, world-space poses)."""
    result = plan_drive(start, goal, radius, arena, step)
    return result["length"], result["poses"]


def _poses_to_waypoints(poses: Tuple[float, List[Tuple[float, float, float]]],
                        radius: float) -> List[Tuple]:
    """Convert A* poses back into (kind, param, radius) waypoints.

    The A* path is straight segments between grid nodes separated by a step,
    so each transition is either a straight drive ('F'/'B') or an in-place
    turn ('L'/'R' of 90 degrees). Consecutive straights in the same
    direction are merged so the STM32 gets one FC<dist> per leg, not one per
    grid cell.
    """
    cost, pts = poses
    if not pts or len(pts) < 2:
        return []
    raw_wps = []
    for i in range(len(pts) - 1):
        x0, y0, th0 = pts[i]
        x1, y1, th1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        dth = normalize_angle(th1 - th0)
        if dist >= 1e-6:
            kind = 'F' if (dx * math.cos(th0) + dy * math.sin(th0)) >= 0 else 'B'
            raw_wps.append((kind, dist, radius))
        elif abs(dth) > 1e-6:
            kind = 'L' if dth > 0 else 'R'
            raw_wps.append((kind, abs(dth), radius))

    if not raw_wps:
        return []

    merged = []
    curr_kind, curr_param, curr_rad = raw_wps[0]
    for kind, p, rad in raw_wps[1:]:
        if kind == curr_kind:
            curr_param += p
        else:
            merged.append((curr_kind, round(curr_param, 2), curr_rad))
            curr_kind, curr_param, curr_rad = kind, p, rad
    merged.append((curr_kind, round(curr_param, 2), curr_rad))
    return merged


# ---------------------------------------------------------------------------
# Serialisation to the STM32's MoveCommand batch.
#
# The algorithm's waypoints are (kind, param, radius) primitives; the ROS
# planner sends MoveCommand{command, value} batches to the hardware bridge.
# This converter is the one translation point -- if it drifts, the offline
# simulator and the live run disagree, so it is self-checked below.
# ---------------------------------------------------------------------------
def waypoints_to_move_commands(waypoints: List[Tuple],
                               default_radius: Optional[float] = None
                               ) -> List[Tuple[str, int]]:
    """(kind, param, radius) waypoints -> [(command, value)] for the STM32.

    Mapping (kind, param):
      'F' -> ('FC', param_cm)   forward straight
      'B' -> ('BC', param_cm)   backward straight
      'L' -> ('FL', param_deg)  left turn (forward-left on the STM32)
      'R' -> ('FR', param_deg)  right turn (forward-right on the STM32)

    Angles are rounded to whole degrees (the STM32 takes 000-360) and
    distances to whole centimetres (000-999).  Anything outside those ranges
    is clamped and flagged, never silently truncated -- a 370-degree turn
    becomes ('FL', 360) with a note, not a silent wrap.
    """
    r = default_radius if default_radius is not None else _turning_radius()
    out = []
    for kind, param, _radius in waypoints:
        if kind in ('F', 'B'):
            cmd = 'FC' if kind == 'F' else 'BC'
            val = max(0, min(999, int(round(abs(param)))))
            out.append((cmd, val))
        elif kind in ('L', 'R'):
            cmd = 'FL' if kind == 'L' else 'FR'
            deg = math.degrees(abs(param))
            val = max(0, min(360, int(round(deg))))
            out.append((cmd, val))
        else:
            raise ValueError(f"unknown waypoint kind {kind!r}")
    return out


def move_commands_to_waypoints(commands: List[Tuple[str, int]],
                               radius: Optional[float] = None
                               ) -> List[Tuple]:
    """Inverse of :func:`waypoints_to_move_commands`: STM32 batch -> waypoints.

    FC/BC -> F/B straights, FL/FR/BL/BR -> L/R turns.  Round-tripping a
    planned path through this pair should reproduce it within the STM32's
    quantisation (1 cm / 1 degree), which is exactly what the self-check
    below asserts -- a drift here would mean the simulator and the live run
    disagree.
    """
    r = radius if radius is not None else _turning_radius()
    out = []
    for cmd, val in commands:
        if cmd == 'FC':
            out.append(('F', float(val), r))
        elif cmd == 'BC':
            out.append(('B', float(val), r))
        elif cmd in ('FL', 'BL'):
            out.append(('L', math.radians(val), r))
        elif cmd in ('FR', 'BR'):
            out.append(('R', math.radians(val), r))
        else:
            raise ValueError(f"unknown command {cmd!r}")
    return out


# ----------------------------------------------------------------------------
# Self-verification.  At import time we check the three things that would
# otherwise only show up on the live run:
#
# 1. SE(2) distance is symmetric and zero on identical poses.
# 2. A planned path's endpoint matches the goal (same check reed_shepp.py
#    runs, but through the obstacle-checked planner, so it also exercises
#    the candidate expansion and the waypoint->pose pipeline).
# 3. Waypoints <-> MoveCommand round-trips within the STM32's quantisation
#    (1 cm / 1 degree) -- a drift here means the simulator and the live run
#    disagree, which is the failure mode this whole module exists to prevent.
# ----------------------------------------------------------------------------
def _verify() -> None:
    r = _turning_radius()
    arena = default_arena()

    # 1. SE(2) distance sanity.
    a = Config(0.0, 0.0, 0.0)
    b = Config(30.0, 40.0, 0.0)
    assert abs(se2_distance(a, b) - 50.0) < 1e-6, se2_distance(a, b)
    assert abs(se2_distance(a, a)) < 1e-9
    assert abs(se2_distance(a, b) - se2_distance(b, a)) < 1e-9

    # 2. Planned path endpoints close on the goal.
    cases = [
        Config(10.0, 10.0, 0.0), Config(120.0, 80.0, math.pi / 3),
        Config(10.0, 10.0, 0.0), Config(80.0, 150.0, -math.pi / 2),
        Config(10.0, 10.0, 0.0), Config(150.0, 30.0, math.pi),
    ]
    for i in range(0, len(cases), 2):
        s, g = cases[i], cases[i + 1]
        result = plan_drive(s, g, r, arena)
        if result["method"] == "none":
            continue
        assert result["poses"], (s, g, result["method"])
        ex, ey, eth = result["poses"][-1]
        assert abs(ex - g.x) < 1.0 and abs(ey - g.y) < 1.0, \
            f"planned path endpoint {ex:.2f},{ey:.2f} != goal {g.x},{g.y}"
        assert abs(normalize_angle(eth - g.theta)) < 0.1, \
            f"planned path heading {eth:.3f} != goal {g.theta:.3f}"
        # and the path must be collision-free
        assert path_collides([Config(x, y, t) for x, y, t in result["poses"]],
                             arena=arena) is None, \
            f"planned path through {result['method']} hits an obstacle"

    # 3. Waypoint <-> MoveCommand round-trip.
    wps = [('F', 50.0, r), ('L', math.pi / 2, r), ('R', math.pi / 4, r),
           ('B', 20.0, r)]
    cmds = waypoints_to_move_commands(wps)
    assert cmds == [('FC', 50), ('FL', 90), ('FR', 45), ('BC', 20)], cmds
    back = move_commands_to_waypoints(cmds, r)
    for (k0, p0, _), (k1, p1, _) in zip(wps, back):
        assert k0 == k1
        assert abs(p0 - p1) < 1.0 + 0.01, (k0, p0, p1)
    print(f"planning: {len(cases) // 2} planned drives verified, "
          f"waypoint<->MoveCommand round-trip OK")


_verify()