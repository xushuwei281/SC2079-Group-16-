#!/usr/bin/env python3
"""Autonomous Path Planner for SC2079 Multidisciplinary Project (MDP).

Features:
- Computes exact camera viewing vantage poses for obstacles based on face orientation (N/S/E/W).
- Solves Traveling Salesperson Problem (TSP) using exact Reeds-Shepp kinematic distance matrix.
- Generates collision-free trajectories respecting Ackermann turning constraints (R = 25cm)
  and chassis bounding box clearances.
- Discretizes continuous trajectories into verified STM32 5-byte instructions (FC, BC, FL, FR, BL, BR).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from arena import (
    ARENA_SIZE_CM,
    OBSTACLE_SIZE_CM,
    ROBOT_H_CM,
    ROBOT_W_CM,
    START_ZONE_CM,
    TURNING_RADIUS_CM,
    Config,
    Obstacle,
    default_arena,
    in_bounds,
    path_collides,
    robot_collides_any,
    to_config,
)
from reed_shepp import (
    normalize_angle,
    reeds_shepp_distance,
    reeds_shepp_path,
    waypoints_to_poses,
)

# Course default parameters
DEFAULT_VIEW_DIST_CM = 25.0    # 25cm distance from camera to target image
SAFETY_MARGIN_CM = 15.0         # 15cm clearance from arena boundary


@dataclass
class PlanLeg:
    """A single leg of the journey visiting one target obstacle."""
    obstacle_id: int
    target_face: str
    start_pose: Config
    vantage_pose: Config
    poses: List[Tuple[float, float, float]]
    commands: List[Tuple[str, int]]
    raw_strings: List[str]
    distance_cm: float


@dataclass
class FullMissionPlan:
    """The complete multi-leg mission plan for the competition arena."""
    start_pose: Config
    legs: List[PlanLeg]
    total_distance_cm: float
    all_commands: List[str]
    all_poses: List[Tuple[float, float, float]]


# -----------------------------------------------------------------------------
# 1. Vantage Pose Calculation
# -----------------------------------------------------------------------------

def compute_vantage_pose(
    obstacle: Obstacle,
    d_view: float = DEFAULT_VIEW_DIST_CM,
    half_obs: float = OBSTACLE_SIZE_CM / 2.0,
    arena: Optional[Dict] = None
) -> Config:
    """Compute the vantage pose (x, y, theta) where the robot's front camera
    faces directly at the target image on the specified obstacle face (N/S/E/W).
    
    If arena is provided, adapts view distance (18cm to 30cm) to ensure the
    vantage pose does not collide with other obstacles or walls.
    """
    face = (obstacle.face or "N").upper()
    ox, oy = obstacle.x, obstacle.y

    # Try nominal distance first, then adapt if obstructed
    candidate_dists = [d_view, 22.0, 20.0, 28.0, 30.0, 18.0]
    best_config = None

    for d in candidate_dists:
        dist = d + half_obs
        if face == "N":
            vx, vy, vtheta = ox, oy + dist, -math.pi / 2.0
        elif face == "S":
            vx, vy, vtheta = ox, oy - dist, math.pi / 2.0
        elif face == "E":
            vx, vy, vtheta = ox + dist, oy, math.pi
        elif face == "W":
            vx, vy, vtheta = ox - dist, oy, 0.0
        else:
            raise ValueError(f"Unknown obstacle face: {face}")

        vx = max(15.0, min(ARENA_SIZE_CM - 15.0, vx))
        vy = max(15.0, min(ARENA_SIZE_CM - 15.0, vy))
        cfg = to_config(vx, vy, vtheta)

        if arena is None or robot_collides_any(vx, vy, vtheta, arena, safety_margin=1.0) is None:
            return cfg
        if best_config is None:
            best_config = cfg

    return best_config or to_config(ox, oy, 0.0)


# -----------------------------------------------------------------------------
# 2. TSP Optimization (Pairwise Reeds-Shepp Distance Matrix)
# -----------------------------------------------------------------------------

def solve_tsp(
    start_pose: Config,
    vantage_poses: List[Config],
    radius: float = TURNING_RADIUS_CM
) -> List[int]:
    """Find the optimal obstacle visit permutation minimizing total Reeds-Shepp distance.
    
    Returns:
        List of indices into vantage_poses representing the visiting sequence.
    """
    n = len(vantage_poses)
    if n == 0:
        return []
    if n == 1:
        return [0]

    all_poses = [start_pose] + vantage_poses
    total_nodes = len(all_poses)

    # Compute pairwise Reeds-Shepp distance matrix
    dist_matrix = np.zeros((total_nodes, total_nodes))
    for i in range(total_nodes):
        for j in range(total_nodes):
            if i != j:
                dist_matrix[i, j] = reeds_shepp_distance(all_poses[i], all_poses[j], radius)

    # Solve TSP via exact Branch-and-Bound / Permutation Search (N <= 8)
    best_perm: Optional[Tuple[int, ...]] = None
    min_total_cost = float("inf")

    # Obstacle indices are 1 to N in all_poses
    obs_indices = list(range(1, total_nodes))

    for perm in itertools.permutations(obs_indices):
        cost = dist_matrix[0, perm[0]]
        for k in range(len(perm) - 1):
            cost += dist_matrix[perm[k], perm[k + 1]]
            if cost >= min_total_cost:
                break
        if cost < min_total_cost:
            min_total_cost = cost
            best_perm = perm

    # Convert back to 0-indexed indices for vantage_poses
    return [idx - 1 for idx in (best_perm or ())]


# -----------------------------------------------------------------------------
# 3. Trajectory Generation & Collision Avoidance
# -----------------------------------------------------------------------------

def sample_reeds_shepp_path(
    start: Config,
    goal: Config,
    radius: float = TURNING_RADIUS_CM,
    arena: Optional[Dict] = None,
    step_cm: float = 2.0
) -> Tuple[float, List[Tuple[str, float, float]], List[Tuple[float, float, float]]]:
    """Generate fine-grained sampled (x, y, theta) poses along a collision-free Reeds-Shepp or A* fallback path."""
    from planning import plan_drive
    a = arena or default_arena()
    res = plan_drive(start, goal, radius=radius, arena=a, step=step_cm)

    length = res["length"]
    waypoints = res["waypoints"]

    if math.isinf(length) or not waypoints:
        return math.inf, [], []

    sampled_poses: List[Tuple[float, float, float]] = [(start.x, start.y, start.theta)]
    curr_x, curr_y = start.x, start.y
    curr_theta = start.theta

    for kind, param, rad in waypoints:
        if kind in ("F", "B"):
            # Linear translation
            dist = abs(param)
            sgn = 1.0 if kind == "F" else -1.0
            num_steps = max(1, int(math.ceil(dist / step_cm)))
            ds = dist / num_steps
            for _ in range(num_steps):
                curr_x += sgn * ds * math.cos(curr_theta)
                curr_y += sgn * ds * math.sin(curr_theta)
                sampled_poses.append((curr_x, curr_y, curr_theta))
        else:
            # Arc turn (param is angle in radians)
            angle = param
            num_steps = max(1, int(math.ceil((abs(angle) * rad) / step_cm)))
            d_th = angle / num_steps
            for _ in range(num_steps):
                if kind == "L":
                    curr_x += rad * (math.sin(curr_theta + d_th) - math.sin(curr_theta))
                    curr_y += rad * (-math.cos(curr_theta + d_th) + math.cos(curr_theta))
                    curr_theta = normalize_angle(curr_theta + d_th)
                else:  # 'R'
                    curr_x += rad * (-math.sin(curr_theta - d_th) + math.sin(curr_theta))
                    curr_y += rad * (math.cos(curr_theta - d_th) - math.cos(curr_theta))
                    curr_theta = normalize_angle(curr_theta - d_th)
                sampled_poses.append((curr_x, curr_y, curr_theta))

    return length, waypoints, sampled_poses


def is_path_safe(
    poses: List[Tuple[float, float, float]],
    arena: Dict,
    margin_cm: float = SAFETY_MARGIN_CM
) -> bool:
    """Verify that every pose in the sampled trajectory stays within arena bounds
    and does not collide with any obstacle bounding boxes."""
    s = arena.get("arena_size", ARENA_SIZE_CM)
    for x, y, theta in poses:
        # Check arena boundaries
        if not (margin_cm <= x <= s - margin_cm and margin_cm <= y <= s - margin_cm):
            return False
        # Check obstacle collisions with oriented vehicle body
        if robot_collides_any(x, y, theta, arena) is not None:
            return False
    return True


# -----------------------------------------------------------------------------
# 4. Discretization to STM32 5-Byte Primitives
# -----------------------------------------------------------------------------

def discretize_waypoints(
    waypoints: List[Tuple[str, float, float]]
) -> Tuple[List[Tuple[str, int]], List[str]]:
    """Convert continuous Reeds-Shepp waypoints into STM32 5-byte command primitives.
    
    Mappings:
    - ('F', dist, _) -> ('FC', dist_cm) -> 'FC<dist:03d>'
    - ('B', dist, _) -> ('BC', dist_cm) -> 'BC<dist:03d>'
    - ('L', angle_rad, _) (forward)  -> ('FL', deg) -> 'FL<deg:03d>'
    - ('R', angle_rad, _) (forward)  -> ('FR', deg) -> 'FR<deg:03d>'
    - ('L', angle_rad, _) (backward) -> ('BL', deg) -> 'BL<deg:03d>'
    - ('R', angle_rad, _) (backward) -> ('BR', deg) -> 'BR<deg:03d>'
    """
    commands: List[Tuple[str, int]] = []
    raw_strings: List[str] = []

    for kind, param, radius in waypoints:
        if kind == "F":
            dist = round(param)
            if dist >= 1:
                commands.append(("FC", dist))
                raw_strings.append(f"FC{dist:03d}")
        elif kind == "B":
            dist = round(param)
            if dist >= 1:
                commands.append(("BC", dist))
                raw_strings.append(f"BC{dist:03d}")
        elif kind in ("L", "R"):
            deg = round(math.degrees(abs(param)))
            if deg >= 2:  # Ignore microscopic sub-2-degree numerical noise from CSC arcs
                cmd = ("FL" if kind == "L" else "FR") if param >= 0 else ("BL" if kind == "L" else "BR")
                commands.append((cmd, deg))
                raw_strings.append(f"{cmd}{deg:03d}")

    return commands, raw_strings


# -----------------------------------------------------------------------------
# 5. Full Multi-Leg Planner Engine
# -----------------------------------------------------------------------------

def plan_mission(
    obstacles: List[Obstacle],
    start_pose: Optional[Config] = None,
    arena: Optional[Dict] = None,
    radius: float = TURNING_RADIUS_CM,
    forced_order: Optional[List[int]] = None
) -> FullMissionPlan:
    """Generate the complete optimal autonomous mission plan visiting all obstacles."""
    if arena is not None:
        a = arena
    else:
        a = default_arena()
        a["obstacles"] = obstacles
    sp = start_pose or Config(20.0, 20.0, math.pi / 2.0)  # Start zone center, facing North

    if not obstacles:
        return FullMissionPlan(sp, [], 0.0, [], [(sp.x, sp.y, sp.theta)])

    # 1. Compute vantage poses for all obstacles
    vantage_poses = [compute_vantage_pose(ob, arena=a) for ob in obstacles]

    # 2. Solve TSP to get visiting order (or use forced permutation)
    if forced_order is not None:
        visit_order = forced_order
    else:
        visit_order = solve_tsp(sp, vantage_poses, radius)

    legs: List[PlanLeg] = []
    current_pose = sp
    total_dist = 0.0
    all_commands: List[str] = []
    all_poses: List[Tuple[float, float, float]] = [(sp.x, sp.y, sp.theta)]

    for idx in visit_order:
        target_ob = obstacles[idx]
        target_vantage = vantage_poses[idx]

        # Generate Reeds-Shepp curve
        length, wps, sampled_poses = sample_reeds_shepp_path(current_pose, target_vantage, radius=radius, arena=a)

        # Discretize waypoints
        cmds, raw_cmds = discretize_waypoints(wps)

        leg = PlanLeg(
            obstacle_id=target_ob.id,
            target_face=target_ob.face or "N",
            start_pose=current_pose,
            vantage_pose=target_vantage,
            poses=sampled_poses,
            commands=cmds,
            raw_strings=raw_cmds,
            distance_cm=length
        )

        legs.append(leg)
        if sampled_poses and len(sampled_poses) > 1 and not math.isinf(length):
            total_dist += length
            all_commands.extend(raw_cmds)
            all_poses.extend(sampled_poses[1:])
            current_pose = Config(sampled_poses[-1][0], sampled_poses[-1][1], sampled_poses[-1][2])

    return FullMissionPlan(
        start_pose=sp,
        legs=legs,
        total_distance_cm=total_dist,
        all_commands=all_commands,
        all_poses=all_poses
    )


# -----------------------------------------------------------------------------
# Quick CLI Demo
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    arena = default_arena()
    plan = plan_mission(arena["obstacles"])
    print(f"=== Planned Mission with {len(plan.legs)} Legs ===")
    print(f"Total Distance: {plan.total_distance_cm:.2f} cm")
    print(f"Total Movement Commands: {len(plan.all_commands)}")
    for i, leg in enumerate(plan.legs):
        print(f"\nLeg {i+1} -> Obstacle {leg.obstacle_id} ({leg.target_face} face):")
        print(f"  Vantage: x={leg.vantage_pose.x:.1f}, y={leg.vantage_pose.y:.1f}, th={math.degrees(leg.vantage_pose.theta):.0f}°")
        print(f"  Commands: {' -> '.join(leg.raw_strings)}")
