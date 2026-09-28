"""Bounded hybrid A* using the same primitives the Pi motion service accepts.

Collision checking samples the complete executable arcs, including reverse.
No existing project's map, global constants or code are imported.
"""
from dataclasses import dataclass
import heapq
import itertools
import math
import time

from .geometry import Pose, wrap


@dataclass(frozen=True)
class Move:
    command: str
    value: int


def rollout(pose, move, radius=.21, sample_step=.004):
    direction = 1 if move.command[0] == 'F' else -1
    steering = {'C': 0, 'L': 1, 'R': -1}[move.command[1]]
    length = move.value/100 if not steering else radius*math.radians(move.value)
    count = max(1, math.ceil(length/sample_step))
    points = []
    for i in range(1, count+1):
        travel = direction*length*i/count
        if steering:
            curvature = steering/radius
            yaw = pose.yaw + travel*curvature
            x = pose.x + (math.sin(yaw)-math.sin(pose.yaw))/curvature
            y = pose.y - (math.cos(yaw)-math.cos(pose.yaw))/curvature
        else:
            yaw = pose.yaw
            x = pose.x + travel*math.cos(yaw)
            y = pose.y + travel*math.sin(yaw)
        points.append(Pose(x, y, wrap(yaw)))
    return points


def rectangle(x, y, yaw, length, width):
    cs, sn = math.cos(yaw), math.sin(yaw)
    return [(x+cs*dx-sn*dy, y+sn*dx+cs*dy) for dx, dy in
            [(-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2)]]


def overlaps(a, b):
    for poly in (a, b):
        for i in range(4):
            p, q = poly[i], poly[(i+1) % 4]
            axis = (-(q[1]-p[1]), q[0]-p[0])
            pa = [v[0]*axis[0]+v[1]*axis[1] for v in a]
            pb = [v[0]*axis[0]+v[1]*axis[1] for v in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Scene:
    def __init__(self, bounds, obstacles, width=.19, length=.23, margin=.03,
                 centre_offset=(0.0, 0.0)):
        self.bounds = bounds
        self.width, self.length, self.margin = width, length, margin
        self.centre_offset = centre_offset
        self.polygons = [rectangle(o.x, o.y, o.yaw, o.size, o.size) for o in obstacles]
        self.obstacles = [(o.x, o.y, math.cos(o.yaw), math.sin(o.yaw), o.size/2) for o in obstacles]
        self.half_length, self.half_width = length/2+margin, width/2+margin

    def clear(self, pose):
        dx, dy = self.centre_offset
        cs, sn = math.cos(pose.yaw), math.sin(pose.yaw)
        x = pose.x+cs*dx-sn*dy
        y = pose.y+sn*dx+cs*dy
        hl, hw = self.half_length, self.half_width
        extent_x, extent_y = hl*abs(cs)+hw*abs(sn), hl*abs(sn)+hw*abs(cs)
        xmin, ymin, xmax, ymax = self.bounds
        if x-extent_x <= xmin or x+extent_x >= xmax or y-extent_y <= ymin or y+extent_y >= ymax:
            return False
        # Exact rectangle SAT on four axes, without constructing/projecting
        # vertex lists for every 4 mm sample. No coarser sampling is introduced.
        for ox, oy, co, so, half in self.obstacles:
            tx, ty = ox-x, oy-y
            dot_u, dot_v = abs(cs*co+sn*so), abs(sn*co-cs*so)
            if abs(tx*cs+ty*sn) > hl+half*(dot_u+dot_v):
                continue
            if abs(-tx*sn+ty*cs) > hw+half*(dot_u+dot_v):
                continue
            if abs(tx*co+ty*so) > half+hl*dot_u+hw*dot_v:
                continue
            if abs(-tx*so+ty*co) > half+hl*dot_v+hw*dot_u:
                continue
            return False
        return True

    def clear_move(self, pose, move, radius):
        return self.clear(pose) and all(self.clear(p) for p in rollout(pose, move, radius))


@dataclass
class Plan:
    moves: list
    poses: list
    reason: str
    expanded: int = 0


def at_goal(pose, goal, position_tolerance=.035, angle_tolerance=math.radians(12)):
    return (math.hypot(pose.x-goal.x, pose.y-goal.y) <= position_tolerance and
            abs(wrap(pose.yaw-goal.yaw)) <= angle_tolerance)


def plan_route(start, goal, scene, radius=.21, allow_reverse=False,
               max_seconds=3.0, max_expanded=18000):
    if radius < .21 or not math.isfinite(radius):
        return Plan([], [], 'invalid_turning_radius')
    if not scene.clear(start) or not scene.clear(goal):
        return Plan([], [], 'start_or_goal_not_clear')
    if at_goal(start, goal):
        return Plan([], [start], 'already_at_goal')
    actions = [Move('FC', 5), Move('FL', 15), Move('FR', 15)]
    if allow_reverse:
        actions += [Move('BC', 5), Move('BL', 15), Move('BR', 15)]

    def key(p):
        return (round(p.x/.025), round(p.y/.025), round(wrap(p.yaw)/math.radians(15)))

    def heuristic(p):
        # Weighted A* prioritises useful progress; shortest-path optimality is not claimed.
        return 1.5*math.hypot(p.x-goal.x, p.y-goal.y) + .08*abs(wrap(p.yaw-goal.yaw))

    counter = itertools.count()
    first = next(counter)
    queue = [(heuristic(start), first)]
    nodes = {first: (start, None, None, 0.0)}
    costs = {key(start): 0.0}
    deadline = time.monotonic()+max_seconds
    expanded = 0
    while queue and expanded < max_expanded:
        if time.monotonic() > deadline:
            return Plan([], [], 'planning_time_limit', expanded)
        _, identity = heapq.heappop(queue)
        pose, parent, move, cost = nodes[identity]
        if cost > costs.get(key(pose), math.inf)+1e-9:
            continue
        expanded += 1
        if at_goal(pose, goal):
            moves = []
            current = identity
            while nodes[current][1] is not None:
                p, parent, move, _ = nodes[current]
                moves.append(move)
                current = parent
            moves.reverse()
            poses = [start]
            for move in moves:
                poses.extend(rollout(poses[-1], move, radius))
            return Plan(moves, poses, 'ok', expanded)
        for move in actions:
            points = rollout(pose, move, radius)
            if not all(scene.clear(p) for p in points):
                continue
            end = points[-1]
            distance = move.value/100 if move.command[1] == 'C' else radius*math.radians(move.value)
            extra = distance*(1.4 if move.command[0] == 'B' else 1.0)
            if move.command[1] != 'C':
                extra *= 1.05
            new_cost = cost+extra
            if new_cost+1e-9 >= costs.get(key(end), math.inf):
                continue
            costs[key(end)] = new_cost
            child = next(counter)
            nodes[child] = (end, identity, move, new_cost)
            heapq.heappush(queue, (new_cost+heuristic(end), child))
    return Plan([], [], 'no_route_within_search_limit', expanded)
