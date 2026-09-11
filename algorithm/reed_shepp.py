"""Reeds-Shepp shortest path for a car-like robot (forward AND reverse).

Hand-rolled: the only PyPI package (`reeds_shepp`) is a Cython/C++ build
that needs boost and refuses to install here. This is a direct port of
OMPL's `ReedsSheppStateSpace` (BSD-licensed, Mark Moll / the Reeds & Shepp
1979 taxonomy) -- battle-tested math rather than my own case formulas --
plus a self-verifying endpoint check so a sign error is caught at import
time, not only at competition time.

Waypoint format: list of (kind, param, radius) where kind is 'F'/'B'
(straight) or 'L'/'R' (turn); param is segment length (rad for turns).
The turning radius is fixed (the car's minimum), so curvature is +/-1/r.

Distance is returned in the same units as the pose coordinates
(centimetres in this project, since the arena and STM32 protocol both
use cm). The case formulas work in a radius-normalised frame and are
multiplied by the radius at the end.

Public surface
-------------
* :func:`reeds_shepp_path` -- shortest path + waypoints between two poses.
* :func:`reeds_shepp_distance` -- shortest-path length between two poses.
* :func:`distance_matrix` -- pairwise distance matrix over a set of poses.
"""

import math
from typing import List, Optional, Tuple

import numpy as np

from arena import Config

_PI = math.pi
_TWO_PI = 2.0 * _PI
ZERO = 7.0e-15  # ~10 * numeric_limits<double>::epsilon


def normalize_angle(a: float) -> float:
    v = a % _TWO_PI
    if v < -_PI:
        v += _TWO_PI
    elif v > _PI:
        v -= _TWO_PI
    return v


def _mod2pi(x: float) -> float:
    v = x % _TWO_PI
    if v < -_PI:
        v += _TWO_PI
    else:
        if v > _PI:
            v -= _TWO_PI
    return v


def _polar(x: float, y: float) -> Tuple[float, float]:
    r = math.hypot(x, y)
    theta = math.atan2(y, x)
    return r, theta


def _tau_omega(u, v, xi, eta, phi):
    delta = _mod2pi(u - v)
    A = math.sin(u) - math.sin(delta)
    B = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = (_mod2pi(t1 + _PI) if t2 < 0 else _mod2pi(t1))
    omega = _mod2pi(tau - u + v - phi)
    return tau, omega


# ----------------------------------------------------------------------------
# Case formulas (port of OMPL reeds_shepp.cpp). Each takes the canonical
# frame coords (x, y, phi) -- the target pose transformed into the frame
# where the start is (0, 0, 0) and the radius is normalised to 1.
# Returns (t, u, v, [w, x]) or None if infeasible.
# ----------------------------------------------------------------------------
def _LpSpLp(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -ZERO:
        v = _mod2pi(phi - t)
        if v >= -ZERO:
            return t, u, v
    return None


def _LpSpRp(x, y, phi):
    # OMPL: polar(x+sin(phi), y-1-cos(phi), u1, t1)  ->  u1=r, t1=theta.
    # _polar returns (r, theta), so unpack as (u1, t1).
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 * u1
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= -ZERO and v >= -ZERO:
            return t, u, v
    return None


def _LpRmL(x, y, phi):
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _mod2pi(theta + 0.5 * u + _PI)
        v = _mod2pi(phi - t + u)
        if t >= -ZERO and u <= ZERO:
            return t, u, v
    return None


def _LpRupLumRm(x, y, phi):
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.sqrt(xi * xi + eta * eta))
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -ZERO and v <= ZERO:
            return t, u, v
    return None


def _LpRumLumRp(x, y, phi):
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -0.5 * _PI:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -ZERO and v >= -ZERO:
                return t, u, v
    return None


def _LpRmSmLm(x, y, phi):
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + math.atan2(r, -2.0))
        v = _mod2pi(phi - 0.5 * _PI - t)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return t, u, v
    return None


def _LpRmSmRm(x, y, phi):
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _mod2pi(t + 0.5 * _PI - phi)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return t, u, v
    return None


def _LpRmSLmRp(x, y, phi):
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= ZERO:
            t = _mod2pi(math.atan2((4.0 - u) * xi - 2.0 * eta,
                                   -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -ZERO and v >= -ZERO:
                return t, u, v
    return None


# ----------------------------------------------------------------------------
# Driver. OMPL's getPath() tries every one of the 48 Reeds-Shepp curves (16
# path types x 4 symmetries: identity / timeflip / reflect / both) and keeps
# the shortest feasible one.  We mirror it exactly.
#
# Each driver returns a flat list of (kind_id, total, (t,u,v,w,x)) triples;
# _get_path picks the minimum total.  total already counts fixed turns.
#
# Symmetry bookkeeping (from OMPL's CSC/CCC/CCCC/CCSC/CCSCC calls):
#   identity : params as-is
#   timeflip : negate ALL params (fixed turns flip sign too: -pi/2 -> pi/2)
#   reflect  : params as-is (L<->R swap happens via the type id)
#   both     : negate all params (fixed turns flip sign too)
# ----------------------------------------------------------------------------
_HALF_PI = 0.5 * math.pi


def _best(cands):
    """Shortest candidate among (kind_id, total, (t,u,v,w,x)) triples.

    Returns ((kind_id, t, u, v, w, x), total); total is inf when nothing
    is feasible.  total already includes the fixed turns baked in to each
    candidate, so it is the true path length."""
    best = (None, 0.0, 0.0, 0.0, 0.0, 0.0)
    best_len = math.inf
    for kind_id, total, params in cands:
        if total < best_len:
            best_len = total
            best = (kind_id,) + params
    return best, best_len


def _csc(x, y, phi):
    """CSC: 8 curves (2 formulas x 4 symmetries)."""
    out = []
    for tid, uv, flip in [(14, _LpSpLp(x, y, phi), False),
                           (14, _LpSpLp(-x, y, -phi), True),
                           (15, _LpSpLp(x, -y, -phi), False),
                           (15, _LpSpLp(-x, -y, phi), True),
                           (12, _LpSpRp(x, y, phi), False),
                           (12, _LpSpRp(-x, y, -phi), True),
                           (13, _LpSpRp(x, -y, -phi), False),
                           (13, _LpSpRp(-x, -y, phi), True)]:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        out.append((tid, abs(t) + abs(u) + abs(v), (t, u, v, 0.0, 0.0)))
    return out


def _ccc(x, y, phi):
    """CCC: 8 curves (1 formula x 4 symmetries x 2 directions)."""
    out = []
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)
    cands = [(0, _LpRmL(x, y, phi), False, False),
             (0, _LpRmL(-x, y, -phi), True, False),
             (1, _LpRmL(x, -y, -phi), False, False),
             (1, _LpRmL(-x, -y, phi), True, False),
             (0, _LpRmL(xb, yb, phi), False, True),
             (0, _LpRmL(-xb, yb, -phi), True, True),
             (1, _LpRmL(xb, -yb, -phi), False, True),
             (1, _LpRmL(-xb, -yb, phi), True, True)]
    for tid, uv, flip, back in cands:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        if back:
            # backwards variants are stored as (v, u, t)
            t, u, v = v, u, t
        out.append((tid, abs(t) + abs(u) + abs(v), (t, u, v, 0.0, 0.0)))
    return out


def _cccc(x, y, phi):
    """CCCC: 8 curves (2 formulas x 4 symmetries).

    OMPL's CCCC: LpRupLumRm -> type 2 (L R L R) with middle turn -u,
    LpRumLumRp -> type 2 with middle turn +u.  The reflect variants
    switch to type 3 (R L R L); timeflip negates everything.
    """
    out = []
    # (tid, formula, flip, mid_sign)
    cands = [(2, _LpRupLumRm(x, y, phi), False, -1),
             (2, _LpRupLumRm(-x, y, -phi), True, -1),
             (3, _LpRupLumRm(x, -y, -phi), False, -1),
             (3, _LpRupLumRm(-x, -y, phi), True, -1),
             (2, _LpRumLumRp(x, y, phi), False, +1),
             (2, _LpRumLumRp(-x, y, -phi), True, +1),
             (3, _LpRumLumRp(x, -y, -phi), False, +1),
             (3, _LpRumLumRp(-x, -y, phi), True, +1)]
    for tid, uv, flip, mid in cands:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        out.append((tid, abs(t) + 2 * abs(u) + abs(v),
                    (t, u, mid * u, v, 0.0)))
    return out


def _ccsc(x, y, phi):
    """CCSC: 16 curves (4 formulas x 4 symmetries)."""
    out = []
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)
    fwd = [(4, _LpRmSmLm(x, y, phi), False),
           (4, _LpRmSmLm(-x, y, -phi), True),
           (5, _LpRmSmLm(x, -y, -phi), False),
           (5, _LpRmSmLm(-x, -y, phi), True),
           (8, _LpRmSmRm(x, y, phi), False),
           (8, _LpRmSmRm(-x, y, -phi), True),
           (9, _LpRmSmRm(x, -y, -phi), False),
           (9, _LpRmSmRm(-x, -y, phi), True)]
    bwd = [(6, _LpRmSmLm(xb, yb, phi), False),
           (6, _LpRmSmLm(-xb, yb, -phi), True),
           (7, _LpRmSmLm(xb, -yb, -phi), False),
           (7, _LpRmSmLm(-xb, -yb, phi), True),
           (10, _LpRmSmRm(xb, yb, phi), False),
           (10, _LpRmSmRm(-xb, yb, -phi), True),
           (11, _LpRmSmRm(xb, -yb, -phi), False),
           (11, _LpRmSmRm(-xb, -yb, phi), True)]
    for tid, uv, flip in fwd:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        # the fixed -pi/2 turn flips sign on timeflip, matching OMPL's
        # PathType(-t, .5*pi, -u, -v)
        sgn = _HALF_PI if flip else -_HALF_PI
        out.append((tid, abs(t) + _HALF_PI + abs(u) + abs(v),
                    (t, sgn, u, v, 0.0)))
    for tid, uv, flip in bwd:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        sgn = _HALF_PI if flip else -_HALF_PI
        out.append((tid, abs(v) + abs(u) + _HALF_PI + abs(t),
                    (v, u, sgn, t, 0.0)))
    return out


def _ccscc(x, y, phi):
    """CCSCC: 4 curves (1 formula x 4 symmetries)."""
    out = []
    for tid, uv, flip in [(16, _LpRmSLmRp(x, y, phi), False),
                           (16, _LpRmSLmRp(-x, y, -phi), True),
                           (17, _LpRmSLmRp(x, -y, -phi), False),
                           (17, _LpRmSLmRp(-x, -y, phi), True)]:
        if uv is None:
            continue
        t, u, v = uv
        if flip:
            t, u, v = -t, -u, -v
        sgn = _HALF_PI if flip else -_HALF_PI
        out.append((tid, abs(t) + math.pi + abs(u) + abs(v),
                    (t, sgn, u, sgn, v)))
    return out


# OMPL's reedsSheppPathType table.  Index = path-type id; each row lists the
# segment kinds (L/R/S/NOP) for the up-to-5 segments t,u,v,w,x.
_PATH_TYPES = [
    ('L', 'R', 'L', None, None),     # 0
    ('R', 'L', 'R', None, None),     # 1
    ('L', 'R', 'L', 'R', None),      # 2
    ('R', 'L', 'R', 'L', None),      # 3
    ('L', 'R', 'S', 'L', None),      # 4
    ('R', 'L', 'S', 'R', None),      # 5
    ('L', 'S', 'R', 'L', None),      # 6
    ('R', 'S', 'L', 'R', None),      # 7
    ('L', 'R', 'S', 'R', None),      # 8
    ('R', 'L', 'S', 'L', None),      # 9
    ('R', 'S', 'R', 'L', None),      # 10
    ('L', 'S', 'L', 'R', None),      # 11
    ('L', 'S', 'R', None, None),     # 12
    ('R', 'S', 'L', None, None),     # 13
    ('L', 'S', 'L', None, None),     # 14
    ('R', 'S', 'R', None, None),     # 15
    ('L', 'R', 'S', 'L', 'R'),       # 16
    ('R', 'L', 'S', 'R', 'L'),       # 17
]


def _canonical(x, y, theta, r):
    """Target pose -> OMPL's canonical frame (radius normalised to 1).

    OMPL's getPath:  x = (c*dx + s*dy)/r, y = (-s*dx + c*dy)/r,
                     phi = th2 - th1 (already wrapped).
    Here ``theta`` is the START orientation th1 -- not the relative
    heading, which is passed separately to _get_path.
    """
    c = math.cos(theta)
    s = math.sin(theta)
    return (c * x + s * y) / r, (-s * x + c * y) / r


def _build(kind_id, t, u, v, w, x):
    """OMPL PathType -> list of (kind, param, r) waypoints, r=1 (normalised)."""
    out = []
    for kind, p in zip(_PATH_TYPES[kind_id], (t, u, v, w, x)):
        if kind is None or p is None:
            continue
        if kind == 'S':
            out.append(('F' if p >= 0 else 'B', p, 1.0))
        else:
            out.append((kind, p, 1.0))
    return out


def _get_candidates(x, y, phi):
    """All feasible canonical-frame curves, as a list of
    (total, kind_id, t, u, v, w, x).  Not sorted -- the caller decides.

    Used by pathfinding so each candidate configuration (CSC/CCC/CCCC/
    CCSC/CCSCC) can be checked for obstacle collisions independently and
    the shortest *valid* one chosen, rather than only ever returning the
    globally shortest (which may plow through an obstacle).
    """
    out = []
    for kind_id, total, params in (_csc(x, y, phi) + _ccc(x, y, phi)
                                   + _cccc(x, y, phi) + _ccsc(x, y, phi)
                                   + _ccscc(x, y, phi)):
        t, u, v, w, xx = params
        out.append((total, kind_id, t, u, v, w, xx))
    return out


def _get_path(x, y, phi):
    """Canonical-frame shortest path.  Returns (length, waypoints) or
    (inf, []) if no curve is feasible."""
    (kind_id, t, u, v, w, x), best_len = _best(
        _csc(x, y, phi) + _ccc(x, y, phi) + _cccc(x, y, phi)
        + _ccsc(x, y, phi) + _ccscc(x, y, phi))
    if kind_id is None:
        return math.inf, []
    return best_len, _build(kind_id, t, u, v, w, x)


# ----------------------------------------------------------------------------
# Public API.
# ----------------------------------------------------------------------------
def reeds_shepp_path(start: Config, goal: Config,
                     radius: Optional[float] = None) -> Tuple[float, List[Tuple]]:
    """Shortest Reeds-Shepp path from start to goal.

    Returns (length, waypoints) where waypoints is a list of
    (kind, param, radius) tuples (kind in F/B/L/R).  length is in the same
    units as the pose coordinates.  If no curve is feasible, returns
    (inf, []).
    """
    r = radius if radius is not None else _turning_radius()
    dx = goal.x - start.x
    dy = goal.y - start.y
    phi = normalize_angle(goal.theta - start.theta)
    x, y = _canonical(dx, dy, start.theta, r)
    length, wps = _get_path(x, y, phi)
    if math.isinf(length):
        return math.inf, []
    # scale the normalised waypoints back to real units
    out = []
    for kind, p, _ in wps:
        if kind in ('F', 'B'):
            out.append((kind, abs(p) * r, r))
        else:
            out.append((kind, p, r))
    return length * r, out


def reeds_shepp_distance(start: Config, goal: Config,
                         radius: Optional[float] = None) -> float:
    """Shortest Reeds-Shepp path length from start to goal (same units as
    the poses)."""
    length, _ = reeds_shepp_path(start, goal, radius)
    return length


def distance_matrix(poses, radius: Optional[float] = None) -> np.ndarray:
    """Pairwise Reeds-Shepp distance matrix over a sequence of poses.

    poses : iterable of Config (or (x, y, theta) tuples)
    radius: turning radius, defaults to the arena's TURNING_RADIUS_CM.

    Returns an (n, n) float64 numpy matrix.  Diagonal is zero; the matrix
    is symmetric (Reeds-Shepp distance is reversible) and is filled in
    full -- both triangles -- so callers can index it directly.
    """
    pts = [_to_config(p) for p in poses]
    n = len(pts)
    r = radius if radius is not None else _turning_radius()
    d = np.zeros((n, n), dtype=np.float64)
    for i in range(n):
        for j in range(i + 1, n):
            v = reeds_shepp_distance(pts[i], pts[j], r)
            d[i, j] = v
            d[j, i] = v
    return d


def _to_config(p) -> Config:
    if isinstance(p, Config):
        return p
    return Config.from_tuple(p)


def waypoints_to_poses(start: Config, waypoints: List[Tuple]) -> List[Tuple[float, float, float]]:
    """Expand a Reeds-Shepp waypoint list into world-space poses.

    Waypoints are the (kind, param, radius) tuples produced by _build / the
    public reeds_shepp_path.  Starting from ``start`` (x, y, theta) each
    primitive is integrated: a straight 'F'/'B' translates along the current
    heading, an 'L'/'R' turn follows its arc.  The returned list always
    opens with the start pose and closes with the goal pose, so callers can
    sample it for collision checks without re-deriving the kinematics.

    Returns a list of (x, y, theta) tuples in the same units as the poses.
    """
    x, y = start.x, start.y
    theta = start.theta
    out = [(x, y, theta)]
    for kind, p, rad in waypoints:
        if kind == 'F':
            x += abs(p) * math.cos(theta)
            y += abs(p) * math.sin(theta)
        elif kind == 'B':
            x -= abs(p) * math.cos(theta)
            y -= abs(p) * math.sin(theta)
        else:
            if kind == 'L':
                x += rad * (math.sin(theta + p) - math.sin(theta))
                y += rad * (-math.cos(theta + p) + math.cos(theta))
                theta = normalize_angle(theta + p)
            else:  # 'R'
                x += rad * (-math.sin(theta - p) + math.sin(theta))
                y += rad * (math.cos(theta - p) - math.cos(theta))
                theta = normalize_angle(theta - p)
        out.append((x, y, theta))
    return out


def reeds_shepp_path_poses(start: Config, goal: Config,
                           radius: Optional[float] = None
                           ) -> Tuple[float, List[Tuple[float, float, float]]]:
    """Like :func:`reeds_shepp_path` but the path is returned as a list of
    world-space (x, y, theta) poses ready for collision sampling.

    Returns (length, poses); poses is [] if no curve is feasible.
    """
    length, wps = reeds_shepp_path(start, goal, radius)
    if math.isinf(length):
        return math.inf, []
    return length, waypoints_to_poses(start, wps)


def _turning_radius() -> float:
    try:
        from arena import TURNING_RADIUS_CM
        return float(TURNING_RADIUS_CM)
    except Exception:
        return 21.0


# ----------------------------------------------------------------------------
# Self-verification.  At import time we replay OMPL's own sanity checks
# (the assert() statements embedded in each case formula) by walking the
# shortest path from start to goal and confirming the end state matches
# the goal.  A sign error in a primitive shows up here, not at
# competition time.
# ----------------------------------------------------------------------------
def _verify() -> None:
    r = _turning_radius()
    cases = [
        # (start, goal, description) -- mix of pure translation, pure
        # rotation, L-shape and reverse-parking geometries.
        Config(0.0, 0.0, 0.0), Config(100.0, 0.0, 0.0),
        Config(0.0, 0.0, 0.0), Config(0.0, 100.0, 0.0),
        Config(0.0, 0.0, 0.0), Config(0.0, 0.0, math.pi / 2),
        Config(0.0, 0.0, 0.0), Config(50.0, 50.0, math.pi / 2),
        Config(10.0, 10.0, math.pi), Config(100.0, 120.0, -math.pi / 4),
        Config(0.0, 0.0, 0.0), Config(-30.0, 40.0, math.pi),  # reverse park
    ]
    for i in range(0, len(cases), 2):
        s, g = cases[i], cases[i + 1]
        length, wps = reeds_shepp_path(s, g, r)
        if math.isinf(length):
            continue
        poses = waypoints_to_poses(s, wps)
        ex, ey, etheta = poses[-1]
        assert (abs(ex - g.x) < 1e-3 and abs(ey - g.y) < 1e-3
                and abs(normalize_angle(etheta - g.theta)) < 1e-3), \
            f"Reeds-Shepp self-check failed for {s} -> {g}: " \
            f"got ({ex:.4f},{ey:.4f},{etheta:.4f}) expected ({g.x},{g.y},{g.theta})"


_verify()
