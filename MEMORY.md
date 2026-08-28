# MEMORY.md — Project memory for AI agents

Persistent notes about work done in this repo so later agents don't re-derive it.

## Algorithm package (`algorithm/`)

Pure-Python pathfinding + arena simulator (runs on a laptop/PC, no ROS2 dep).
Coordinates are **centimetres**; orientation theta in radians, East = 0,
counter-clockwise positive, in (-pi, pi].  Imports `arena.py` for `Config`,
obstacles, and `TURNING_RADIUS_CM = 25`.

### `algorithm/reed_shepp.py` — Reeds-Shepp shortest path (DONE, verified)

Hand-rolled faithful port of OMPL's `ReedsSheppStateSpace`
(https://raw.githubusercontent.com/ompl/ompl/refs/heads/main/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp,
BSD-licensed, Mark Moll / Reeds & Shepp 1979).  The PyPI `reeds_shepp` package
won't build here (Cython/C++, needs boost) — this is the replacement.

**Public API:**
- `reeds_shepp_path(start, goal, radius=None) -> (length, waypoints)`
  where waypoints are `(kind, param, radius)` tuples, kind in `F`/`B` (forward/
  backward straight) or `L`/`R` (turn).  `radius` defaults to
  `arena.TURNING_RADIUS_CM`.  Returns `(inf, [])` if infeasible.
- `reeds_shepp_distance(start, goal, radius=None) -> float`
- `distance_matrix(poses, radius=None) -> np.ndarray` — (n,n) float64,
  symmetric, zero diagonal, filled in full (both triangles).

**Internal structure:** 8 primitive case formulas (`_LpSpLp`, `_LpSpRp`,
`_LpRmL`, `_LpRupLumRm`, `_LpRumLumRp`, `_LpRmSmLm`, `_LpRmSmRm`,
`_LpRmSLmRp`) → 5 case-group drivers (`_csc`, `_ccc`, `_cccc`, `_ccsc`,
`_ccscc`) that try all **48 curves** (16 path types × 4 symmetries:
identity / timeflip / reflect / both) → `_get_path` picks the shortest.
`_PATH_TYPES` is OMPL's `reedsSheppPathType` table.  Waypoints are built by
`_build`; the canonical-frame transform is in `_canonical`.

**Bugs in the pre-existing file that were fixed (see git history / file):**
1. `_LpSpRp` unpacked `_polar()` as `(t1, u1)` but `_polar` returns
   `(r, theta)` — it squared the *angle* instead of the radius, so
   reflect-symmetric paths never closed.  Fixed: `u1, t1 = _polar(...)`.
2. `_turn('R', ...)` sign-flipped both x and y displacements vs OMPL's
   `interpolate()` RS_RIGHT case.
3. `_canonical` used the relative heading `th2 - th1` where OMPL uses the
   **start orientation** `th1` for the rotation.  `_get_path` receives the
   relative heading separately.

**Self-verification runs at import time** (`_verify()`): walks the shortest
path and asserts the endpoint matches the goal to 1e-3.  If a sign error is
ever introduced, `import reed_shepp` will fail loudly.

**Tested:** compiles clean; 500 random pose pairs in a 200×200 cm arena,
500/500 feasible, **0 endpoint failures** (within 1 cm / 1e-2 rad).
Distance matrix confirmed symmetric with zero diagonal.

### `algorithm/arena.py` — arena definition (source of truth)

`default_arena()` returns start zone + 5 placeholder obstacles.  Also
`in_bounds`, `in_start_zone`, `obstacle_footprint`, `point_collides_obstacle`,
`collides_any`, `image_positions`, `to_config`.  Used by both the simulator
and `mdp_bringup/planner_core.py` so the offline sim and live run can't
drift apart.

## Repo map

- `algorithm/` — this package.  `requirements.txt`: numpy>=1.24, pygame>=2.0,
  scipy>=1.10 (scipy not actually imported anywhere yet).
- `ros2_ws/` — ROS2 workspace; `mdp_bringup/planner_core.py` is the ROS
  planner that imports `algorithm/` (not yet present).
- `stm32/` — STM32F407 firmware (PlatformIO).
- `raspberry-pi/cv/` — CV inference on the Pi.
- `docs/` — `protocol.md`, `stm32-uart-protocol-spec.md`,
  `cv-integration-checklist.md`, `week2-checklist.md`.

## TODO / next steps for pathfinding

- Wire `reed_shepp_path` / `distance_matrix` into `mdp_bringup/planner_core.py`
  (the ROS planner that currently doesn't exist yet — it's listed as importing
  `algorithm/` but the file is absent).
- Obstacle avoidance: Reeds-Shepp gives geometric shortest paths but ignores
  obstacles; the planner needs to add footprint clearance (the briefing's
  30 cm padded planning footprint) or a search over the configuration space.
- Discretisation: waypoints are `(kind, param, radius)` primitives; the STM32
  protocol wants `FC<dist>` / `FL<deg>` / etc. — a converter from waypoints
  to the 5-byte UART packets is needed (see `docs/stm32-uart-protocol-spec.md`).