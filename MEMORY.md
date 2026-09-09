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

## STM32 Firmware (`stm32/`)

- **FreeRTOS Stack Sizing**: `defaultTask` stack was increased from 512B (`128 * 4`) to 2048B (`512 * 4`) and `MotorTask` to 1024B (`256 * 4`). The 512B stack previously overflowed during `snprintf` + floating-point operations and FPU context saving, causing hard MCU lockups.
- **HC-SR04 Ultrasonic Driver**: Uses hardware timer `TIM6` (1 tick = 1 µs at 16 MHz HSI) instead of software loops. Wrapped with `vTaskSuspendAll()` / `xTaskResumeAll()` during echo pulse measurement to prevent ~1 ms FreeRTOS preemption jitter (which created an artificial 19 cm measurement floor). Includes pre-trigger check to avoid hanging when ECHO is stuck HIGH from blind-zone (< 2 cm) reflections.
- **Flashing Flags & DTR Pin Latch**: In `platformio.ini`, `upload_flags` exit sequence MUST be `-rts,-dtr`. A previous trailing `dtr` (`...:-rts,-dtr,dtr`) left DTR asserted, holding the C30D reset/bootloader circuit and preventing execution of user firmware.
- **Boot Gyro Calibration**: MCU runs 5.0 seconds of gyro offset and drift calibration at power-on before enabling USART3 interrupt. Any commands sent during the first 5s will be ignored.
- **UART Busy Priority Check**: In `HAL_UART_RxCpltCallback`, `runRequested == 1` is evaluated *before* `#`. This ensures that incoming trigger packets are rejected with `BUS\r\n` alongside instruction packets when the MCU is busy, preventing empty batches from executing.
- **Motion Dead Time Reduction**:
  - Reduced `comm_task` mechanical settling: `osDelay(200)` -> `osDelay(20)`.
  - Reduced steering servo throw delay: `osDelay(200)` -> `osDelay(80)`.
  - Reduced static friction break: `osDelay(200)` -> `osDelay(50)`.
  - Reduced post-stop settling: `osDelay(100)` -> `osDelay(20)`.
  - Reduced post-turn settling & centering: `200 ms` -> `40 ms`.
  - Adaptive creep: for commands <= 15 (cm/deg), creep threshold is 2 cm / 1 deg instead of 5 cm / 4 deg.
  - Motion timeouts: all movement loops include hardware tick timeouts to prevent MCU hangs on wheel slip.

## ROS 2 Bridges & Teleoperation (`ros2_ws/`)

- **1-Deep Move Queue in `android_bridge_node`**: When direction buttons are held on Android (streamed every 50–60 ms), in-flight moves buffer the newest command in `self._pending_move` and suppress noisy `BUSY_LOCAL` status messages. On move completion callback, pending moves chain immediately without returning to idle.
- **Serial Bridge Retry & Buffer Management**: `serial_bridge_node` flushes the serial input buffer once before the retry loop, and drains residual output with backoff upon receiving `BUS\r\n`, preventing buffer wipes of incoming `RUN\r\n` handshakes.
- **Checklist C.10 Android Pose Streaming**: `android_bridge_node` maps `/robot_pose` to `ROBOT,<x>,<y>,<dir>` with heading mapped to cardinal `N`/`S`/`E`/`W` and grid cells $[0..19]$.
- **Task 1 Autonomous Planner & Bull's Eye Orbit Recovery (`planner_node.py`)**: Computes optimal Reeds-Shepp TSP tour from Android `ALG:{...}` layout. If `/perception/sample_target` returns Bull's Eye marker (ID 40) or low confidence, triggers 90° orbit around obstacle footprint to candidate adjacent faces until the true target is confirmed.
- **Task 2 Fastest Car Reactive Sprint (`fastest_car_node.py`, `task2.launch.py`)**: Autonomous sprint FSM for the unmapped 2-obstacle slalom track. Drives forward using ultrasonic feedback, calls `/perception/sample_target` to classify Left Arrow (39) vs Right Arrow (38), executes calibrated S-curve bypass maneuvers (`slalom_left_cmds`/`slalom_right_cmds`), rounds Obstacle 2, and sprints back into the Carpark. Triggered via tablet "SP" button (`STM:sp`) or `pixi run -e pi task2`.
- **Operating Mode Switching**:
  - **Checklist Mode**: `pixi run -e pi teleop` (manual driving, sensor tests, C.10 pose streaming).
  - **Task 1 Mode**: `pixi run -e pi robot` (autonomous exploration, 4-8 obstacles).
  - **Task 2 Mode**: `pixi run -e pi task2` (fastest car reactive sprint).
  - **Runtime Switching**: `android_bridge_node` routes `ALG:START` to `planner_node` and `STM:sp`/`SP` to `fastest_car_node` without node restarts.


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