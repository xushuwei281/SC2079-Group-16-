# MEMORY.md — Project memory for AI agents

Persistent notes about work done in this repo so later agents don't re-derive it.

## Continuous velocity control refactor (2026-09-11)

- Android held controls use `VEL:<linear_mps>,<yaw_rps>` and send zero on release.
  The Android bridge publishes `/cmd_vel/teleop`. `motion_controller_node`
  arbitrates teleoperation and `ExecuteMoves`, supervises primitives from live
  telemetry, and publishes the sole base `/cmd_vel` stream. The serial bridge
  consumes that stream and owns UART transport/telemetry/safety, not movement
  completion. The planner's primitive schema remains compatible.
- Normal UART motion is `V` + signed int16 little-endian mm/s + signed int16
  little-endian mrad/s, exactly five bytes. New targets replace old targets;
  movement completion never waits for `FIN`. The MCU's periodic wheel-speed and
  steering loop runs independently of range sampling and telemetry.
- The Pi sends zero after 200 ms without input. The MCU fallback watchdog stops
  nonzero targets after 750 ms without refresh; 300 ms caused observed false
  `STOP:WATCHDOG` trips despite a nominal 50 ms tablet stream. Local
  raw front protection uses US <=12 cm or IR <=10 cm; stale required sensors
  stop forward motion. There is no rear sensor protection. `STOP:WATCHDOG`
  discards the expired Pi target without latching; a fresh command may resume.
  Other `STOP:*` reports require explicit RESET. `Q` latches, zero velocity does
  not reset, and `R` discards old motion state rather than resuming a canceled
  request. After `R`, both Pi layers hold zero until the first fresh TLM frame,
  preventing joystick/reset ordering from creating a false stale-telemetry stop.
- Live logs showed the remaining random latches were mainly `DISCONNECTED` after
  `/dev/ttyACM1` write timeouts, not zero range or a QoS mismatch. Latest-value
  QoS is now best-effort depth 1, bursts are coalesced to 20 Hz UART output, and
  transient write failures reconnect without publishing `/estop`. Sustained
  loss is still stopped by the MCU watchdog and Pi feedback timeout.
- Task 1/2 START does not clear E-STOP. Task 1 no longer automatically resets
  and backs up after a proximity stop; the old recovery distance parameter is
  retained for configuration compatibility. Perception-driven orbit recovery
  remains a separate collision-checked behavior. If direct sampling, legacy
  recognition, and all adjacent-face attempts fail, Task 1 enters
  `MISSION_FAILED`; it does not fabricate or publish a fallback symbol ID.
- The measured full-lock radius is 21–22 cm; planner/controller use 21 cm and
  retain existing calibrated steering endpoints. The absolute-speed PI loop is
  new and requires physical speed/tracking and stopping checks. Firmware build
  and unit tests do not establish braking performance or mean it was flashed.
- **Hardware Calibration & Runout Compensation (2026-09-14)**:
  - Straight distance: `WHEEL_D_CM = 6.33f` in `stm32/src/main.c` (calibrated on `FC 50`).
  - Steering center: `VELOCITY_SERVO_CENTER = 144` in `stm32/include/velocity_control.h` (corrected 3° left tilt).
  - Steering left lock: `VELOCITY_SERVO_LEFT = 94` (`SERVOMIN`) in `stm32/include/velocity_control.h` (corrected ~10° turning radius deficit).
  - 180° Turn Runout: Due to Ackermann entry latency (servo transitioning from center 144 to full lock 94 over the first 2–3 cm of motion), a 180° turn halts ~5 cm short along the final heading line. Handled cleanly in `motion_controller_node.py` via `turn_180_runout_m = 0.05`: when a turn with `mc.value >= 170` completes its rotational yaw goal, it engages a straight forward runout (`angular.z = 0.0`) for 5 cm along the final heading before returning `FIN`.
  - Velocity & Motor Drive Tuning (2026-09-14): Increased `velocity_speed_mps` from 0.15 m/s to 0.25 m/s in `motion_controller_node.py` and `serial_bridge_node.py` (+67% speedup). Deceleration creep floor increased from 0.04 m/s to 0.08 m/s with steeper deceleration slope (`distance_left * 2.0`). Tuned STM32 motor PI loop: `VELOCITY_PWM_FEEDFORWARD = 2.5f` (producing 625 PWM cruise) and `VELOCITY_PWM_KP = 2.0f`.
  - Turning Speed & Inner Wheel Stall Fix (2026-09-14): Turns previously crawled because `distance_left = remaining * radius` caused linear deceleration to start 35° before completion and bottomed out at 0.08 m/s, where differential kinematics dropped the inner wheel to 51 mm/s (<120 PWM), causing inner tire scrub to stall. Fixed in `motion_controller_node.py` by maintaining turn cruise speed (0.25 m/s / ~68 deg/s) until the final 15°, flooring turn speed at 0.18 m/s (~49 deg/s) so the inner wheel always receives >290 PWM (> MOTORLOW 280), keeping both wheels actively powered throughout the turn.
  - Proximity Noise Rejection & Debouncing (2026-09-14): Eliminated false E-STOP triggers in open space. Filtered HC-SR04 transducer ringing glitches (`echo_us < 150` rejected in `HCSR04_ReadCm`). Fixed Sharp IR clamping bug in `IR_RawToCm` and `IR1_RawToCm` (calibrated smoothly down to 6 cm rather than flatlining at the 10 cm trip boundary). Added 8-tick (~80 ms) proximity confirmation debounce in STM32 `motor()` and 2-sample debounce in `planner_node.py` and `fastest_car_node.py`, ensuring single-frame acoustic or motor electrical switching spikes do not cause false hardware E-STOPs.
- See [architecture](ros2_ws/ARCHITECTURE.md), [UART protocol](docs/stm32-uart-protocol-spec.md),
  and [migration/bench guide](docs/cmd-vel-migration.md). Older discrete-batch
  notes below are historical where they conflict with this section.

## Algorithm package (`algorithm/`)

Pure-Python pathfinding + arena simulator (runs on a laptop/PC, no ROS2 dep).
Coordinates are **centimetres**; orientation theta in radians, East = 0,
counter-clockwise positive, in (-pi, pi].  Imports `arena.py` for `Config`,
obstacles, and `TURNING_RADIUS_CM = 21`.

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

- **Continuous Range Safety Ownership**: `motion_controller_node` is the Pi-side
  safety owner for every forward `/cmd_vel` source (teleop, Task 1, and Task 2),
  at 12/10 cm ultrasonic/IR thresholds. It reads the Kalman-filtered
  `/sensors/*` topics (same pipeline `android_bridge_node`'s E-STOP cause
  attribution and the tablet's live readout use) rather than the `*/raw`
  topics, closing a false-latch source where a single-frame reflection spike
  on the raw feed tripped E-STOP with no smoothing (2026-09-11). Firmware
  retains an independent fallback. STM32 range value zero is no-return/
  out-of-range; freshness tracks the sensor task heartbeat so an open arena
  does not produce `STOP:SENSOR_STALE`.
- **Measured Turning Radius & Steering**: Physical full-lock radius is 21–22 cm. The planner
  and continuous velocity controller use 21 cm; calibrated servo endpoints are
  `SERVOLEFT=94` (tightened from 101 to full lock `SERVOMIN=94`, eliminating ~10 deg position undershoot and understeer on left turns),
  `SERVOCENTER=144` (converged from 146 [+5 deg R], 141 [-3 deg L], and 143 [-3 deg L]), and `SERVORIGHT=206`.
- **Effective Wheel Diameter**: Calibrated to `WHEEL_D_CM = 6.33f` (from 6.09 * 52/50) after
  FC 50cm was measured covering 52cm on ground.
- **FreeRTOS Stack Sizing**: `defaultTask` stack was increased from 512B (`128 * 4`) to 2048B (`512 * 4`) and `MotorTask` to 1024B (`256 * 4`). The 512B stack previously overflowed during `snprintf` + floating-point operations and FPU context saving, causing hard MCU lockups.
- **HC-SR04 Ultrasonic Driver**: Uses hardware timer `TIM6` (1 tick = 1 µs at 16 MHz HSI) instead of software loops. Wrapped with `vTaskSuspendAll()` / `xTaskResumeAll()` during echo pulse measurement to prevent ~1 ms FreeRTOS preemption jitter (which created an artificial 19 cm measurement floor). Includes pre-trigger check to avoid hanging when ECHO is stuck HIGH from blind-zone (< 2 cm) reflections.
- **Flashing Flags & DTR Pin Latch**: In `platformio.ini`, `upload_flags` exit sequence MUST be `-rts,-dtr`. A previous trailing `dtr` (`...:-rts,-dtr,dtr`) left DTR asserted, holding the C30D reset/bootloader circuit and preventing execution of user firmware.
- **Boot Gyro Calibration**: MCU runs 5.0 seconds of gyro offset and drift calibration at power-on before enabling USART3 interrupt. Any commands sent during the first 5s will be ignored.
- **Software E-STOP Reset ('R' Packet)**: A `b"R\x00\x00\x00\x00"` packet in `HAL_UART_RxCpltCallback` clears latched `estopFlag`, resets `runRequested` and `instrLen`, halts motors, and unblocks `comm_task` without requiring a physical MCU reboot.
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
- **E-STOP Reset Forwarding in Serial Bridge**: On receiving `/android/cmd` `RESET` or `ALG|...`, `serial_bridge_node` clears its local `_estop_event` and `_busy` flags and sends `b"R\x00\x00\x00\x00"` to un-latch the STM32 firmware's `estopFlag`, resolving the post-E-STOP `NO_RESPONSE` freeze.
- **Checklist C.10 Android Pose Streaming**: `android_bridge_node` maps `/robot_pose` to `ROBOT,<x>,<y>,<dir>` with heading mapped to cardinal `N`/`S`/`E`/`W` and grid cells $[0..19]$.
- **Task 1 Autonomous Planner & Bull's Eye Orbit Recovery (`planner_node.py`)**: Computes optimal Reeds-Shepp TSP tour from Android `ALG:{...}` layout. If `/perception/sample_target` returns Bull's Eye marker (ID 40) or low confidence, triggers 90° orbit around obstacle footprint to candidate adjacent faces until the true target is confirmed.
- **Task 2 Fastest Car Reactive Sprint (`fastest_car_node.py`, `task2.launch.py`)**: Autonomous sprint FSM for the unmapped 2-obstacle slalom track. Drives forward using ultrasonic feedback, calls `/perception/sample_target` to classify Left Arrow (39) vs Right Arrow (38), executes calibrated S-curve bypass maneuvers (`slalom_left_cmds`/`slalom_right_cmds`), rounds Obstacle 2, and sprints back into the Carpark. Triggered via tablet "SP" button (`STM:sp`) or `pixi run -e pi task2`.
- **Telemetry Kalman Filtering (`kalman_filter.py`, `serial_bridge_node.py`)**:
  - **1D Sensor Kalman Filter (`KalmanFilter1D`)**: Filters ultrasonic (HC-SR04) and Sharp analog IR streams. Features innovation-based outlier gating (`outlier_threshold=0.30m`) that suppresses 1-frame reflection spikes (e.g. sharp sensor drops from 80cm to 25cm while stationary), preventing false collision stops in `planner_node` and early stopping in `fastest_car_node`.
  - **Pose Kalman Filter (`PoseKalmanFilter`)**: 3-DoF Kalman filter for 2D position $(x, y)$ and heading $(\theta)$ with continuous circular angle wrapping in $(-\pi, \pi]$ and large jump rejection. Smooths discrete 100ms odometry jitter while synchronizing instantly to authoritative batch completion feedback (`FIN:POS`, `FIN:...`) and start calibration (`GC` / `RESET`).
- **Operating Mode Switching**:
  - **Checklist Mode**: `pixi run -e pi teleop` (manual driving, sensor tests, C.10 pose streaming).
  - **Task 1 Mode**: `pixi run -e pi robot` (autonomous exploration, 4-8 obstacles).
  - **Task 2 Mode**: `pixi run -e pi task2` (fastest car reactive sprint).
  - **Runtime Switching**: `android_bridge_node` routes `ALG:START` to `planner_node` and `STM:sp`/`SP` to `fastest_car_node` without node restarts.

## Android Remote Control & Real-Time Dashboards (`android/`)

- **Manual Turn Speed**: The Android joystick full-deflection speed is 0.35 m/s.
  Yaw rate remains curvature-derived from the measured 21 cm radius and capped at
  1.75 rad/s. At full lock, the required outer-wheel target is about 0.475 m/s;
  firmware therefore uses a separate 0.480 m/s wheel target cap while retaining
  the 0.35 m/s chassis-command cap.
- **E-STOP Topic Propagation**: Nodes consume an incoming `/estop` without
  republishing it. The origin publishes once; this prevents the planner from
  reporting the same STM32 watchdog or proximity stop twice.
- **Task 1 Real-Time Mission Dashboard**:
  - **Live 2D Arena Map**: Single `ArenaView` instance dynamically reparented into `task1ArenaContainer` on tab switch, showing live dead-reckoned robot pose, obstacle positions, and target icons.
  - **FSM State Badge**: Color-coded mission state pill (`IDLE`, `PLANNING`, `NAVIGATING`, `SAMPLING_TARGET`, `ORBIT_RECOVERY`, `MISSION_COMPLETE`, `ESTOP`).
  - **Mission Stopwatch**: 6-minute competition countdown/elapsed timer (`00:00 / 06:00`), turns red after 5:00.
  - **Leg & Distance Tracker**: Displays current leg progress and remaining distance (`Leg X/Y (Obs Z face) | Rem: W cm`).
  - **Target Consensus Card**: Displays recognized obstacle targets with symbol IDs, labels, and confidence percentages.
  - **Proximity Telemetry**: Displays real-time ultrasonic and IR rangefinder readings.
- **Task 2 Fastest Car Reactive Sprint Dashboard**:
  - **Sprint Stopwatch**: High-precision hundredth-of-a-second stopwatch (`00.00s`), synchronised and locked to the robot ROS clock on completion.
  - **Sprint State Badge**: Tracks active phase (`APPROACH_OBS1`, `DETECT_ARROW1`, `SLALOM_OBS1`, `APPROACH_OBS2`, `DETECT_ARROW2`, `ROUND_OBS2`, `PARK`, `COMPLETE`).
  - **Dual Arrow Detection Cards**: Visual direction indicators (`⬅ LEFT` / `➡ RIGHT`) with symbol IDs and confidence scores for Obstacle 1 and Obstacle 2.
  - **7-Step Pipeline Stepper**: Interactive visual stepper with `✓` green completed steps, `▶` active blue step, and gray pending steps.
  - **Ultrasonic & Maneuver Row**: Real-time ultrasonic range (`US: XX.X cm`) and active maneuver descriptions.
- **Shared Telemetry Protocol & Resilience**:
  - `T1_STATE`, `T1_TARGET`, `T2_STATE`, `T2_ARROW`, and `SENSORS` structured wire formats.
  - Regex fallback parser in `onStatusLine` catches legacy log formats if older nodes run in isolation.
  - Zero memory leaks: all UI handler runnables and Bluetooth threads cleaned up in `onDestroy()`.


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
