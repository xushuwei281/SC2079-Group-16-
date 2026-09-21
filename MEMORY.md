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
  - Turn Overshoot Compensation & Cushioned Braking (2026-09-14): Fast turns slightly overshot (~3°–4°) because flooring at 0.18 m/s (~49 deg/s) right up to the completion threshold caused kinetic momentum and ~80 ms braking/serial latency to carry the robot past the target angle. Fixed by: (1) tapering the final turn approach down to 0.10 m/s (~27 deg/s) within the last 8°, reducing kinetic energy at cutoff by 70%; (2) adding dynamic parameter `turn_overshoot_deg = 2.2` (default 2.2°) to lead the stop threshold (`remaining <= stop_threshold`), cleanly absorbing the 80 ms braking transition so yaw settles within ±0.3° of target; (3) trimming `turn_180_runout_m` from 0.05 m to 0.02 m since the energized turn with tightened lock (94) tracks the true 21 cm radius without dragging. Supported dynamic tuning via `ros2 param set /motion_controller_node turn_overshoot_deg <deg>`.
  - Proximity Noise Rejection & Debouncing (2026-09-14): Eliminated false E-STOP triggers in open space. Filtered HC-SR04 transducer ringing glitches (`echo_us < 150` rejected in `HCSR04_ReadCm`). Fixed Sharp IR clamping bug in `IR_RawToCm` and `IR1_RawToCm` (calibrated smoothly down to 6 cm rather than flatlining at the 10 cm trip boundary). Added 8-tick (~80 ms) proximity confirmation debounce in STM32 `motor()` and 2-sample debounce in `planner_node.py` and `fastest_car_node.py`, ensuring single-frame acoustic or motor electrical switching spikes do not cause false hardware E-STOPs.
- See [architecture](ros2_ws/ARCHITECTURE.md), [UART protocol](docs/stm32-uart-protocol-spec.md),
  and [migration/bench guide](docs/cmd-vel-migration.md). Older discrete-batch
  notes below are historical where they conflict with this section.

## Closed-Loop Straight Heading Hold & Steering Trim (2026-09-21)

- **Problem:** When running straight moves (e.g. `FC 200`), the robot accumulated a +10° rightward yaw drift over 200 cm.
- **Root Causes:**
  1. Open-loop straight driving in `motion_controller_node.py`: straight commands (`FC`, `BC`, `FU`, `BU`) and 180° turn runouts previously set `yaw_rate = 0.0`, leaving vehicle heading vulnerable to wheel slip, floor irregularities, and mechanical trim imbalance.
  2. Steering servo center trim: `VELOCITY_SERVO_CENTER` in `stm32/include/velocity_control.h` was set to `146` (slight right bias; previously `144` had a ~3° left tilt; `145` is the true balanced mechanical center).
- **Fixes Applied:**
  1. **Closed-Loop Heading Hold in `motion_controller_node.py`:**
     - Added dynamic parameters:
       - `straight_heading_kp = 1.2` (range: `0.0`–`5.0`)
       - `straight_heading_deadband_deg = 0.3` (deadband in degrees to avoid servo jitter on IMU noise; range: `0.0`–`5.0`)
       - `straight_max_yaw_rps = 0.35` (maximum angular velocity cap for straight corrections; range: `0.05`–`1.0`)
     - During straight moves and 180° turn runouts, the controller computes heading error relative to the initial move pose:
       $e_\theta = \text{atan2}(\sin(\theta_{\text{target}} - \theta), \cos(\theta_{\text{target}} - \theta))$.
     - If $|e_\theta| \ge \text{deadband}$, it computes corrective yaw rate:
       $\omega = \text{clip}(K_p \cdot e_\theta, -\omega_{\max}, \omega_{\max})$
       where $\omega_{\max} = \min(\text{straight\_max\_yaw}, |v| / R_{\min})$ to strictly respect the Ackermann curvature limit.
     - Supports live tuning via `ros2 param set /motion_controller_node straight_heading_kp <val>`.
  2. **Firmware Trim Adjustment in `stm32/include/velocity_control.h`:**
     - Adjusted `VELOCITY_SERVO_CENTER` from `146` to `145` (dead center between 144 and 146).
- **Testing & Verification:**
  - Added unit test `test_straight_drive_corrects_heading_drift` in `ros2_ws/src/mdp_hardware_bridge/test/test_motion_controller.py` verifying both clockwise and counter-clockwise drift elicit opposing corrective yaw commands.
  - All test suites (`test-algo`, `test-bridge`, `test-planner`) pass 100% (135/135 tests).

## Planner turning radius 28 cm -> 25 cm (2026-09-21)

- **Why:** Team re-measured the physical full-lock radius on the arena surface as **25 cm** (down from the 2026-09-18 28 cm calibration).
- **Changed (planner prediction only, 3 values):**
  - `algorithm/arena.py:24` `TURNING_RADIUS_CM` 28.0 -> 25.0
  - `ros2_ws/src/mdp_bringup/mdp_bringup/planner_node.py:90` param `turning_radius_cm` 28.0 -> 25.0
  - `ros2_ws/src/mdp_bringup/launch/robot.launch.py:91` launch arg `turning_radius_cm` "28.0" -> "25.0"
- **Deliberately NOT changed:** `motion_controller_node`/`serial_bridge_node` `velocity_turn_radius_m` (0.21) and firmware `VELOCITY_MIN_RADIUS_MM` (210), where 21 cm represents the command scale for full servo lock.
- **A* Obstacle Safety Margin 1.0 cm -> 4.0 cm (2026-09-21):** Increased `hybrid_astar` action expansion collision check margin against obstacles from 1.0 cm to 4.0 cm in `algorithm/planning.py`, matching the main path planning margin and preventing tight wall-clipping during obstacle fallback maneuvers. Supported independent `boundary_margin=1.0` in `algorithm/arena.py:robot_collides_any` so arena wall checks do not artificially reject valid turns starting near start/boundary zones.
- **Bull's Eye Hardcoded Orbit Right Macro (2026-09-21):** Replaced multi-candidate A* search with a fixed, calibrated relative macro `[("BC", 30), ("FR", 90), ("FC", 18), ("FL", 180)]` in `ros2_ws/src/mdp_bringup/mdp_bringup/planner_node.py`. When Bull's Eye (ID 40) is detected, the planner immediately transitions clockwise to the adjacent right face (`N` -> `W`, `W` -> `S`, `S` -> `E`, `E` -> `N`), skipping the 30s visual-servo timeout (`enable_closed_loop_orbit=False`). Checked candidate vantage pose against arena boundaries ($15 \le x, y \le 185$) and obstacle collisions before executing.
- **Checklist A.5 Autonomous Approach & Orbit Script (2026-09-21):** Implemented dedicated standalone node and task `checklist-a5` (`ros2_ws/src/mdp_bringup/mdp_bringup/checklist_a5.py`, entrypoint `checklist_a5`, task alias `pixi run -e pi checklist-a5`). Node subscribes to `/sensors/ultrasonic`, approaches obstacle safely down to target distance (default 25.0 cm) via incremental forward steps (`FC`) in Phase 1 only, queries perception (`/perception/sample_target`), and executes the calibrated orbit macro `[("BC", 30), ("FR", 90), ("FC", 18), ("FL", 180)]` if no target or Bull's Eye (ID 40) is detected, repeating for up to 4 obstacle faces. Does NOT step forward after orbiting; if no symbol is confirmed, it directly orbits to the next side. Includes comprehensive unit tests in `test_checklist_a5.py` (9 tests).

## Planner turning radius 21 cm -> 28 cm (2026-09-18)

- **Why:** Task 1 vantage poses were ~10 cm off (S-face obstacle: car ended too
  far right in x after right turns). Team re-measured the physical full-lock
  radius on the current (reverted, `SERVOLEFT=101`) firmware as **28 cm**, while
  the planner was drawing 21 cm Reeds-Shepp arcs. Turns terminate on gyro yaw,
  so every 90° turn landed ~7 cm off in x and y versus the plan.
- **Changed (planner prediction only, 3 values):**
  - `algorithm/arena.py:24` `TURNING_RADIUS_CM` 21.0 -> 28.0
  - `ros2_ws/src/mdp_bringup/mdp_bringup/planner_node.py:90` param `turning_radius_cm` 21.0 -> 28.0
  - `ros2_ws/src/mdp_bringup/launch/robot.launch.py:91` launch arg `turning_radius_cm` "21.0" -> "28.0"
- **Deliberately NOT changed:** `motion_controller_node`/`serial_bridge_node`
  `velocity_turn_radius_m` (0.21), firmware `VELOCITY_MIN_RADIUS_MM` (210) and
  servo endpoints, Android `JOYSTICK_TURN_RADIUS_M` / `teleop_min_turn_radius_m`
  (0.21), sim launch files (`sim_robot.launch.py:80` still 21.0). In those
  layers "21 cm" just means "command full lock", which physically is 28 cm, so
  planner and robot now agree. Only side effect: the firmware's rear-wheel
  differential speeds are still sized for a 21 cm arc (a little tyre scrub).
  Never change the controller to 0.28 alone — firmware would then give only
  ~80% lock (radius > 28 cm). An all-layers 28 cm change needs controller +
  serial bridge + firmware 280 (reflash) + teleop radius together.
- **Verified:** `algorithm/test` 13/13 pass (run with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ros2_ws/.pixi/envs/pi/bin/python -m pytest test`
  from `algorithm/`; the ROS `launch_testing` pytest plugin breaks plain
  collection). Default arena: 448.9 cm / 33 cmds at 28 vs 419.4 cm / 34 cmds
  at 21, same per-leg methods (legs 1, 2, 5 use A* fallback at both radii).
  Not yet verified on hardware.
- **Revert:** from repo root, `git apply -R docs/revert-patches/turning-radius-28cm.patch`
  (patch contains only these 3 lines; other uncommitted edits in the same files
  are untouched), then `pixi run -e pi build`. Or at runtime without editing
  code: `pixi run -e pi robot turning_radius_cm:=21.0` (pixi appends the arg
  to `ros2 launch mdp_bringup robot.launch.py`) — but note `algorithm/arena.py`'s constant is also the default for any code path
  that doesn't receive the radius explicitly.
- Unrelated open issue found during the same investigation (not fixed):
  planner treats pose as body centre while STM32 odometry tracks the rear-axle
  midpoint (~7 cm behind centre per the sim URDF, unmeasured on hardware) —
  expected to leave ~7 cm (E/W faces) to ~14 cm (N face) error.

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

- **Reverted to pre-calibration firmware (2026-09-18)**: `stm32/include/velocity_control.h`
  and `stm32/src/main.c` were rolled back to their 2026-09-11 14:14 state
  (commit `a3d6c89`, PR/commit `09db539`) and reflashed to the C30D board over
  Port 1 (`platformio run --target upload`, `[SUCCESS]`) — confirmed working
  on hardware. **This supersedes the "Measured Turning Radius & Steering" and
  "Effective Wheel Diameter" bullets below**: the board currently runs
  `SERVOLEFT=101`, `SERVOCENTER=146`, `WHEEL_D_CM=6.5f`, PWM feedforward/KP at
  2.0f/1.5f, and without the 2026-09-14 IR/ultrasonic clamp fix, echo-ringing
  rejection, or the 8-tick proximity-confirmation debounce. Those bullets stay
  below as a record of what was previously derived (and may be reapplied) —
  check `git log -- stm32/` before assuming any of them are live.
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

- **Zenoh Client Config Path Resolution (2026-09-18, commit `5c6875b`)**:
  `robot.launch.py`, `hardware.launch.py`, `teleop.launch.py`, `task2.launch.py`,
  and `mdp_camera_bringup/camera.launch.py` resolved `ZENOH_SESSION_CONFIG_URI`
  relative to the *installed* launch file location
  (`install/<pkg>/share/<pkg>/launch/`), two directory levels short of the
  actual `config/` dir, with a hardcoded fallback pointing at a different
  machine's checkout path (`/home/mdp/dev/SC2079-Group-16/...`). Both misses
  left `zenoh_cfg` pointing nowhere, killing `motion_controller_node`, the
  static TF publishers, and `foxglove_bridge` on launch with "Invalid
  configuration file". Fixed by trying `PIXI_PROJECT_ROOT` (set by pixi for
  every task, correct regardless of checkout path) first; old candidates kept
  as last-resort fallback for a bare `ros2 launch` outside pixi.
- **pixi.toml dependency audit (2026-09-18, commit `a6aca87`)**: removed three
  deps confirmed unused anywhere in `src/`: `ros-jazzy-v4l2-camera`
  (`camera.launch.py` actually launches a custom Picamera2-shared-memory
  `pi_camera_node`, not the `v4l2_camera` ROS package — cleaned up matching
  stale `v4l2-compat.so`/LD_PRELOAD references left in `mdp_camera_bringup`'s
  `package.xml`/`setup.py`/launch docstring from before that rewrite),
  `ros-jazzy-tf2-geometry-msgs` (never imported; only `tf2_ros` is used
  anywhere), and `ros-jazzy-vision-msgs` (declared in `mdp_perception`'s
  `package.xml` but never imported). `pygame` (`[feature.common]`, only used
  by `algorithm/simulator.py`'s desktop GUI) was deliberately left installable
  on the Pi too, per team preference, despite the robot never running it.
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

## Gazebo Simulation (`ros2_ws/src/mdp_gazebo/`, pixi `-e sim`)

- New pixi environment `sim = [common, pc, sim]` (`ros2_ws/pixi.toml`) layers
  `ros-jazzy-ros-gz-sim`/`ros-jazzy-ros-gz-bridge` (Gazebo Harmonic) on top of
  `pc`, so plain `pc` installs (perception dev) don't pay for Gazebo. Tasks:
  `pixi run -e sim gazebo` (world + robot + bridges) and
  `pixi run -e sim gazebo-autonomous` (adds `motion_controller_node` +
  `planner_node`, unmodified, driving the simulated robot).
- **Design: `sim_bridge_node` reproduces `serial_bridge_node`'s ROS contract**,
  not the hardware. It subscribes the DiffDrive plugin's bridged odometry
  (`/sim/odom`) and republishes `/robot_pose(/raw)` (PoseStamped, `odom`
  frame) plus the `odom->base_link` TF and the ultrasonic/IR `Range` topics —
  same topics, frames, and message shapes `serial_bridge_node` provides on
  the Pi. This is why `motion_controller_node`/`planner_node` need zero
  changes to run against sim (see `gazebo-autonomous` above).
- The Range topics are a **stub** (always max-range/"clear", fresh
  timestamp) — no obstacle-sensing beams are simulated. It exists solely so
  `motion_controller_node`'s `SENSOR_STALE`/`PROXIMITY` interlock doesn't
  block forward motion in sim. Real collision-triggered stopping isn't
  testable in sim yet.
- Arena world (`worlds/mdp_arena.sdf`) is a flat 200x200cm plane with its
  **bottom-left corner at the Gazebo world origin**, matching
  `algorithm/arena.py`'s `ARENA_SIZE_CM=200` coordinate convention exactly —
  a spawned pose in metres times 100 equals the arena/planner pose in cm, no
  transform needed (confirmed from `planner_node.py:_on_pose`, which already
  does `x_cm = msg.pose.position.x * 100.0`). Robot spawns at (0.2, 0.2),
  the real 40x40cm start zone's centre.
- Robot description (`description/mdp_robot.urdf.xacro`) visuals come from
  Wheeltec's `mini_diff_robot_meshes` STLs (user-supplied, from a local
  `wheeltec_robot_urdf` copy — no license file ships with them, check terms
  before distributing this repo further). **Only the wheel radius/width
  (33.2mm / 28.6mm) came from measuring those meshes' own bounding boxes**
  (no wheel-diameter constant exists anywhere else in the repo); wheel
  separation (0.15m) is `docs/stm32-uart-protocol-spec.md`'s 150mm track,
  independently confirmed by the base mesh's own 153.6mm footprint width.
  The chassis **collision** box uses the real footprint instead
  (`algorithm/arena.py` `ROBOT_W_CM`/`ROBOT_H_CM` = 19/23cm) since that's
  what should govern physics/clearance, not the stand-in visual mesh.
  Chassis height and caster placement are unmeasured visual estimates.
- **Two unrelated macOS-toolchain bugs surfaced while getting the sim to
  actually run**, both pre-existing (confirmed identical on plain `-e pc`,
  nothing to do with this package):
  - **Colcon build failure** (`libSystem.tbd: ... unknown architecture
    arm64e.x1-macos`): this Mac's Xcode CLT had updated to a `27.0` preview
    SDK whose `libSystem.tbd` lists an `arm64e.x1-macos` target the linker
    can't parse. Fix lives in `~/.zshrc` (personal, not the repo — it's
    this-machine-specific, not project-specific):
    `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk`.
  - **Every node importing an `mdp_interfaces` type crashed at runtime**
    (`Library not loaded: @rpath/libmdp_interfaces__rosidl_generator_py.dylib`,
    then separately `rmw_cyclonedds_cpp`/`rmw_zenoh_cpp` failing to
    `dlopen()` `libmdp_interfaces__rosidl_typesupport_introspection_c.dylib`
    by bare name when creating any service/client of an mdp_interfaces
    type — e.g. `motion_controller_node`'s `/execute_moves` or
    `planner_node`'s client of it). Two different bugs needing two
    different fixes:
    1. mdp_interfaces' 3 Python typesupport `.so`s are missing an rpath
       back to their own package's `lib/`. `pixi run -e pc build` now runs
       `scripts/fix_macos_mdp_interfaces_rpath.sh` afterward to patch it
       (must use the conda env's own `install_name_tool`, not the system
       one — only conda's re-signs after modifying rpaths; skipping that
       gets the binary SIGKILLed on load). No-op on Linux; wiped by every
       rebuild, hence reapplied automatically rather than done once.
    2. `rmw_cyclonedds_cpp`'s dlopen()-by-name needs
       `DYLD_LIBRARY_PATH=install/mdp_interfaces/lib`, which rpath can't
       provide (different loading path entirely). Confirmed pixi's own
       `env = {...}` task field **silently drops any `DYLD_`-prefixed
       variable** — a deliberate filter, not a bug in this repo — so it has
       to be a literal shell-level prefix inside the task's `cmd` string
       instead (see the `gazebo`/`gazebo-autonomous` task definitions).
       Same applies to any *other* task added later that runs a node using
       mdp_interfaces types under cyclonedds/zenoh on macOS.
- **GUI rendering (`gz sim -g`) initially appeared broken on this Mac**, with
  the OGRE2 log (`~/.gz/rendering/ogre2.log`) showing the
  `gz-rendering8-ogre2` package's baked-in plugin directory string
  corrupted, so every `dlopen()` of a render-system plugin failed on a
  bogus path. **This was not an ogre2 packaging bug** — it was the same
  stale-prefix issue as the rpath/dlopen fixes above: `.pixi/envs/{pc,sim}`
  had been installed while this repo lived at a different absolute path
  (it was moved/renamed since), so every conda package with a baked-in
  absolute path — including ogre2's plugin dir string, not just
  `colcon`/Python shebangs — pointed at a path that no longer existed.
  `rm -rf .pixi/envs/{pc,sim} && pixi install -e sim` (relinking from the
  local package cache, no re-download) fixed it: the GUI opens and renders
  correctly. If it breaks again, suspect a stale prefix before assuming a
  real ogre2 bug. `headless:=true` (physics/topics/services all still work)
  plus Foxglove or RViz remains a valid fallback if needed.

## Bluetooth RFCOMM Provisioning (Pi)

- The one-time-per-Pi OS setup in `ros2_ws/bluetooth-setup/` (see its
  `README.md`) had never been run on this machine as of 2026-09-18:
  `bluetoothd` wasn't in `--compat` mode (needed for `sdptool`), the adapter
  was soft rfkill-blocked and powered off, no SPP SDP record existed, no
  tablet was paired, and neither `mdp-bluetooth-setup.service` nor
  `mdp-rfcomm-listen.service` was installed under systemd.
- Provisioned per the README: installed `bluetooth-compat-override.conf` to
  `/etc/systemd/system/bluetooth.service.d/override.conf`, copied
  `bring-up-and-register.sh`/`rfcomm-listen.sh` to `/opt/mdp/bluetooth-setup/`
  (the path the systemd unit files hardcode), installed + `enable --now`'d
  both units, restarted `bluetoothd`, and ran `bring-up-and-register.sh`
  (clears the soft rfkill block, powers the adapter, registers the SPP SDP
  record on RFCOMM channel 1). `rfcomm watch 0 1` is now running persistently
  under `mdp-rfcomm-listen.service`, surviving reboots.
- Paired the tablet: `Galaxy Tab A7 Lite` (`F4:F3:09:BA:6E:94`) — confirmed
  `Paired: yes`, `Trusted: yes` via `bluetoothctl info`.
- **`pair-agent.sh` has a live bug**: each `bluetoothctl <cmd>` line in the
  script is its own one-shot subprocess, so the agent registered by
  `agent NoInputNoOutput` is gone (its D-Bus connection closes) by the time
  the next line's `default-agent` runs in a fresh process — fails with
  "No agent is registered". Worked around by driving one persistent
  `bluetoothctl` session over a named pipe (`mkfifo`, `exec 3<>fifo`,
  `bluetoothctl <&3 &`, then `printf 'cmd\n' > fifo` from later shell
  invocations — note a plain `>` open on the fifo works fine as a follow-up
  writer as long as `bluetoothctl` still holds the read end open; you don't
  need to reopen fd 3 in every subsequent shell call). The script itself is
  still unfixed.

## Perception (`mdp_perception`) — NCNN Backend (2026-09-18)

- `TargetDetector._load_model` tries, in order: an Ultralytics NCNN export
  directory (`<base_name>_ncnn_model/`, fastest on Pi CPU, no torch), a
  `.onnx` file via onnxruntime, then a `.pt` fallback via `ultralytics` —
  that last path is only reachable off-Pi, since the `pi` pixi environment
  deliberately keeps `ultralytics`/`torch` off the robot (see `pixi.toml`).
  Loading a bare `.pt` on the Pi with no matching NCNN/ONNX export fails
  *silently* into mock mode (always "no detection", ~9ms instead of real
  inference) rather than raising a visible error — confirmed by pointing
  `perception_node` at the raw COCO-pretrained `yolov8n.pt` (downloaded from
  the `ultralytics/assets` GitHub release `v8.3.0`, since deleted — verified
  it was stock COCO by grepping its `data.pkl` for the literal class-name
  ordering `names`/`person`/`bicycle`/`car`, not the project's `11`-`40`
  symbol set): `ModuleNotFoundError: No module named 'ultralytics'`, silently
  swallowed into the mock-mode warning print.
  Do not point `model_path` at a `.pt` with no NCNN/ONNX sibling on the Pi.
- **NCNN thread count 2→4** (`detector.py` `_load_ncnn`, commit `6e9ed41`):
  was capped at 2 threads to leave headroom for `motion_controller_node`/
  `serial_bridge_node`'s real-time loop. Measured `/perception/sample_target`
  latency on hardware: ~3.3-4.0s/call at 2 threads (deployed `best_ncnn_model`,
  214 layers/39.4MB `.bin`, imgsz 480) vs ~1.55-1.65s at 4 threads. Judged
  worth the CPU contention since a sample call is a short on-demand burst
  (robot stopped at an obstacle), not continuous inference.
- **`models/best-yolov8n-baseline_ncnn_model/`** (committed by the CV
  teammate's agent, `125ff3b`): a from-scratch yolov8n architecture trained
  on the same `data.yaml` classes (`11`-`40`, `marker`), exported at imgsz
  640, 12.1MB `.bin` — a genuine baseline-comparison model, distinct from the
  stock COCO `yolov8n.pt` above. Swapping `perception_node` onto it
  (`--ros-args -p model_path:=models/best-yolov8n-baseline.pt`) measured
  ~1.06-1.26s/call at 4 threads with real, confident detections (e.g. marker
  conf 0.96) — faster than the deployed model despite the larger 640 imgsz,
  because it's a genuinely smaller architecture. Further speedup needs a
  re-export at smaller imgsz or int8 quantization, both requiring
  `ultralytics` on a PC — not doable from the Pi. As of this writing
  `perception_node` is running standalone on this baseline model (started
  via the CLI override above, outside the `robot.launch.py` tree) — it is
  **not** wired into `robot.launch.py` as the default; that launch file's
  `perception` Node still has no `parameters=` block, so a full
  `pixi run -e pi robot` relaunch would revert to `best_ncnn_model`.
- **rosidl stale-build gotcha**: after the CV teammate's agent added
  `TrackedTarget.msg`/`TrackingControl.msg`/`BullseyeOrbit.srv` to
  `mdp_interfaces`, an incremental `colcon build` left the C typesupport
  `.so` out of sync (`undefined symbol:
  mdp_interfaces__srv__execute_moves__event__convert_to_py`) even after the
  Python-level `ImportError` for the new message types had already cleared
  via a prior rebuild. A full `pixi run -e pi clean && pixi run -e pi build`
  was required — incremental colcon builds are not reliable after adding new
  interfaces to an already-built `mdp_interfaces`.

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
