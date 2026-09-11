# Continuous velocity migration and validation

The refactor moves movement completion to the Pi and sends replaceable velocity
targets to the STM32. It keeps the existing planner and Android movement service
interface. The purpose is to observe and interrupt a movement while it runs,
without waiting for a distance command's `FIN` response.

Read [the architecture](../ros2_ws/ARCHITECTURE.md) and
[the UART specification](stm32-uart-protocol-spec.md) for the contract.
This guide is a deployment and physical validation procedure; it does not assert
that the refactor has been flashed or tested on the moving robot.

## What changes for operators

- Android joystick/held controls send `VEL:<linear_mps>,<yaw_rps>` over Bluetooth;
  the Android bridge publishes `/cmd_vel/teleop`, and button release sends zero.
- Legacy tablet distance commands, Task 1 paths, and Task 2 primitives call
  `ExecuteMoves` on `motion_controller_node`. It measures completion from live
  telemetry and publishes `/cmd_vel`. The serial bridge consumes that topic and
  sends periodic `V` targets; it no longer owns `ExecuteMoves`.
- A continuous input may publish `/cmd_vel/teleop` (`Twist`), refreshing
  `linear.x` in m/s and `angular.z` in rad/s. Defaults: 20 Hz transmission,
  0.20 s command expiry, 0.40 s telemetry expiry, 0.35 m/s speed limit,
  1.75 rad/s yaw limit, and 0.21 m calibrated minimum turning radius.
- The Pi sends zero after 200 ms without input. The independent MCU fallback
  watchdog stops velocity motion after 750 ms without a new `V` packet.
  Its stop report clears the expired Pi target but does not require RESET; only
  a later fresh velocity command can resume motion.
  Independent firmware proximity protection can stop forward motion without
  waiting for the Pi. Front sensing does not cover reverse travel.
- A stop fault requires explicit tablet RESET. Reset stops and cancels old
  commands; issue a fresh motion request afterwards. Task 1 does not silently
  clear a proximity stop to execute its former automatic reverse tap.
- Use the controller's input topic rather than publishing over its base
  `/cmd_vel` output. The motion controller arbitrates manual and service motion;
  active service motion ignores teleop, and active teleop causes new services to
  return `BUSY_LOCAL`. E-STOP interrupts both. Do not run competing base velocity
  publishers.

## Deploy matching software and firmware

The supplied launches must include both `motion_controller_node` and
`serial_bridge_node` from `mdp_hardware_bridge`. The controller owns
`/execute_moves`; the driver only owns `/hardware/maintenance` for stationary
GC/G0/TO requests. Verify that remappings preserve one base `/cmd_vel` publisher.

| Setting | Default | Owner |
| --- | --- | --- |
| `velocity_speed_mps` | 0.15 m/s | Motion controller primitive speed. |
| Android joystick maximum | 0.35 m/s | Full-deflection manual speed; 1.67 rad/s at the calibrated 21 cm radius. |
| `velocity_turn_radius_m` | 0.21 m | Controller and serial bridge; keep consistent. |
| `velocity_max_yaw_rps` | 1.75 rad/s | Controller and serial bridge; allows 0.35 m/s at 21 cm radius. |
| `velocity_max_speed_mps` | 0.35 m/s | Controller, serial bridge, and firmware limit. |
| `cmd_vel_timeout_sec` | 0.20 s | Controller teleop input and serial bridge base input. |
| `telemetry_timeout_sec` | 0.40 s | Controller and serial bridge freshness checks. |
| `batch_timeout_sec` | 30 s | Controller deadline for each primitive; bridge maintenance timeout. |

Ordinary input expiry and `STOP:WATCHDOG` release manual control. Sensor/telemetry
faults and E-STOP latch motion until `RESET` (or `ALG:RESET`) on `/android/cmd`.
After RESET, velocity remains zero until the first fresh telemetry frame arrives;
a joystick update racing that frame does not create another E-stop.

From `/home/mdp/dev/SC2079-Group-16/ros2_ws`, run offline checks and builds:

```bash
pixi run -e pi test-all
pixi run -e pi build
pio run -d /home/mdp/dev/SC2079-Group-16/stm32
```

Deploy the bridge and firmware from the same revision. An old MCU does not
understand the new velocity payload; an old Pi bridge still expects movement
batch semantics. Before flashing, the operator must stop the running stack and
ensure the programming/UART port has no competing owner. Flash manually:

```bash
pio run -d /home/mdp/dev/SC2079-Group-16/stm32 -t upload
```

Keep `upload_flags` ending in `-rts,-dtr`; do not append `dtr`. Allow the MCU's
boot gyro calibration to complete with the car stationary. Do not let an agent
launch robot operations. The user starts exactly one selected mode manually:

```bash
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
pixi run -e pi teleop
```

After bench validation, the user can stop teleop and manually start either
`pixi run -e pi robot` or `pixi run -e pi task2`. These modes must not overlap.
The serial bridge is the only `/dev/ttyAMA0` owner; Android bridge exclusively
owns `/dev/rfcomm0`. A second launch, serial monitor, or manual UART writer can
break this ownership.

## Bench validation to complete before arena runs

1. **Stationary telemetry:** verify that pose and all ranges update while the
   robot is idle and throughout a long movement request. Record telemetry gaps;
   do not infer freshness solely from a dashboard showing an old value.
2. **Raised-wheel direction check:** at a low requested speed, verify forward,
   reverse, and the steering sign for both forward and reverse turns. Confirm
   zero speed does not request rotation in place. Verify wheel speed changes
   when the target changes during an existing movement.
3. **Stop precedence:** during motion, trigger manual stop; verify motor output
   remains zero despite continued input. Send RESET and confirm the previous
   movement does not resume. Only a fresh command may move the robot.
4. **Command expiry:** stop publishing velocity and observe the ordinary command
   timeout. Separately stop delivery of fresh UART velocity packets to verify
   the MCU watchdog, rather than only the Pi timeout. Record measured cutoff
   times. Perform fault injection only with the car secured and wheels raised.
5. **Telemetry/sensor expiry:** interrupt the applicable measurement stream and
   confirm motion fails and stops; verify an old but plausible sensor value
   cannot sustain a movement. Restore the stream and confirm RESET is required.
6. **Local proximity:** with a secured low-speed setup, bring a target into each
   front sensor's threshold during a long command. Confirm it stops before the
   requested distance ends and reports the cause. Repeat with Pi supervision
   unavailable to establish that firmware protection works independently.
7. **Measured travel and steering:** on a clear floor at low speed, measure
   distance accuracy, clockwise/CCW turn angles, reversing, short movements,
   stopping distance, and overshoot. Check FU/BU behavior against actual front
   range. Confirm failure, rather than completion, when progress stalls.
8. **Mission regression:** only after calibration, validate one obstacle, a full
   Task 1 route and Bull's Eye recovery, then Task 2 at increasing speed. Recheck
   path clearance using the measured turn radius and stopping distance.

## Calibration and limits

Firmware retains the tuned 65 mm wheel diameter, 1527 encoder counts/revolution,
servo center, and steering endpoints from the previous controller. The measured
minimum turning radius is 21–22 cm; the controller and planner use 21 cm. The new
absolute-speed PI loop still needs a short physical response check because the
previous firmware regulated travel distance and left/right balance rather than
accepting speed in metres per second. Tune only if measured speed or tracking
shows that it is necessary.

Local forward stop thresholds start at ultrasonic 12 cm and IR 10 cm. These are
not guarantees of collision avoidance at the configured maximum speed. Validate
sensor coverage, missed echoes, sample delay, braking distance, and steering
sweep. Reverse motion has no rear sensor protection in this implementation.

Keep a record of hardware measurements and resulting parameter/firmware changes
in [MEMORY.md](../MEMORY.md). Automated tests and compilation validate software
behavior, not physical stopping performance. Record physical outcomes separately
with the flashed revision, settings, setup, and measured values.
