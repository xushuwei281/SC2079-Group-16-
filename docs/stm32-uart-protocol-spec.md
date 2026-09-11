# STM32 ↔ Raspberry Pi UART protocol

Version 3 — continuous velocity control, SC2079 Group 16.
This document supersedes the normal movement batch path in
[the archived v2 specification](archive/stm32-uart-protocol-v2.md).
See [the migration guide](cmd-vel-migration.md) before deploying.

## Transport and ownership

USART3, 115200 baud, 8 data bits, no parity, 1 stop bit. The deployed Pi uses the
configured UART device (normally `/dev/ttyAMA0`); use the launch `serial_port`
setting for the actual connection. `serial_bridge_node` exclusively owns it.
An external serial monitor must not open the port while the bridge is running.

Pi-to-STM32 packets are exactly five bytes. Velocity payloads are binary, not
decimal ASCII. Do not add a newline or split a packet with other writes.
STM32-to-Pi telemetry and status are ASCII lines ending in `\r\n`.
The protocol has no CRC or version handshake: matched deployments and correct
five-byte framing matter. It is not a safety-rated transport.

## Velocity packet

| Byte offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | ASCII `V` (`0x56`). |
| 1–2 | signed `int16`, little-endian | Body forward speed in mm/s; negative means reverse. |
| 3–4 | signed `int16`, little-endian | Body yaw rate in mrad/s; positive means CCW. |

Equivalent Python packing is `struct.pack("<chh", b"V", speed_mm_s, yaw_mrad_s)`.
For 0.20 m/s forward and 0.40 rad/s CCW, transmit `56 C8 00 90 01` (hex).
For an ordinary stop, transmit `56 00 00 00 00`.

The ROS input is `/cmd_vel` (`geometry_msgs/msg/Twist`), with `linear.x` in m/s
and `angular.z` in rad/s. The bridge validates finite values and supported axes
and rejects unsupported targets before scaling to wire units. Defaults are 0.30 m/s maximum
speed, 1.20 rad/s maximum yaw rate, and a calibrated minimum 0.21 m turning radius.

Each `V` replaces the previous velocity target; it never joins an instruction
queue and needs no `#` trigger. There is no per-packet `RUN`/`FIN` handshake.
Ackermann steering depends on `yaw_rate / speed`, including the sign of reverse
speed. In-place rotation is unsupported. The bridge and firmware enforce
curvature limits rather than asking the steering servo for impossible motion.

Firmware uses nominal geometry of a 160 mm wheelbase and 150 mm track. The
measured full-lock radius is 21–22 cm; the controller uses 210 mm and retains
calibrated servo endpoints left 101, centre 146, right 206. The absolute-speed
PI controller is new, so verify measured speed, path tracking and braking after
deployment rather than assuming the previous distance-loop tuning transfers.

## Stop and reset packets

| Bytes | Behavior |
| --- | --- |
| `51 00 00 00 00` (`Q` + four zeros) | Immediate latched emergency stop; discard active velocity/movement state. |
| `52 00 00 00 00` (`R` + four zeros) | Explicit reset while stopped; clear latched state and old targets. Fresh commands are required to move again. |
| `56 00 00 00 00` (`V` + four zeros) | Ordinary zero target; does not clear a stop latch. |

The firmware stops nonzero velocity output after 300 ms without a fresh setpoint.
An expired zero target does not trigger `STOP:WATCHDOG`.
Its watchdog stop permits a later fresh `V` at firmware level, but the Pi treats
the asynchronous stop report as a latched fault requiring explicit RESET.
RESET cancels an in-flight `ExecuteMoves` operation; it never resumes the old
primitive from a saved target.

Forward motion is stopped locally when valid raw ultrasonic distance is at or
below 12 cm or either valid raw IR distance is at or below 10 cm. Sensor freshness
is checked locally with a 300 ms limit. Local proximity/stale-sensor faults are
latched. Front sensors do not protect the rear during reverse travel. These
thresholds are firmware settings and must be reconciled with physical stopping
distance and sensor coverage; filtered ROS readings are not the local stop input.

## Telemetry and asynchronous status

| Line | Meaning |
| --- | --- |
| `TLM:<x>,<y>,<heading>,<us>,<ir_left>,<ir_right>` | Live position in cm, legacy heading in degrees, and ranges in cm. |
| `STOP:PROXIMITY` | Local distance threshold interrupted motion. |
| `STOP:WATCHDOG` | Velocity setpoint expired. |
| `STOP:SENSOR_STALE` | Required local sensor updates expired. |
| `STOP:INVALID_VELOCITY` | Binary speed/yaw target exceeded supported limits; latched stop. |

`TLM` continues while moving and while stopped. Its nominal sampling task delay
is 50 ms; sensor acquisition and UART transmission add time. Neither this delay
nor the 300 ms watchdog is a measured bound on total stopping latency.
The Pi reads these lines independently of its movement service handler.

The existing telemetry heading is `90 + clockwise_gyro_degrees`, not a ROS yaw.
The bridge preserves the established transform
`yaw_ros = wrap(radians(180 - heading))` and scales cm to metres. This conversion
is independent of the velocity packet, which uses ROS-positive CCW yaw rate.
`/robot_pose` and `sensor_msgs/Range` use ROS metres; tablet `POSE` and `SENSORS`
messages continue using centimetres.

## Compatibility boundary

The ROS `MoveCommand` and `ExecuteMoves` schemas remain unchanged. Existing
callers can request FC/BC straights, FL/FR/BL/BR turns, and FU/BU range-based
moves. `motion_controller_node` supervises these on the Pi and publishes
`/cmd_vel`; `serial_bridge_node` only consumes the velocity stream and handles
UART telemetry, transport, and safety.
Completion is measured from fresh unfiltered `/robot_pose/raw` and
`/sensors/ultrasonic/raw`, not a received `FIN`. The success status string remains
`FIN` for caller compatibility; it is generated by the Pi controller.
The service remains blocking for callers, but motion, telemetry reception, and
stop callbacks are concurrent.

FC/BC use centimetres; turns use degrees. FU moves forward until ultrasonic
distance is at or below the target; BU reverses until front distance reaches or
exceeds the target. Targets below the minimum front clearance are rejected.
Odometry, sensor, command, or movement timeouts result in stopping/failed motion,
not inferred successful completion.

Legacy ASCII movement packets (`FC050`, `FL090`, etc.) and their `#`, `RUN`,
`FIN`, `BUS`, and `FUL` flow are not the normal ROS movement path. Firmware
retains its legacy parser: after `R`, old movement batches can be accepted until
the first `V` switches to velocity mode. Thereafter legacy motion is rejected
until another `R`. The ROS controller never uses that movement fallback.
The bridge's `/hardware/maintenance` service supports GC/G0/TO only. Operations must run
stationary, with velocity transmission paused, and cannot be mixed into a motion
request. See the source for the supported maintenance whitelist. Do not use a
manual legacy batch to test the new bridge's continuous control behavior.

## Expected sequence

```mermaid
sequenceDiagram
    participant Caller as Planner / Android
    participant Controller as Motion controller
    participant Pi as Serial bridge
    participant MCU as STM32
    Caller->>Controller: ExecuteMoves(FC, 50)
    loop Until measured travel completes
        Controller->>Pi: cmd_vel(speed, yaw_rate)
        Pi->>MCU: V(speed, yaw_rate)
        MCU-->>Pi: TLM(position, heading, ranges)
        Pi-->>Controller: live pose / ranges
    end
    Controller->>Pi: cmd_vel(0, 0)
    Pi->>MCU: V(0, 0)
    Controller-->>Caller: Motion result
    Note over Pi,MCU: At any point a local fault or Q can stop motion
    MCU-->>Pi: STOP:PROXIMITY (if tripped)
    Pi-->>Controller: stop state
    Controller-->>Caller: Active motion fails; RESET required
```

The packet format deliberately stays five bytes to match the receiver framing.
This change requires updating both firmware and bridge together. Offline tests
cover encoding/control decisions; wheel speed, steering response, braking
distance, sensor timing, and loss-of-link behavior still require bench validation.
