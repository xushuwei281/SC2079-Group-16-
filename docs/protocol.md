# Communication Protocol

Shared reference for the message formats used between components. Update
this doc whenever a format changes — every subteam depends on it.

The active ROS motion path uses continuous velocity commands. The legacy
TCP/JSON sections below remain references for the non-ROS implementation.

## Android ↔ Raspberry Pi (Bluetooth serial)

Continuous teleoperation uses `VEL` messages. Legacy short movement commands
remain supported through the Pi motion controller. Status/update messages use a separate `STATUS,`
prefix so the Android GUI's status TextView only ever shows curated text,
not the raw stream (per the checklist's C.4 requirement — don't dump every
byte to the screen).

| Direction | Format | Example | Meaning |
|---|---|---|---|
| Android → RPi | `VEL:<speed_mps>,<yaw_rps>` | `VEL:0.15,0.50` | Latest body speed in m/s and CCW yaw rate in rad/s; refresh while driving and send `VEL:0,0` on release. |
| Android → RPi | `FW:<mm>` | `FW:50` | move forward 50mm |
| Android → RPi | `BW:<mm>` | `BW:50` | move backward 50mm |
| Android → RPi | `TL:<deg>` / `TR:<deg>` | `TL:90` | turn left/right by degrees |
| Android → RPi | `STP` | `STP` | stop immediately |
| Android → RPi | `RESET` | `RESET` | Explicitly clear the stop latch while stopped; old motion does not resume. |
| RPi → Android | `STATUS,<text>` | `STATUS,moving` | free-text status update |
| RPi → Android | `DONE` | `DONE` | last movement command completed |
| RPi → Android | `ROBOT,<x>,<y>,<dir>` | `ROBOT,<5>,<12>,<N>` | **Checklist C.10** live pose: `x`/`y` are grid cells `[0..19]`, `dir` ∈ `{N,S,E,W}`. Deliberately coarse (4 headings, 10cm cells) — this is the exact wire format the checklist grades, so don't change its shape without checking `docs/task1-fsm-and-orbit-recovery.md` §4.1. |
| RPi → Android | `POSE,<x_cm>,<y_cm>,<yaw_deg>` | `POSE,52.3,118.7,133` | **Supplemental, non-checklist** full-precision pose, sent alongside `ROBOT` at up to `hires_pose_rate_hz` (`android_bridge_node`, default 10Hz) so the tablet's arena view can track smoothly through turns instead of only updating on 90°/10cm crossings. Purely additive — nothing consumes this for grading. |
| RPi → Android | `T1_STATE,<state>,<current_leg>,<total_legs>,<obs_id>,<face>,<rem_dist_cm>` | `T1_STATE,NAVIGATING,1,5,3,N,45.2` | **Task 1 Autonomous State & Progress:** Emitted on FSM state transitions, leg starts, and waypoint progress. Updates tablet state badge, mission timer, and leg distance counter. |
| RPi → Android | `T1_TARGET,<obs_id>,<symbol_id>,<symbol_name>,<confidence>,<face>` | `T1_TARGET,3,39,Arrow Left,0.92,N` | **Task 1 Target Classification:** Relayed on target recognition consensus and orbit recovery confirmation. Updates tablet targets card and 2D arena target icons. |
| RPi → Android | `T2_STATE,<state>,<step_idx>,<step_desc>,<elapsed_sec>` | `T2_STATE,APPROACH_OBS1,1,Approaching Obstacle 1,2.45` | **Task 2 Sprint State & Stepper:** Emitted on sprint pipeline transitions (steps 1–8). Powers the 7-step progress stepper and millisecond-accurate sprint stopwatch. |
| RPi → Android | `T2_ARROW,<obs_num>,<LEFT\|RIGHT>,<symbol_id>,<confidence>` | `T2_ARROW,1,LEFT,39,0.94` | **Task 2 Arrow Detection:** Real-time steering decision card for Obstacle 1 and 2. Drives the visual arrow indicators (`⬅ LEFT` / `➡ RIGHT`). |
| RPi → Android | `SENSORS,<us_cm>,<ir_left_cm>,<ir_right_cm>` | `SENSORS,28.4,15.2,42.0` | **Throttled Proximity Telemetry (4 Hz):** Front ultrasonic, left IR, and right IR distance in cm for proximity display across all panels. |
| RPi → Android | `ESTOP_ALERT,<sensor>,<dist_cm>,<threshold_cm>` | `ESTOP_ALERT,Ultrasonic,8.5,12.0` | **Emergency Stop Proximity Cause:** Emitted when collision avoidance halts the vehicle. Identifies the offending sensor (Ultrasonic, IR Left, IR Right, Manual) and measured distance. |

**Implementation note:** `android_bridge_node`
(`ros2_ws/src/mdp_android_bridge/`) opens `/dev/rfcomm0` via `pyserial` —
see `ros2_ws/bluetooth-setup/` for the one-time pairing + persistent
`rfcomm watch` listener this needs on the Pi side (systemd-managed, so the
Pi is always ready for a connection without a human running commands
first).

- **Continuous teleoperation:** held controls refresh `VEL`; the Android bridge
  publishes the latest target on `/cmd_vel/teleop`. `motion_controller_node`
  arbitrates inputs and publishes the base `/cmd_vel`; button release sends zero velocity.
  Missing refreshes expire, so losing the stream cannot leave the last nonzero
  velocity active indefinitely. `DONE` remains a discrete-movement result and is
  not an acknowledgement for every velocity update.
- **Legacy movement compatibility:** FC/BC and turn requests use `ExecuteMoves`
  on `motion_controller_node`, which observes live odometry and publishes
  `/cmd_vel`. These requests no longer become queued UART movement batches.
- **Distance-unit handling:** a linear move (`FC`/`BC`) value is divided
  by 10 (treated as mm→cm) when it's `>= 100` or when the node's
  `distance_in_mm` parameter is set; smaller values are passed through as
  already being centimetres. `TL`/`TR` map to the STM32's forward-turn codes
  (`FL`/`FR`); backward turns (`BL`/`BR`) are directly supported via two-letter codes.
- **Emergency Stop:** `STP`, `STOP`, or `Q` immediately flushes any pending move
  and broadcasts to `/estop`.

## Raspberry Pi ↔ STM32 (UART/serial, 115200 baud)

Normal motion uses fixed **5-byte binary velocity packets**, with continuous
ASCII telemetry and asynchronous stop reports. Velocity replaces the previous
target immediately; it is not queued behind a distance/turn command.

See [the UART specification](stm32-uart-protocol-spec.md) for byte layout,
limits, reset behavior, and legacy maintenance restrictions, and
[the migration guide](cmd-vel-migration.md) for deployment and physical checks.

| Direction | Format | Example | Meaning |
|---|---|---|---|
| RPi → STM32 | `V` + signed int16 LE mm/s + signed int16 LE mrad/s | Hex `56 C8 00 90 01` | 0.20 m/s forward, 0.40 rad/s CCW; latest target replaces the previous one. |
| RPi → STM32 | `b"V\x00\x00\x00\x00"` | Hex `56 00 00 00 00` | Ordinary zero velocity; does not clear a stop latch. |
| RPi → STM32 | `b"Q\x00\x00\x00\x00"` | `Q\x00...` | **Emergency Stop:** Immediate ISR motor cutoff |
| RPi → STM32 | `b"R\x00\x00\x00\x00"` | `R\x00...` | Explicit reset; clears old targets and requires a fresh motion command. |
| STM32 → RPi | `TLM:<x>,<y>,<heading>,<us>,<ir_left>,<ir_right>\r\n` | `TLM:20,20,90,50,40,40` | Live pose/ranges in cm and legacy heading in degrees, throughout motion. |
| STM32 → RPi | `STOP:<reason>\r\n` | `STOP:PROXIMITY` | Asynchronous stop: `PROXIMITY`, `WATCHDOG`, `SENSOR_STALE`, or `INVALID_VELOCITY`. |

The central Pi motion controller checks every forward velocity against fresh raw
ultrasonic and IR samples before forwarding it. It latches at 12 cm ultrasonic or
10 cm IR, and it covers Android teleoperation and both autonomous modes. The STM32
also applies those thresholds as a last-resort cutoff and zeros velocity after
300 ms without a fresh setpoint. A zero range means no return/out of range; sensor
freshness follows the acquisition-task heartbeat instead of requiring a positive
echo. Front sensors do not protect reverse travel. Telemetry reception and stop
callbacks do not wait for `FIN`. The old `#`/`RUN`/`FIN` sequence is restricted to
legacy maintenance; normal motion completion belongs to the Pi motion controller.


## Raspberry Pi ↔ Algorithm (PC) (TCP, JSON lines)

| Direction | Format | Meaning |
|---|---|---|
| RPi → Algo | `{"type":"path_request","obstacles":[...]}` | ask for a path given known obstacles |
| Algo → RPi | `{"type":"path","path":[[x,y,theta],...]}` | ordered list of target configurations |

For today's integration pass, the Algorithm side can hardcode a 2-point
dummy path so RPi↔STM32↔Algorithm can be exercised end-to-end before real
pathfinding exists.

## Image recognition results (Raspberry Pi → Algorithm PC + Android)

No format existed for this until the CV model was actually trained — added
now that it's needed.

The **raw frame goes to the Algorithm PC**, not Android: the PC already has
a TCP link and far more headroom than the Bluetooth serial line, and it's
the PC that builds the stitched verification image required by the
checklist ("stitched RAW images... shown as a single image in android or
PC"). Android gets a lightweight text status only — no image payload over
Bluetooth.

| Direction | Format | Meaning |
|---|---|---|
| RPi → Algo | `{"type":"image_result","obstacle_id":"<id>","label":"<class or UNCERTAIN>","confidence":<float>,"image_b64":"<jpeg>"}` | one detection, raw frame attached |
| RPi → Android | `STATUS,recognized <label> at <obstacle_id>` | text-only status, no image |

`label` uses the numeric class IDs from the table below (`11`–`40`), or the
string `marker` for a bull's-eye, or `UNCERTAIN` if confidence is below
threshold — see `raspberry-pi/cv/pi_infer.py`, which already produces this
exact shape (`Detection.label`/`.confidence`/`.raw_frame`), it just needs
`_send_result()` wired to this format instead of its current `TODO`.

The Algorithm PC accumulates `image_result` messages per `obstacle_id` and
builds the stitched grid the same way `pi_infer.py`'s
`build_verification_stitch()` sketches it — whichever side ends up owning
the display, don't duplicate the stitching logic in both places.

## Image IDs

See `MDP briefing.pdf` / `algarithms_briefing` for the full image ID table
(numbers, alphabets, arrows, stop, bullseye) — 31 classes total (`marker`
+ IDs 11–40), confirmed against the team's Roboflow dataset (see
`raspberry-pi/cv/`).
