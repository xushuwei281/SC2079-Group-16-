# Communication Protocol

Shared reference for the message formats used between components. Update
this doc whenever a format changes — every subteam depends on it.

Draft v0 below — good enough to unblock all four subteams working in
parallel today. Treat every field as negotiable; whoever needs to change
something updates this file and pings the group.

## Android ↔ Raspberry Pi (Bluetooth serial)

Reuses the same short movement commands as RPi↔STM32 below, so the RPi's
job is mostly relay. Status/update messages use a separate `STATUS,`
prefix so the Android GUI's status TextView only ever shows curated text,
not the raw stream (per the checklist's C.4 requirement — don't dump every
byte to the screen).

| Direction | Format | Example | Meaning |
|---|---|---|---|
| Android → RPi | `FW:<mm>` | `FW:50` | move forward 50mm |
| Android → RPi | `BW:<mm>` | `BW:50` | move backward 50mm |
| Android → RPi | `TL:<deg>` / `TR:<deg>` | `TL:90` | turn left/right by degrees |
| Android → RPi | `STP` | `STP` | stop immediately |
| RPi → Android | `STATUS,<text>` | `STATUS,moving` | free-text status update |
| RPi → Android | `DONE` | `DONE` | last movement command completed |
| RPi → Android | `ROBOT,<x>,<y>,<dir>` | `ROBOT,<5>,<12>,<N>` | **Checklist C.10** live pose: `x`/`y` are grid cells `[0..19]`, `dir` ∈ `{N,S,E,W}`. Deliberately coarse (4 headings, 10cm cells) — this is the exact wire format the checklist grades, so don't change its shape without checking `docs/task1-fsm-and-orbit-recovery.md` §4.1. |
| RPi → Android | `POSE,<x_cm>,<y_cm>,<yaw_deg>` | `POSE,52.3,118.7,133` | **Supplemental, non-checklist** full-precision pose, sent alongside `ROBOT` at up to `hires_pose_rate_hz` (`android_bridge_node`, default 10Hz) so the tablet's arena view can track smoothly through turns instead of only updating on 90°/10cm crossings. Purely additive — nothing consumes this for grading. |
| RPi → Android | `T1_STATE,<state>,<current_leg>,<total_legs>,<obs_id>,<face>,<rem_dist_cm>` | `T1_STATE,NAVIGATING,1,5,3,N,45.2` | **Task 1 Autonomous State & Progress:** Emitted on FSM state transitions, leg starts, and waypoint progress. Updates tablet state badge, mission timer, and leg distance counter. |
| RPi → Android | `T1_TARGET,<obs_id>,<symbol_id>,<symbol_name>,<confidence>,<face>` | `T1_TARGET,3,39,Arrow Left,0.92,N` | **Task 1 Target Classification:** Relayed on target recognition consensus and orbit recovery confirmation. Updates tablet targets card and 2D arena target icons. |
| RPi → Android | `T2_STATE,<state>,<step_idx>,<step_desc>,<elapsed_sec>` | `T2_STATE,APPROACH_OBS1,1,Approaching Obstacle 1,2.45` | **Task 2 Sprint State & Stepper:** Emitted on sprint pipeline transitions (steps 1–8). Powers the 7-step progress stepper and millisecond-accurate sprint stopwatch. |
| RPi → Android | `T2_ARROW,<obs_num>,<LEFT\|RIGHT>,<symbol_id>,<confidence>` | `T2_ARROW,1,LEFT,39,0.94` | **Task 2 Arrow Detection:** Real-time steering decision card for Obstacle 1 and 2. Drives the visual arrow indicators (`⬅ LEFT` / `➡ RIGHT`). |
| RPi → Android | `SENSORS,<us_cm>,<ir_left_cm>,<ir_right_cm>` | `SENSORS,28.4,15.2,42.0` | **Throttled Proximity Telemetry (4 Hz):** Front ultrasonic, left IR, and right IR distance in cm for proximity display across all panels. |

**Implementation note:** `android_bridge_node`
(`ros2_ws/src/mdp_android_bridge/`) opens `/dev/rfcomm0` via `pyserial` —
see `ros2_ws/bluetooth-setup/` for the one-time pairing + persistent
`rfcomm watch` listener this needs on the Pi side (systemd-managed, so the
Pi is always ready for a connection without a human running commands
first).

- **1-Deep Move Queueing & Streaming Debounce:** When direction buttons are held
  on Android, commands are streamed every ~50–60 ms. If a move is currently in
  flight, `android_bridge_node` buffers the latest command in `_pending_move` and
  suppresses noisy `BUSY_LOCAL` status alerts over Bluetooth. Upon completion of
  the current move, the pending move is immediately dispatched without returning
  to idle, providing smooth, continuous driving. When the button is released, no
  further commands arrive, and `DONE` is emitted once the last move completes.
- **Distance-unit handling:** a linear move (`FC`/`BC`) value is divided
  by 10 (treated as mm→cm) when it's `>= 100` or when the node's
  `distance_in_mm` parameter is set; smaller values are passed through as
  already being centimetres. `TL`/`TR` map to the STM32's forward-turn codes
  (`FL`/`FR`); backward turns (`BL`/`BR`) are directly supported via two-letter codes.
- **Emergency Stop:** `STP`, `STOP`, or `Q` immediately flushes any pending move
  and broadcasts to `/estop`.

## Raspberry Pi ↔ STM32 (UART/serial, 115200 baud)

Fixed **5-byte packets** for instructions, trigger, and e-stop. Handshake lines (`RUN\r\n`, `FIN:<dist>,<heading>\r\n`, `BUS\r\n`, `FUL\r\n`) for execution status and telemetry.

> **Full Specification:** See [`docs/stm32-uart-protocol-spec.md`](stm32-uart-protocol-spec.md) for sequence diagrams, C code templates, and complete packet definitions.

| Direction | Format | Example | Meaning |
|---|---|---|---|
| RPi → STM32 | `FC<dist>` / `BC<dist>` | `FC050` | Forward/Backward straight distance in cm (000–999) |
| RPi → STM32 | `FL<deg>` / `FR<deg>` | `FL090` | Forward Left/Right turn in degrees (000–360) |
| RPi → STM32 | `BL<deg>` / `BR<deg>` | `BL045` | Backward Left/Right turn in degrees (000–360) |
| RPi → STM32 | `FU<dist>` / `BU<dist>` | `FU020` | Forward/Backward until ultrasound reads `dist` cm |
| RPi → STM32 | `b"#\x00\x00\x00\x00"` | `#\x00...` | **Trigger:** Begin executing queued batch |
| RPi → STM32 | `b"Q\x00\x00\x00\x00"` | `Q\x00...` | **Emergency Stop:** Immediate ISR motor cutoff |
| STM32 → RPi | `RUN\r\n` | `RUN` | Batch execution started |
| STM32 → RPi | `FIN:<dist>,<heading>\r\n` | `FIN:50.2,0.4` | Batch completed with measured encoder distance (cm) & gyro heading (deg) |
| STM32 → RPi | `BUS\r\n` | `BUS` | Rejected: STM32 busy executing a move (evaluated *before* trigger packet `#`) |
| STM32 → RPi | `FUL\r\n` | `FUL` | Rejected: Command queue full (>40) |

**UART Concurrency & Timing Guarantees:**
1. **Busy Rejection Priority:** `HAL_UART_RxCpltCallback` evaluates `runRequested == 1` *before* the trigger `#` packet. Both instructions and trigger are rejected with `BUS\r\n` while executing, preventing empty-queue executions.
2. **Reduced Latencies:**
   - Post-batch settling delay in `comm_task` is 20 ms (reduced from 200 ms).
   - Servo arrival throw delay is 80 ms (reduced from 200 ms).
   - Static friction break delay is 50 ms (reduced from 200 ms).
   - Post-turn delay is 40 ms (reduced from 200 ms).
3. **Safety Timeouts:** All straight and turn loops enforce hardware tick timeouts (`timeout_ticks = delta * 80 + 2000 ms`) to prevent infinite MCU hangs if wheels slip.


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
