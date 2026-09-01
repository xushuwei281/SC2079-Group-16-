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

**Implementation note:** `android_bridge_node`
(`ros2_ws/src/mdp_android_bridge/`) opens `/dev/rfcomm0` via `pyserial` —
see `ros2_ws/bluetooth-setup/` for the one-time pairing + persistent
`rfcomm watch` listener this needs on the Pi side (systemd-managed, so the
Pi is always ready for a connection without a human running commands
first). Distance-unit handling: a linear move (`FC`/`BC`) value is divided
by 10 (treated as mm→cm) when it's `>= 100` or when the node's
`distance_in_mm` parameter is set; smaller values are passed through as
already being centimetres. `TL`/`TR` map to the STM32's forward-turn codes
(`FL`/`FR`); there is currently no way to request a backward turn
(`BL`/`BR`) from the Android app — extend `android_bridge_node`'s
`_MOVEMENT_MAP` if that's ever needed.

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
| STM32 → RPi | `BUS\r\n` | `BUS` | Rejected: STM32 busy executing a move (Pi retries) |
| STM32 → RPi | `FUL\r\n` | `FUL` | Rejected: Command queue full (>40) |


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
