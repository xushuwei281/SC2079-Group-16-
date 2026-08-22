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

## Raspberry Pi ↔ STM32 (UART/serial)

Plain ASCII lines, `\n`-terminated. STM32 ACKs on receipt, then sends
`DONE` once the physical motion finishes (add odometry to `DONE` once
encoders are wired up — `DONE:<x>,<y>,<theta>` — not required for the
first integration pass).

| Direction | Format | Meaning |
|---|---|---|
| RPi → STM32 | `FW:<mm>` / `BW:<mm>` | move forward/backward |
| RPi → STM32 | `TL:<deg>` / `TR:<deg>` | turn left/right |
| RPi → STM32 | `STP` | stop immediately |
| STM32 → RPi | `ACK` | command received |
| STM32 → RPi | `DONE` | motion completed |

## Raspberry Pi ↔ Algorithm (PC) (TCP, JSON lines)

| Direction | Format | Meaning |
|---|---|---|
| RPi → Algo | `{"type":"path_request","obstacles":[...]}` | ask for a path given known obstacles |
| Algo → RPi | `{"type":"path","path":[[x,y,theta],...]}` | ordered list of target configurations |

For today's integration pass, the Algorithm side can hardcode a 2-point
dummy path so RPi↔STM32↔Algorithm can be exercised end-to-end before real
pathfinding exists.

## Image IDs

See `MDP briefing.pdf` / `algarithms_briefing` for the full image ID table
(numbers, alphabets, arrows, stop, bullseye) — 31 classes total (`marker`
+ IDs 11–40), confirmed against the team's Roboflow dataset (see
`raspberry-pi/cv/`).
