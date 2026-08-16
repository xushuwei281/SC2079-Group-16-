# Communication Protocol

Shared reference for the message formats used between components. Update
this doc whenever a format changes — every subteam depends on it.

## Android ↔ Raspberry Pi (Bluetooth serial)

TODO: define string command format (e.g. movement commands, status updates).

## Raspberry Pi ↔ STM32 (UART/serial)

TODO: define command set (e.g. move forward/back, turn, distances).

## Raspberry Pi ↔ Algorithm (PC) (TCP/HTTP)

TODO: define request/response format for path requests, obstacle list,
image recognition results.

## Image IDs

See `MDP briefing.pdf` / `algarithms_briefing` for the full image ID table
(numbers, alphabets, arrows, stop, bullseye).
