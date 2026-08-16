# SC2079 MDP — Group 16

Multi-Disciplinary Design Project (SC2079). Robot car that autonomously
navigates a known arena, recognizes images on obstacles, avoids them using
visual markers, and can be remote-controlled from an Android app. A separate
PC-side algorithm module handles pathfinding and simulation.

## Structure

| Folder | Component | Toolchain |
|---|---|---|
| [`stm32/`](stm32/) | Robot movement firmware (STM32F407VET6) | C, STM32CubeIDE / Keil |
| [`raspberry-pi/`](raspberry-pi/) | Comms hub + image capture/processing | Python |
| [`algorithm/`](algorithm/) | Pathfinding algorithm + arena simulator | Python |
| [`android/`](android/) | Remote controller app | Java/Kotlin, Android Studio |
| [`docs/`](docs/) | Protocol spec, checklist, wiring diagrams | — |

Each component folder has its own README with setup/build instructions.
See [`docs/protocol.md`](docs/protocol.md) for the message formats used
between Android ↔ RPi ↔ STM32 ↔ Algorithm PC — update it whenever the
wire format changes so all subteams stay in sync.

## Team

- Robot & STM32 firmware: TBD
- RPi & image processing: TBD
- Algorithm: TBD
- Android: TBD
