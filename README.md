# SC2079 MDP — Group 16

Multi-Disciplinary Design Project (SC2079). Robot car that autonomously
navigates a known arena, recognizes images on obstacles, avoids them using
visual markers, and can be remote-controlled from an Android app. A separate
PC-side algorithm module handles pathfinding and simulation.

## Structure

| Folder | Component | Toolchain |
|---|---|---|
| [`stm32/`](stm32/) | Robot movement firmware (STM32F407VET6) | C, STM32CubeIDE / Keil |
| [`raspberry-pi/`](raspberry-pi/) | Comms hub + image capture/processing (non-ROS track) | Python |
| [`ros2_ws/`](ros2_ws/) | ROS 2 Jazzy implementation of the Pi + PC nodes | Python, pixi |
| [`algorithm/`](algorithm/) | Pathfinding algorithm + arena simulator | Python |
| [`android/`](android/) | Remote controller app | Java/Kotlin, Android Studio |
| [`docs/`](docs/) | Protocol spec, checklist, wiring diagrams | — |

Each component folder has its own README with setup/build instructions.

`ros2_ws/` sits at the top level rather than under `raspberry-pi/` because
its ROS 2 graph is split across two hosts — hardware-bound nodes on the Pi,
perception and planning on a laptop. It is one source tree with two pixi
environments (`pixi run -e pi build` / `pixi run -e pc build`); see
[`ros2_ws/README_PIXI.md`](ros2_ws/README_PIXI.md) for setup and
[`ros2_ws/ARCHITECTURE.md`](ros2_ws/ARCHITECTURE.md) for the design.

Note that `raspberry-pi/` and `ros2_ws/` are two parallel implementations of
the same module — bespoke sockets vs. ROS 2. See `ARCHITECTURE.md` for how
they relate.
See [`docs/protocol.md`](docs/protocol.md) for the message formats used
between Android ↔ RPi ↔ STM32 ↔ Algorithm PC — update it whenever the
wire format changes so all subteams stay in sync.

## Team

- Robot & STM32 firmware: TBD
- RPi & image processing: TBD
- Algorithm: TBD
- Android: TBD
