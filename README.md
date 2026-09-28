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

## CV training, deployment and work evidence

Start with the [complete CV documentation index](docs/cv/README.md) for the documented
Hugging Face nano/medium baselines, dataset audit, training/export tools, verified ONNX
files, Raspberry Pi testing and evidence of completed work. Each tool family includes
usage, dependencies, inputs, outputs and validation limits.

The exact models are available through [Git LFS](models/cv-baselines/README.md).
Historical live-test runtimes and the bullseye experiment are kept separate from the
current robot stack. The recorded physical nano tests and medium saved-image replay
are distinguished from unverified full-field or moving-robot performance.

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

## 3D Gazebo Simulation (`mdp_simulation`)

A high-fidelity Gazebo Harmonic 3D simulation package is provided under [`ros2_ws/src/mdp_simulation`](ros2_ws/src/mdp_simulation/README.md) matching the NTU SC2079 course hardware and arena specifications:
- **Robot Model**: Real Wheeltec Mini-Ackermann CAD model (aluminum baseplate, upper acrylic deck, STM32 board, Ackermann steering linkages, Pi Camera v2.1, front ultrasonic sensor, and dual Sharp IR sensors).
- **Multiple Arenas**:
  - `arena.sdf` (Default): Task 1 competition arena with 5 canonical obstacles, YOLO target textures, and Bull's Eye visual markers.
  - `task2_arena.sdf`: Task 2 fastest car slalom sprint world with 2 obstacles and directional arrows.
  - `arena_empty.sdf`: Clean 2.0m × 2.0m arena with start carpark and boundary walls for free-roam teleop and calibration.
- **Quick Commands** (from `ros2_ws/`):
  - `pixi run -e pc sim-gz` — Launch 3D Gazebo simulation
  - `pixi run -e pc sim-teleop` — Keyboard teleoperation
  - `pixi run -e pc sim-robot` — Full Task 1 autonomy (planner + YOLO perception + motion controller + RViz)
  - `pixi run -e pc sim-task2` — Task 2 fastest car slalom sprint
  - See [`ros2_ws/src/mdp_simulation/README.md`](ros2_ws/src/mdp_simulation/README.md) for full instructions.

## Team

- Robot & STM32 firmware: TBD
- RPi & image processing: TBD
- Algorithm: TBD
- Android: TBD
