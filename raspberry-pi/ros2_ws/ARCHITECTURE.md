# MDP ROS2 Architecture

SC2079 Group 16 — ROS2 Jazzy implementation of the Raspberry Pi module,
living at `raspberry-pi/ros2_ws/` in this repo. It implements a
**host-side kinematics** architecture: the STM32 is a dumb hardware I/O
controller, and everything that thinks (control, sensor fusion,
perception, path planning) runs on the ROS2 host.

This is a parallel track to the non-ROS bespoke-socket implementation in
[`raspberry-pi/cv/`](../cv/) (raw Bluetooth/UART/TCP, no ROS, alongside
the existing [`docs/protocol.md`](../../docs/protocol.md)). The trained
CV weights (`Frieddeli/mdp-symbols` on HF) and all the course-material
research already captured in this repo still apply here — this doc is
about how the same problem is wired up as a ROS2 graph instead.

## System diagram

```mermaid
flowchart TB
    subgraph Host["Host: RPi4B or Host PC (TBD, see Open Decisions)"]
        ROS["ROS2 Jazzy Stack<br/>Task 1 and 2 autonomy<br/>ros2_control + EKF fusion<br/>YOLO perception"]
        AndroidBridge["android_bridge_node<br/>pyserial, RFCOMM"]
        HWBridge["mdp_hardware_bridge<br/>serial_bridge_node"]
        ROS <-->|topics/services| AndroidBridge
        ROS <-->|topics/services| HWBridge
    end

    Gazebo["Gazebo Simulation<br/>3D physics engine"]
    STM32["STM32 MCU: mdp_stm32<br/>PlatformIO / STM32Cube HAL<br/>Rear motor PWM: AT8236<br/>Steering servo PWM: HWZ020<br/>Encoders + ICM-20948 IMU"]
    Android["Android Tablet<br/>Remote app: 2D arena and controls"]

    ROS <-->|sim bridge| Gazebo
    HWBridge <-->|Serial UART, USART3 at 115200| STM32
    AndroidBridge <-->|Bluetooth Serial RFCOMM| Android
```

## Design principle: host-side kinematics

The STM32 does **no** path planning, no odometry integration, no control
loops beyond raw PWM output — it exposes motor/servo actuation and raw
sensor reads (encoders, IMU) over UART and nothing else. All of the
following live on the ROS2 host:

- Odometry integration and sensor fusion (`robot_localization` EKF, fed
  by encoder + IMU data relayed from the STM32)
- `ros2_control` hardware interface + controllers (turns ROS `cmd_vel`
  into the STM32's PWM commands, and STM32 encoder reads into ROS
  odometry)
- Perception (YOLO detection of the 31-class symbol set)
- Path planning / autonomy (Task 1 image recognition run, Task 2 fastest
  car run)

This mirrors the course's own recommended split (STM32 = motor control
board, RPi = "brain") but pushes *more* onto the host than the course's
bespoke-socket reference architecture does, since ROS2 gives us
`ros2_control` + `robot_localization` instead of hand-rolled odometry.

## Nodes

| Node                                             | Package (planned)                                | Responsibility                                                                                                                                                                                                                                                                                                               |
| ------------------------------------------------ | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mdp_hardware_bridge` / `serial_bridge_node` | `mdp_hardware_bridge`                          | UART link to STM32 (USART3 @ 115200). Translates`ros2_control` hardware interface calls into PWM commands, and STM32 encoder/IMU frames into ROS messages.                                                                                                                                                                 |
| `android_bridge_node`                          | `mdp_android_bridge`                           | Bluetooth RFCOMM link to the Android tablet (`/dev/rfcommN`, via `pyserial`). Relays remote-control commands in and status/telemetry out — same role as `raspberry-pi/android_bridge` in the non-ROS implementation, reimplemented as a ROS node.                                                                               |
| perception node(s)                               | `mdp_perception`                               | Runs the trained YOLO model (`Frieddeli/mdp-symbols` weights) on camera frames, publishes `vision_msgs/Detection2D` results. Camera capture via a libcamera-based ROS driver (see Open Decisions — **not** the legacy `picamera` module).                                                                       |
| `ekf_node`                                     | `robot_localization` (stock)                   | Fuses encoder odometry + IMU into a filtered pose estimate.                                                                                                                                                                                                                                                                  |
| `controller_manager` + controllers             | `ros2_control` / `ros2_controllers` (stock)  | Owns the hardware interface abstraction and the drive controller (e.g.`diff_drive_controller` or a custom car-kinematics controller, since this is a car with steering, not a differential-drive base).                                                                                                                    |
| autonomy / path planner                          | `mdp_bringup` or a new `mdp_planner` package | Implements the course-required Hamiltonian-path ordering (nearest-neighbour + 2-opt / exhaustive search over the 5 obstacles) and Dubins path segments between configurations.**This is graded coursework — must be self-implemented, not a stock Nav2 planner.** See `mdp_bringup`'s eventual `planner/` module. |
| Gazebo sim bridge                                | `ros_gz_bridge` (stock)                        | Bridges ROS topics ↔ Gazebo Harmonic for simulating the robot/arena before/alongside real hardware runs — satisfies the course's "simulate the physical robot and algorithms in software" requirement.                                                                                                                     |

`mdp_bringup` (already scaffolded under `src/`) is the launch/config entry
point that will eventually bring all of the above up together.

## Topics, services, and messages

Concrete interface contracts between nodes. Anything not marked "(stock)"
needs a custom message and belongs in a new `mdp_interfaces` package
(see below) — `vision_msgs`/`geometry_msgs` don't have a clean slot for
this task's specific fields (numeric class IDs 11–40, `marker`,
`UNCERTAIN`, obstacle IDs).

| Topic / service | Type | Publisher → Subscriber(s) | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` (stock) | planner → `controller_manager` | Standard `ros2_control` input; the drive controller converts this into rear-motor velocity + steering angle. |
| `/odom` | `nav_msgs/Odometry` (stock) | drive controller → `ekf_node`, planner | Raw wheel-encoder odometry, pre-fusion. |
| `/imu/data_raw` | `sensor_msgs/Imu` (stock) | `mdp_hardware_bridge` → `ekf_node` | ICM-20948 readings relayed from STM32. |
| `/odometry/filtered` | `nav_msgs/Odometry` (stock) | `ekf_node` → planner, `mdp_bringup` | Fused pose estimate; this is what the planner should treat as ground truth for `(x, y, θ)`. |
| `/camera/image_raw` | `sensor_msgs/Image` (stock) | camera driver node → `mdp_perception` | From the libcamera-based driver (see Open Decisions). |
| `/detections` | `mdp_interfaces/ObstacleDetection` (custom) | `mdp_perception` → planner, `mdp_bringup` | One message per recognized obstacle: `obstacle_id`, `label` (`"11"`–`"40"` or `"marker"` or `"UNCERTAIN"`), `confidence`, `image` (`sensor_msgs/Image`, for the verification stitch). Mirrors the `image_result` JSON already drafted in the non-ROS implementation's `docs/protocol.md`. |
| `/android/cmd` | `std_msgs/String` (stock, or a thin custom msg) | `android_bridge_node` → planner / teleop mux | Parsed remote-control commands from the tablet. |
| `/android/status` | `std_msgs/String` (stock) | planner/bridge → `android_bridge_node` | Curated status text relayed to the tablet — same intent as the non-ROS protocol's `STATUS,<text>` line. |
| `~/plan_run` (service or action) | `mdp_interfaces/PlanRun` (custom) | UI/bringup → planner | Kicks off the Hamiltonian-path + Dubins planning run given the known obstacle list; an action (not a plain service) if you want progress feedback as each obstacle is visited. |
| `/gz/...` bridge topics | `ros_gz_bridge`-generated (stock) | Gazebo ↔ ROS graph | Auto-generated from the bridge YAML config once the robot's SDF/URDF exists. |

## TF tree

```
map
 └── odom
      └── base_link
           ├── camera_link   (front-center, per URDF below)
           ├── imu_link
           └── (wheel/steering joint frames, from ros2_control's URDF)
```

- `odom → base_link`: published by the drive controller / `ekf_node`
  (whichever owns odometry — typically the EKF republishes a filtered
  `odom → base_link` transform).
- `map → odom`: only needed if you localize against a fixed map of the
  200×200cm arena; for Task 1/2 the arena is fully known ahead of time,
  so this may just be a static identity transform rather than a real
  localization problem — worth confirming with the team before adding
  AMCL or similar.

## Robot description (URDF/xacro) — grounded in the course spec

Not yet created (`ros-jazzy-xacro`/`robot_state_publisher` are installed
but there's no `.urdf.xacro` in the workspace yet). Numbers to encode,
taken directly from the Algorithms briefing:

| Quantity | Value | Source |
|---|---|---|
| Actual robot footprint | 20cm × 21cm | Algorithms briefing, "Robot's Environment" |
| Recommended *planning* footprint (padding) | 30cm × 30cm | Algorithms briefing, representation option 1/2 |
| Camera mount position | front-center | Algorithms briefing |
| Camera-to-obstacle ideal recognition distance | 20cm | Algorithms briefing |
| Turning radius | ~25cm (larger at speed) | Algorithms briefing |
| Arena size | 200cm × 200cm, virtual boundary | Algorithms briefing |
| Obstacle footprint | 10cm × 10cm | Algorithms briefing |
| Start zone | 40cm × 40cm, bottom-left corner at origin | Algorithms briefing |
| Robot pose convention | `(x, y, θ)`, `θ ∈ (-π, π]`, East = 0 | Algorithms briefing |

Note the unit mismatch to design around: the course spec and existing
non-ROS protocol work entirely in **centimeters**; ROS conventions
(URDF, `nav_msgs/Odometry`, TF) are entirely in **meters**. Pick the
conversion boundary deliberately (e.g. convert cm→m immediately at the
planner's input, keep everything downstream in meters) rather than
letting it leak across multiple nodes.

## STM32 serial protocol — open design tension

The non-ROS implementation's `docs/protocol.md` already defines a **discrete**
command set for RPi↔STM32: `FW:<mm>`, `BW:<mm>`, `TL:<deg>`, `TR:<deg>`,
`STP`, with the STM32 replying `ACK` then `DONE` once a move completes.
That protocol assumes the RPi issues one bounded move at a time and
waits.

`ros2_control` doesn't work that way — a `SystemInterface` writes
**continuous** command values (e.g. wheel velocity, steering angle) into
the hardware every control-loop tick (typically 50-100Hz) and reads
state back every tick; there's no notion of "move 50mm then tell me
you're done."

This is a real decision, not just a wiring detail:

1. **Change the STM32 firmware's contract** to accept a continuous
   `(velocity, steering_angle)` setpoint stream instead of discrete
   move commands, with a watchdog (stop if no new setpoint within
   ~200ms) — this is the "correct" `ros2_control` way, but means the
   STM32 firmware needs new commands beyond what `docs/protocol.md`
   currently specifies.
2. **Keep the discrete protocol and adapt at the bridge** — have
   `mdp_hardware_bridge` translate continuous `cmd_vel` into a stream of
   small discrete `FW:`/`TL:` commands. Simpler (no STM32 firmware
   change), but laggy and fights against what `ros2_control` is designed
   for.

Leaning toward (1), but this needs to be decided with whoever owns the
STM32 firmware before `mdp_hardware_bridge` is written, since it changes
what the firmware needs to expose.

## Perception message contract

- Labels: `marker` (bull's-eye, non-target) or numeric IDs `11`–`40`
  (digits 1–9 = 11–19; letters A,B,C,D,E,F,G,H,S,T,U,V,W,X,Y,Z = 20–35;
  arrows + stop = 36–40) — same 31-class set as the Roboflow dataset and
  the reference code's `data.yaml`, confirmed earlier.
- Below-threshold detections publish `label = "UNCERTAIN"` rather than a
  guessed class — mirrors the `CONF_THRESHOLD` gating already designed
  in the non-ROS implementation's `pi_infer.py`. `UNCERTAIN` is what should trigger
  the planner's reverse-and-recheck fallback (Algorithms briefing §2.3).
- The raw frame travels alongside the detection (`image` field) so
  something downstream can build the verification stitch the checklist
  requires ("stitched RAW images... shown as a single image") — don't
  reimplement stitching in two places; decide once whether that lives in
  `mdp_perception` or the planner/bringup package.

## Custom interfaces package (not yet created)

`mdp_interfaces` — holds `ObstacleDetection.msg`, and `PlanRun.action` (or
`.srv`) referenced in the topics table above. Needs to exist before
`mdp_perception` or the planner package can be written against it, so
it's a good first "real" package to scaffold after `mdp_bringup`.

## Launch files (planned, not yet written)

- `bringup.launch.py` — everything except Gazebo: hardware bridge,
  `controller_manager`, `ekf_node`, perception, planner. Used on real
  hardware.
- `sim.launch.py` — Gazebo + `ros_gz_bridge` + the same
  `controller_manager`/`ekf_node`/perception/planner stack, swapping the
  hardware interface for Gazebo's simulated one. Keeping the same
  planner/perception nodes between sim and real hardware (only the
  hardware interface changes) is what makes the "simulate the algorithms
  in software" checklist item actually meaningful rather than a
  throwaway demo.
- `teleop.launch.py` — just the Android bridge + hardware bridge, for
  early integration testing before the planner exists (equivalent to the
  non-ROS implementation's Week 2 "prove every comms link works" milestone).

## Hardware topology

- **STM32 (`mdp_stm32` firmware, PlatformIO/STM32Cube HAL):** rear motor
  PWM via AT8236 driver, steering servo PWM via HWZ020, wheel encoders,
  ICM-20948 IMU. Talks UART only (USART3 @ 115200) — no logic beyond
  reading sensors and writing PWM on command.
- **Host:** runs the full ROS2 graph. Physical placement (on the RPi4B
  itself vs. an external host PC) is not yet decided — see below.
- **Android tablet:** remote controller app, 2D arena display, connects
  over classic Bluetooth SPP/RFCOMM (matches the course's AMDTOOL-tested
  protocol, not BLE).

## Software stack (pixi-ros, ROS2 Jazzy)

Managed via `pixi.toml` in this workspace (robostack-jazzy + conda-forge
channels), resolved and locked for three platforms: `osx-arm64` (local
dev on this Mac), `linux-aarch64` (the RPi4B's real architecture — see
"Where does the ROS2 host actually run?" above), and `linux-64` (a Linux
laptop/host PC with an NVIDIA GPU, in case perception ends up offloaded
there rather than run on the Pi — same Track A/Track B split as the
non-ROS implementation's CV pipeline). All three added and verified 2026-08-24.

Note on `linux-64` + GPU: the `ultralytics` PyPI dependency pulls in
`torch`, and PyPI's Linux `torch` wheels bundle CUDA by default (unlike
macOS, which is CPU/MPS-only) — but that's a packaging fact, not a
runtime one. Confirm CUDA is actually picked up on the target laptop
(`python -c "import torch; print(torch.cuda.is_available())"`) once
deployed there; the driver/CUDA-toolkit version on the laptop itself
still has to match what the wheel expects.

| Concern          | Package(s)                                                                                                                                                                            |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Core             | `ros-jazzy-ros-base`, `ros-jazzy-rclpy`, `ros-jazzy-ros2cli`                                                                                                                    |
| Control + fusion | `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-robot-localization`                                                                                          |
| Perception       | `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ros-jazzy-image-transport`, `ultralytics` (pip — not a ROS package)                                                         |
| Bridges          | `pyserial` (Bluetooth RFCOMM + UART), `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`                                                                                        |
| Simulation       | `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-interfaces`, `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-joint-state-publisher` |

## Open decisions

- **Where does the ROS2 host actually run?** The diagram labels it
  "RPi4B / Host PC" — still open, but **not** for OS-compatibility
  reasons. Earlier draft of this doc assumed ROS2-on-RPi4B would force
  Ubuntu Server over the course's recommended Raspberry Pi OS (Buster),
  since official ROS2 debs only target Ubuntu. That assumption doesn't
  apply here: `pixi-ros` installs ROS2 as self-contained robostack
  conda packages, never touching the system package manager, so it's
  agnostic to the underlying distro. Confirmed directly (not assumed) —
  `robostack-jazzy`'s `linux-aarch64` channel (the RPi4B's actual
  architecture) publishes every package this workspace depends on
  (`ros2-control`, `robot_localization`, `ros-gz-sim`, `cv-bridge`,
  etc.), and `pixi install` resolves and locks the full stack for
  `linux-aarch64` alongside `osx-arm64` with no gaps (added as a
  workspace platform in `pixi.toml`, verified 2026-08-24). So running
  the ROS2 host directly on the RPi4B under plain **Raspberry Pi OS**
  (matching the course's own setup guide, no Ubuntu detour) is fully
  viable.

  What's still genuinely open is **compute/thermal budget**, not
  packaging: whether the RPi4B alone can run `ros2_control` + EKF +
  YOLO perception + (optionally) the Gazebo bridge simultaneously at
  acceptable rates, or whether perception specifically should stay
  offloaded to a host PC/4060 (as in the non-ROS implementation's Track B) while
  control/fusion stays on the Pi. That's a benchmarking question once
  hardware is in hand, not a blocker to starting development.
- **Camera driver.** Must use a libcamera-based ROS node (`camera_ros`
  or `v4l2_camera` off the libcamera V4L2 device) — the legacy
  `picamera` module the course guide assumes is Buster-only and isn't a
  ROS concept at all.
- **Drive controller.** Needs a car-like (Ackermann-ish: rear drive +
  front steering servo) `ros2_control` controller, not the stock
  `diff_drive_controller` — not yet selected/written.
- **Planner package name/location.** Hamiltonian-path + Dubins planner
  not yet scaffolded as a package.
- **STM32 command interface: discrete vs. continuous.** See "STM32
  serial protocol" above — needs a decision with the STM32 firmware
  owner before `mdp_hardware_bridge` is written.
- **`map → odom` transform.** Static identity vs. real localization —
  the arena is fully known ahead of time, so a full localization stack
  (AMCL etc.) may be unnecessary overhead; not yet decided.
- **Where does image stitching live** — `mdp_perception` or the
  planner/bringup package? Only decide once, since `pi_infer.py` in the
  non-ROS implementation and this doc both flag "don't duplicate the stitching
  logic in two places."

## Course requirements this maps to

- "Simulate the physical robot and algorithms in software" → Gazebo +
  `ros_gz_bridge`.
- Task 1/2 autonomy, image recognition → perception node + planner
  package.
- System functionality checklist (comms, movement, image recognition) →
  the bridge nodes + `ros2_control` loop.

See the non-ROS implementation's `docs/week2-checklist.md` and
`docs/cv-integration-checklist.md` for the equivalent checklist items —
not yet ported to a ROS2-specific checklist.
