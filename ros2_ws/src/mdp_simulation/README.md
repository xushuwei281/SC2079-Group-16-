# `mdp_simulation` — Gazebo Harmonic Simulation for SC2079 MDP

This package provides a Gazebo Harmonic (gz-sim 8) 3D simulation environment for the SC2079 Multi-Disciplinary Design Project (Group 16) robot car and competition arenas, built in strict adherence to official SC2079 technical briefing materials.

---

## 1. Robot Kinematics & 3D Model

### Real Wheeltec Mini-Ackermann CAD Integration

The robot model in [`urdf/mdp_car.urdf.xacro`](urdf/mdp_car.urdf.xacro) is built from the official course CAD STL meshes located in `meshes/mini_akm_robot_meshes/` (extracted from `wheeltec_robot_urdf`):

- **Base Chassis (`base_link.STL`)**: 22.96 cm length × 15.36 cm width × 5.5 cm height aluminum baseplate with motor cutouts and M2/M3 standoff mounts.
- **Electronics Deck (`controller_link.STL`)**: Acrylic mounting deck with the STM32F407VET6 controller board.
- **Steering Linkage (`front_link.STL`, `left_link.STL`, `right_link.STL`)**: Functional Ackermann steering tie-rod and kingpin steering knuckles on the front axle.
- **Wheels (`lb_link.STL`, `rb_link.STL`, `lf_link.STL`, `rf_link.STL`)**: 66.4 mm diameter treaded rubber tires with 26 mm width and calibrated ground friction.
- **Camera Mount (`camera_link.STL`)**: Front bumper mount with Pi Camera v2.1.

### Physical Kinematics

- **Wheelbase ($L$)**: Exactly 16.0 cm (0.16 m) between front and rear axles.
- **Track Width ($W$)**: 15.0 cm (0.15 m) between left and right wheel centres.
- **Turning Radius ($R$)**: Calibrated minimum 21.0 cm (0.21 m) radius at 0.52 rad full steering lock ($\approx 30^\circ$), matching the Reeds–Shepp path planner.
- **Collision Envelope**: Central body collision box (`0.21 m × 0.10 m × 0.05 m`) maintaining a **2 cm clearance buffer** to the wheels with `<self_collide>false</self_collide>`, preventing physics binding and contact jitter.

### Simulated Sensor Suite

- **Pi Camera v2.1**: 640×480 @ 30 FPS, 62.2° horizontal FOV, published to `/camera/image_raw`.
- **HC-SR04 Ultrasonic Sensor**: 15° beam cone, 0.02 m to 3.00 m range, published to `/sensors/ultrasonic` and `/sensors/ultrasonic/raw`.
- **Dual Sharp GP2Y0A21YK IR Sensors**: Left (`/sensors/ir_left`) and right (`/sensors/ir_right`) rangefinders (0.10 m to 0.80 m range) on the front bumper.
- **6-DoF IMU**: High-rate gyro and accelerometer published to `/model/mdp_car/sensors/imu`.

---

## 2. Multiple Arena Worlds

The package provides **3 dedicated arena worlds** under [`worlds/`](worlds/):

| World File                                   | Purpose                               | Features                                                                                                                                                                                                            |
| :------------------------------------------- | :------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **`worlds/arena.sdf`** *(Default)* | **Task 1: Image Recognition**   | 2.0 m × 2.0 m floor, 10 cm grid, 40 cm green start carpark at`[0, 0]`, 5 cm perimeter boundary, and the 5 canonical course obstacles with YOLO target textures (1, B, W, 4, E) and Bull's Eye visual markers.    |
| **`worlds/task2_arena.sdf`**         | **Task 2: Fastest Car Sprint**  | 2.0 m × 2.0 m sprint world per MDP Briefing Slides 16–17 with 40 cm carpark and 2 obstacles placed 60 cm apart along the lane displaying Left and Right directional arrows.                                       |
| **`worlds/arena_empty.sdf`**         | **Open Driving & Dynamic Maze** | Clean 2.0 m × 2.0 m grid and perimeter walls with**no fixed obstacles**. Ideal for manual teleop, odometry calibration, straight-line distance tests, or dynamic obstacle placement from the Android tablet. |

---

## 3. How to Use the Simulation

All simulation tasks are executed using Pixi from the [`ros2_ws/`](../..) directory.

### A. Launch Base Simulation

Launch Gazebo with the robot spawned in the arena and all ROS bridges running:

```bash
cd ros2_ws

# 1. Launch Task 1 Competition Arena (Default)
pixi run -e pc ros2 launch mdp_simulation gz_sim.launch.py headless:=false

# 2. Launch Task 2 Fastest Car Arena
pixi run -e pc ros2 launch mdp_simulation gz_sim.launch.py headless:=false world:=$(ros2 pkg prefix --share mdp_simulation)/worlds/task2_arena.sdf

# 3. Launch Empty / Free-Roam Arena
pixi run -e pc ros2 launch mdp_simulation gz_sim.launch.py headless:=false world:=$(ros2 pkg prefix --share mdp_simulation)/worlds/arena_empty.sdf
```

*Note: If `headless:=false` is omitted, the launch script auto-detects if a graphical `DISPLAY` is attached. On remote headless servers or CI, it runs with `-s` (headless server).*

---

### B. Keyboard Teleoperation

In a separate terminal, launch the Ackermann keyboard teleop node:

```bash

cd ros2_ws
pixi run -e pc sim-teleop
```

**Keybindings:**

- `w` / `x` : Increase / decrease linear forward speed
- `a` / `d` : Steer front wheels left / right
- `s` or `Space` : Immediate stop
- `q` / `z` : Increase / decrease max speed

---

### C. Full Autonomous Task 1 Mission

To launch the complete autonomy stack (Simulation + Motion Controller + Reeds–Shepp Planner + YOLO Perception + RViz2 visualization):

```bash
cd ros2_ws
pixi run -e pc sim-robot
```

**What happens during the run:**

1. Gazebo 3D simulation starts with the 5 canonical course obstacles.
2. RViz2 opens displaying the 2.0 m × 2.0 m grid, planned Reeds–Shepp path, and live robot footprint.
3. The robot visits each obstacle vantage point, holds position, runs YOLO inference on the camera frame, registers crops, and generates `stitched_verification.jpg`.
4. If an obstacle face has a Bull's Eye visual marker instead of a target symbol, orbit recovery is automatically executed to find the active face.

---

### D. Full Task 2 (Fastest Car Reactive Sprint)

To run the speed-optimized slalom sprint around the two arrow obstacles:

```bash
cd ros2_ws
pixi run -e pc sim-task2
```

**What happens during the run:**

1. Car launches from the carpark towards Obstacle 1.
2. Stops at vantage distance using ultrasonic sensor feedback.
3. Detects Left Arrow (Symbol 39) vs Right Arrow (Symbol 38) via `/perception/sample_target`.
4. Executes a calibrated slalom bypass around Obstacle 1 in the indicated direction.
5. Sprints to Obstacle 2, scans Arrow 2, slaloms around Obstacle 2, loops around, and sprints back into the carpark.

---

## 4. Topic & Telemetry Mapping

The simulation bridge (`sim_bridge_node`) maps Gazebo topics into standard ROS 2 topics matching physical robot hardware:

| Gazebo Simulator Topic                | ROS 2 Topic                          | ROS Type                          | Description                            |
| :------------------------------------ | :----------------------------------- | :-------------------------------- | :------------------------------------- |
| `/cmd_vel`                          | `/cmd_vel`                         | `geometry_msgs/msg/Twist`       | Ackermann velocity & steering commands |
| `/model/mdp_car/odometry`           | `/robot_pose`, `/robot_pose/raw` | `geometry_msgs/msg/PoseStamped` | Odometry in arena`odom` frame        |
| `/model/mdp_car/odometry`           | `/tf`                              | `tf2_msgs/msg/TFMessage`        | Broadcasts`odom -> base_link`        |
| `/camera/image_raw`                 | `/camera/image_raw`                | `sensor_msgs/msg/Image`         | 640×480 RGB8 camera stream            |
| `/model/mdp_car/sensors/ultrasonic` | `/sensors/ultrasonic`, `.../raw` | `sensor_msgs/msg/Range`         | HC-SR04 sonar distance (meters)        |
| `/model/mdp_car/sensors/ir_left`    | `/sensors/ir_left`, `.../raw`    | `sensor_msgs/msg/Range`         | Left Sharp IR distance (meters)        |
| `/model/mdp_car/sensors/ir_right`   | `/sensors/ir_right`, `.../raw`   | `sensor_msgs/msg/Range`         | Right Sharp IR distance (meters)       |
| `/model/mdp_car/sensors/imu`        | `/sensors/imu`                     | `sensor_msgs/msg/Imu`           | 6-DoF angular velocity & linear accel  |

---

## 5. Inspection & Debugging Commands

While the simulation is running, open another terminal to inspect live data:

```bash
cd ros2_ws

# Live robot position (x, y in meters, orientation quaternion)
pixi run -e pc ros2 topic echo /robot_pose --once

# Live ultrasonic distance
pixi run -e pc ros2 topic echo /sensors/ultrasonic --once

# Live front IR sensors
pixi run -e pc ros2 topic echo /sensors/ir_left --once
pixi run -e pc ros2 topic echo /sensors/ir_right --once

# Send a manual velocity command (0.2 m/s forward, 0.1 rad/s turn)
pixi run -e pc ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once

# Inspect camera info
pixi run -e pc ros2 topic echo /camera/camera_info --once
```
