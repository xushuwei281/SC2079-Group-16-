#!/usr/bin/env python3
"""ROS 2 Autonomous Path Planning & Mission Execution Node for SC2079 MDP.

Responsibilities:
- Receives arena obstacle layout from the Android tablet (/android/cmd).
- Runs the TSP & Reeds-Shepp trajectory optimizer.
- Sequentially executes movement batches via /execute_moves.
- Coordinates snap-and-recognize image classification at each vantage pose.
- Reports live status and target recognition results back to the tablet.
"""

from __future__ import annotations

from enum import Enum
import json
import math
import os
import re
import sys
import threading
import time
from typing import List, Optional, Tuple

# Locate algorithm directory
_curr_dir = os.path.dirname(os.path.abspath(__file__))
for _p in [
    os.path.join(_curr_dir, "../../../../algorithm"),
    os.path.join(_curr_dir, "../../../algorithm"),
    os.path.join(_curr_dir, "../../algorithm"),
    os.path.join(os.getcwd(), "algorithm"),
    os.path.join(os.getcwd(), "../algorithm"),
    "/home/mdp/dev/SC2079-Group-16/algorithm",
]:
    _abs_p = os.path.abspath(_p)
    if os.path.isdir(_abs_p) and _abs_p not in sys.path:
        sys.path.insert(0, _abs_p)

try:
    from arena import Config, Obstacle, default_arena, in_bounds, robot_collides_any
    from planner import (
        FullMissionPlan,
        PlanLeg,
        compute_vantage_pose,
        discretize_waypoints,
        plan_mission,
        sample_reeds_shepp_path,
    )
except ImportError:
    from algorithm.arena import Config, Obstacle, default_arena, in_bounds, robot_collides_any
    from algorithm.planner import (
        FullMissionPlan,
        PlanLeg,
        compute_vantage_pose,
        discretize_waypoints,
        plan_mission,
        sample_reeds_shepp_path,
    )

import rclpy
from geometry_msgs.msg import PoseStamped
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
from nav_msgs.msg import Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String


class MissionState(str, Enum):
    """FSM Mission States for Task 1 Autonomous Orchestration."""
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    AVOIDANCE_RECOVERY = "AVOIDANCE_RECOVERY"
    SAMPLING_TARGET = "SAMPLING_TARGET"
    ORBIT_RECOVERY = "ORBIT_RECOVERY"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    ESTOP = "ESTOP"


class PlannerNode(Node):
    """Autonomous Mission Planner & Execution Orchestrator Node."""

    def __init__(self) -> None:
        super().__init__("planner_node")

        self.declare_parameter("turning_radius_cm", 42.0)
        self.declare_parameter("camera_view_dist_cm", 25.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("enable_collision_avoidance", True)
        self.declare_parameter("safety_stop_dist_cm", 12.0)
        self.declare_parameter("recovery_backup_cm", 8.0)
        self.declare_parameter("recognition_timeout_s", 3.0)
        self.declare_parameter("enable_orbit_recovery", True)

        self._radius = float(self.get_parameter("turning_radius_cm").value)
        self._view_dist = float(self.get_parameter("camera_view_dist_cm").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._enable_avoidance = bool(self.get_parameter("enable_collision_avoidance").value)
        self._safety_dist_cm = float(self.get_parameter("safety_stop_dist_cm").value)
        self._recovery_backup_cm = float(self.get_parameter("recovery_backup_cm").value)
        self._recognition_timeout_s = float(self.get_parameter("recognition_timeout_s").value)
        self._enable_orbit_recovery = bool(self.get_parameter("enable_orbit_recovery").value)

        self._state = MissionState.IDLE
        self._obstacles: List[Obstacle] = []
        self._current_plan: Optional[FullMissionPlan] = None
        self._current_pose = Config(20.0, 20.0, math.pi / 2.0)
        self._is_executing = False
        self._mission_thread: Optional[threading.Thread] = None
        self._recognized_targets: dict[int, int] = {}

        # Live sensor distance tracking (in meters)
        self._us_range_m = float("inf")
        self._ir_left_range_m = float("inf")
        self._ir_right_range_m = float("inf")
        self._proximity_alert = False

        callback_group = ReentrantCallbackGroup()

        # Publishers
        self._status_pub = self.create_publisher(String, "/android/status", 10)
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        self._path_pub = self.create_publisher(Path, "/planner/path", 10)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        # Subscribers
        self._cmd_sub = self.create_subscription(
            String, "/android/cmd", self._on_cmd, 10, callback_group=callback_group
        )
        # perception_node is the sole publisher of recognition results on
        # /android/target (it tracks "At Obs <id>" status and reports the
        # real YOLO symbol there); the planner only consumes it here to
        # avoid two nodes racing to publish onto the same topic.
        self._target_sub = self.create_subscription(
            String, "/android/target", self._on_target, 10, callback_group=callback_group
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._on_pose, 10, callback_group=callback_group
        )
        self._estop_sub = self.create_subscription(
            Empty, "/estop", self._on_estop, 10, callback_group=callback_group
        )
        self._us_sub = self.create_subscription(
            Range, "/sensors/ultrasonic", self._on_us_range, 10, callback_group=callback_group
        )
        self._ir_left_sub = self.create_subscription(
            Range, "/sensors/ir_left", self._on_ir_left_range, 10, callback_group=callback_group
        )
        self._ir_right_sub = self.create_subscription(
            Range, "/sensors/ir_right", self._on_ir_right_range, 10, callback_group=callback_group
        )

        # Service Client to STM32 Hardware Bridge
        self._move_client = self.create_client(
            ExecuteMoves, "/execute_moves", callback_group=callback_group
        )

        # Service Client to Live Perception Consensus Sampler
        self._sample_client = self.create_client(
            SampleTarget, "/perception/sample_target", callback_group=callback_group
        )

        self.get_logger().info(
            f"Planner node initialized. Collision avoidance: {'ON' if self._enable_avoidance else 'OFF'} "
            f"(threshold: {self._safety_dist_cm} cm)"
        )

    def _on_us_range(self, msg: Range) -> None:
        """Track front ultrasonic range."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._us_range_m = msg.range
        else:
            self._us_range_m = float("inf")

    def _on_ir_left_range(self, msg: Range) -> None:
        """Track front-left IR sensor range."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._ir_left_range_m = msg.range
        else:
            self._ir_left_range_m = float("inf")

    def _on_ir_right_range(self, msg: Range) -> None:
        """Track front-right IR sensor range."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._ir_right_range_m = msg.range
        else:
            self._ir_right_range_m = float("inf")

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update robot pose from the odometry/gyro fusion topic."""
        x_cm = msg.pose.position.x * 100.0
        y_cm = msg.pose.position.y * 100.0
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._current_pose = Config(x_cm, y_cm, yaw_rad)

    def _on_target(self, msg: String) -> None:
        """Record a recognition result reported by perception_node."""
        parts = msg.data.split(",")
        if len(parts) != 2:
            return
        try:
            obs_id = int(parts[0])
            symbol_id = int(parts[1])
        except ValueError:
            return
        self._recognized_targets[obs_id] = symbol_id

    def _wait_for_recognition(self, obstacle_id: int, timeout_s: float) -> Optional[int]:
        """Poll for perception_node's recognition result for this obstacle."""
        self._recognized_targets.pop(obstacle_id, None)
        deadline = time.time() + timeout_s
        while self._is_executing and time.time() < deadline:
            symbol_id = self._recognized_targets.get(obstacle_id)
            if symbol_id is not None:
                return symbol_id
            time.sleep(0.05)
        return self._recognized_targets.get(obstacle_id)

    def _transition_state(self, new_state: MissionState, reason: str = "") -> None:
        """Centralized state machine transition helper with logging and tablet status updates."""
        old_state = self._state
        self._state = new_state
        msg = f"FSM: {old_state.value} -> {new_state.value}"
        if reason:
            msg += f" ({reason})"
        self.get_logger().info(msg)

    def _on_estop(self, msg: Empty) -> None:
        """Emergency stop handler."""
        self._transition_state(MissionState.ESTOP, "Received E-STOP signal")
        self._is_executing = False

    def _on_cmd(self, msg: String) -> None:
        """Handle incoming commands from the Android tablet or CLI."""
        raw = msg.data.strip()
        self.get_logger().info(f"Planner received command: {raw}")

        raw_upper = raw.upper()
        if raw_upper in ("START", "ALG:START", "ALG|START", "IR:START", "START_TASK1"):
            self.start_mission()
        elif raw_upper in ("RESET", "ALG:RESET"):
            self.reset_mission()
        elif raw.startswith("ALG|") or raw.startswith("ALG:") or "OBSTACLE" in raw_upper or raw.startswith("{") or raw.startswith("["):
            self._parse_and_plan(raw)
        elif raw_upper.startswith("ADD"):
            # Support ADD,id,x,y,face (e.g. ADD,1,2,9,N or ADD,1,60,60,N)
            parts = raw.split(",")
            if len(parts) >= 5:
                try:
                    obs_id = int(parts[1])
                    x = float(parts[2])
                    y = float(parts[3])
                    face = parts[4].strip().upper()
                    ox = int(x * 10 + 5) if x <= 20 else int(x)
                    oy = int(y * 10 + 5) if y <= 20 else int(y)
                    self._obstacles = [ob for ob in self._obstacles if ob.id != obs_id]
                    self._obstacles.append(Obstacle(id=obs_id, x=ox, y=oy, face=face))
                    self.get_logger().info(f"Added/Updated obstacle {obs_id} at ({ox}, {oy}, {face})")
                    self._status_pub.publish(String(data=f"Obs {obs_id} Added: {ox},{oy},{face}"))
                except ValueError:
                    pass
        else:
            self.get_logger().warn(f"Unrecognized planner command: {raw}")

    def _parse_and_plan(self, raw_str: str) -> None:
        """Parse obstacle list string (supports ALG|..., ALG:{...}, JSON, ADD) and compute TSP plan."""
        content = raw_str.strip()
        if content.startswith("ALG:") or content.startswith("ALG|"):
            content = content[4:].strip()

        obstacles: List[Obstacle] = []

        # Format 1: Reference Android App ALG:{Obstacle 1: [2,9,N,], ...}
        if "Obstacle" in raw_str or content.startswith("{"):
            pattern = r"Obstacle\s*(\d+)\s*:\s*\[([^\]]*)\]"
            matches = re.findall(pattern, raw_str, re.IGNORECASE)
            for obs_id_str, vals_str in matches:
                tokens = [v.strip() for v in vals_str.split(",") if v.strip()]
                if len(tokens) >= 3:
                    try:
                        obs_id = int(obs_id_str)
                        x = float(tokens[0])
                        y = float(tokens[1])
                        face = tokens[2].upper()
                        # Convert grid indices [0..19] to cm (center is x*10+5, y*10+5)
                        ox = int(x * 10 + 5) if x <= 20 else int(x)
                        oy = int(y * 10 + 5) if y <= 20 else int(y)
                        obstacles.append(Obstacle(id=obs_id, x=ox, y=oy, face=face))
                    except ValueError:
                        continue

        # Format 2: Pipe-delimited string (e.g. ALG|1,60,60,N|2,130,60,E)
        if not obstacles:
            tokens = [t.strip() for t in raw_str.split("|") if t.strip()]
            for tok in tokens:
                if tok.upper().startswith("ALG"):
                    continue
                parts = tok.split(",")
                if len(parts) >= 4:
                    try:
                        obs_id = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        face = parts[3].strip().upper()
                        ox = int(x * 10 + 5) if x <= 20 else int(x)
                        oy = int(y * 10 + 5) if y <= 20 else int(y)
                        obstacles.append(Obstacle(id=obs_id, x=ox, y=oy, face=face))
                    except ValueError:
                        continue

        # Format 3: JSON fallback (e.g. {"obstacles": [[x,y,face], ...]})
        if not obstacles and (content.startswith("{") or content.startswith("[")):
            try:
                data = json.loads(content)
                raw_list = data.get("obstacles", []) if isinstance(data, dict) else data
                for idx, item in enumerate(raw_list):
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        x, y, face = float(item[0]), float(item[1]), str(item[2]).upper()
                        ox = int(x * 10 + 5) if x <= 20 else int(x)
                        oy = int(y * 10 + 5) if y <= 20 else int(y)
                        obstacles.append(Obstacle(id=idx + 1, x=ox, y=oy, face=face))
            except Exception:
                pass

        if not obstacles:
            self.get_logger().warn(f"No valid obstacles could be parsed from: {raw_str}")
            return

        self._obstacles = obstacles
        self._transition_state(MissionState.PLANNING, f"Parsed {len(obstacles)} obstacles")
        self.get_logger().info(f"Computing optimal Reeds-Shepp TSP plan for {len(obstacles)} obstacles...")

        arena_map = default_arena()
        arena_map["obstacles"] = self._obstacles

        self._current_plan = plan_mission(
            self._obstacles,
            start_pose=self._current_pose,
            arena=arena_map,
            radius=self._radius,
            d_view=self._view_dist,
        )

        self.get_logger().info(
            f"Plan computed: {len(self._current_plan.legs)} legs, "
            f"Total Distance: {self._current_plan.total_distance_cm:.1f} cm, "
            f"{len(self._current_plan.all_commands)} commands."
        )

        # Publish status to Android
        self._status_pub.publish(
            String(data=f"Plan Ready: {len(self._current_plan.legs)} targets, {self._current_plan.total_distance_cm:.0f}cm")
        )

        # Publish ROS Path for visualization
        self._publish_ros_path(self._current_plan)

        if self._auto_start:
            self.start_mission()

    def _publish_ros_path(self, plan: FullMissionPlan) -> None:
        """Publish nav_msgs/Path message for RViz / Foxglove visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for x_cm, y_cm, theta in plan.all_poses:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x_cm / 100.0
            ps.pose.position.y = y_cm / 100.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)

    def start_mission(self) -> None:
        """Begin automated execution of the computed plan."""
        if not self._current_plan or not self._current_plan.legs:
            self.get_logger().warn("Cannot start mission: No plan available.")
            self._status_pub.publish(String(data="No Plan Available"))
            return

        if self._is_executing:
            self.get_logger().warn("Mission already in progress.")
            return

        self._is_executing = True
        self._mission_thread = threading.Thread(target=self._execute_mission_loop, daemon=True)
        self._mission_thread.start()

    def _execute_commands_sync(self, cmds: List[Tuple[str, int]], label: str = "") -> bool:
        """Synchronously dispatch a sequence of MoveCommands with active proximity sensor guard."""
        if not cmds:
            return True

        req = ExecuteMoves.Request()
        for code, val in cmds:
            mc = MoveCommand()
            mc.command = code
            mc.value = int(val)
            req.commands.append(mc)

        future = self._move_client.call_async(req)
        interrupted_by_sensor = False
        is_backward = all(code in {"BC", "BL", "BR", "BU"} for code, _ in cmds)

        while rclpy.ok() and not future.done():
            if not self._is_executing:
                return False

            # Active Proximity Safety Guard (only active during forward maneuvers)
            if self._enable_avoidance and not is_backward:
                min_dist_m = min(self._us_range_m, self._ir_left_range_m, self._ir_right_range_m)
                if min_dist_m < (self._safety_dist_cm / 100.0):
                    self.get_logger().warn(
                        f"⚠️ PROXIMITY ALERT ({label})! Obstacle at {min_dist_m*100.0:.1f} cm (< {self._safety_dist_cm} cm). Halting."
                    )
                    self._transition_state(MissionState.AVOIDANCE_RECOVERY, f"Proximity alert during {label}")
                    self._estop_pub.publish(Empty())
                    interrupted_by_sensor = True
                    break
            time.sleep(0.04)

        if interrupted_by_sensor:
            self._status_pub.publish(String(data="Obstacle Alert: Executing clearance backup..."))
            time.sleep(0.3)
            if self._recovery_backup_cm > 0:
                self.get_logger().info(f"Executing {self._recovery_backup_cm:.0f} cm reverse recovery move...")
                backup_req = ExecuteMoves.Request()
                mc = MoveCommand()
                mc.command = "BC"
                mc.value = int(self._recovery_backup_cm)
                backup_req.commands.append(mc)
                backup_future = self._move_client.call_async(backup_req)
                while rclpy.ok() and not backup_future.done():
                    time.sleep(0.04)
                time.sleep(0.4)
            return False

        if future.done() and future.result() and future.result().success:
            return True
        return False

    def _query_perception_sampler(self, obstacle_id: int) -> Optional[Tuple[int, str, float, bool]]:
        """Call /perception/sample_target service and return (symbol_id, symbol_name, confidence, is_marker) or None."""
        if not (self._sample_client.service_is_ready() or self._sample_client.wait_for_service(timeout_sec=0.3)):
            return None

        req = SampleTarget.Request()
        req.obstacle_id = obstacle_id
        t_start = time.time()
        future = self._sample_client.call_async(req)
        while rclpy.ok() and not future.done():
            if time.time() - t_start > 3.0:
                break
            time.sleep(0.02)

        if future.done() and future.result():
            res: SampleTarget.Response = future.result()
            if res.success:
                return (res.symbol_id, res.symbol_name, res.confidence, res.is_marker)
        return None

    def _inspect_adjacent_faces(
        self, leg: PlanLeg, target_ob: Obstacle, arena: Dict
    ) -> Optional[Tuple[int, str, float]]:
        """Algorithms Briefing §2.3: Orbit around obstacle to inspect adjacent faces.

        If a Bull's Eye marker is detected on the nominal face, the target image is located
        on one of the other faces. This method orbits to adjacent faces in sequence until a
        valid target symbol (11-39) is confirmed.
        """
        nominal_face = (leg.target_face or "N").upper()
        if nominal_face == "N":
            candidates = ["E", "W", "S"]
        elif nominal_face == "S":
            candidates = ["W", "E", "N"]
        elif nominal_face == "E":
            candidates = ["S", "N", "W"]
        else:  # W
            candidates = ["N", "S", "E"]

        self._transition_state(
            MissionState.ORBIT_RECOVERY,
            f"Bull's Eye on {nominal_face} face of Obs {leg.obstacle_id}"
        )

        for cand_face in candidates:
            if not self._is_executing:
                return None

            alt_ob = Obstacle(id=leg.obstacle_id, x=target_ob.x, y=target_ob.y, face=cand_face)
            alt_vantage = compute_vantage_pose(alt_ob, d_view=self._view_dist, arena=arena)

            # Safety check: boundary and collision
            if not (15.0 <= alt_vantage.x <= 185.0 and 15.0 <= alt_vantage.y <= 185.0):
                self.get_logger().info(f"Skipping {cand_face} face for Obs {leg.obstacle_id}: Vantage pose out of bounds.")
                continue

            if robot_collides_any(alt_vantage.x, alt_vantage.y, alt_vantage.theta, arena, safety_margin=2.0) is not None:
                self.get_logger().info(f"Skipping {cand_face} face for Obs {leg.obstacle_id}: Vantage pose collides.")
                continue

            # Plan trajectory from current pose to candidate vantage pose
            length, wps, sampled_poses, method = sample_reeds_shepp_path(
                self._current_pose, alt_vantage, radius=self._radius, arena=arena
            )
            if math.isinf(length) or length > 160.0 or not wps:
                self.get_logger().info(
                    f"Skipping {cand_face} face for Obs {leg.obstacle_id}: Trajectory unreachable or too long ({length:.1f}cm)."
                )
                continue

            cmds, raw_cmds = discretize_waypoints(wps)
            if not cmds:
                continue

            self.get_logger().info(
                f"🔄 Orbiting Obs {leg.obstacle_id} to inspect adjacent {cand_face} face "
                f"({length:.1f}cm, {len(cmds)} cmds): {' -> '.join(raw_cmds)}"
            )
            self._status_pub.publish(String(data=f"Obs {leg.obstacle_id}: Orbiting to {cand_face} face"))

            sub_plan = FullMissionPlan(start_pose=self._current_pose, legs=[], total_distance_cm=length, all_commands=cmds, all_poses=sampled_poses)
            self._publish_ros_path(sub_plan)

            move_ok = self._execute_commands_sync(cmds, label=f"Orbit to {cand_face}")
            if not move_ok or not self._is_executing:
                self.get_logger().warn(f"Orbit move to {cand_face} face failed or was interrupted.")
                continue

            # Re-sample perception at the adjacent vantage pose
            sample_result = self._query_perception_sampler(leg.obstacle_id)
            if sample_result:
                sid, sname, conf, is_marker = sample_result
                if not is_marker and 11 <= sid <= 39:
                    self.get_logger().info(
                        f"🎉 Success! Target confirmed on {cand_face} face: Symbol {sid} ({sname}) [conf={conf:.2f}]"
                    )
                    self._status_pub.publish(String(data=f"Obs {leg.obstacle_id}: ID {sid} ({sname}) on {cand_face}"))
                    self._target_pub.publish(String(data=f"{leg.obstacle_id},{sid}"))
                    return (sid, sname, conf)
                elif is_marker:
                    self.get_logger().info(f"Adjacent face {cand_face} also has a marker. Checking next face...")
                else:
                    self.get_logger().warn(f"Adjacent face {cand_face} produced uncertain symbol ID {sid}.")

        self.get_logger().warn(f"Exhausted candidate adjacent faces for Obs {leg.obstacle_id}.")
        return None

    def _execute_mission_loop(self) -> None:
        """Main autonomous execution loop running in worker thread."""
        self._transition_state(MissionState.NAVIGATING, "Starting autonomous mission")
        self._status_pub.publish(String(data="MISSION RUNNING"))

        if not self._move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Hardware bridge /execute_moves unavailable.")
            self._status_pub.publish(String(data="Hardware bridge offline"))
            self._transition_state(MissionState.IDLE, "Hardware bridge offline")
            self._is_executing = False
            return

        arena_map = default_arena()
        arena_map["obstacles"] = self._obstacles

        for i, leg in enumerate(self._current_plan.legs):
            if not self._is_executing:
                self.get_logger().warn("Mission execution interrupted.")
                break

            self.get_logger().info(
                f"\n>>> Executing Leg {i+1}/{len(self._current_plan.legs)} -> "
                f"Obstacle {leg.obstacle_id} ({leg.target_face} face) <<<"
            )
            self._transition_state(
                MissionState.NAVIGATING, f"Leg {i+1}/{len(self._current_plan.legs)} -> Obs {leg.obstacle_id}"
            )
            self._status_pub.publish(String(data=f"Navigating to Obs {leg.obstacle_id}"))

            leg_success = False
            max_retries = 3

            for attempt in range(max_retries):
                if not self._is_executing:
                    break

                # 1. Dynamically plan trajectory from current live resting pose to target vantage pose
                current_start = self._current_pose
                length, wps, sampled_poses, method = sample_reeds_shepp_path(
                    current_start, leg.vantage_pose, radius=self._radius, arena=arena_map
                )
                cmds, raw_cmds = discretize_waypoints(wps)
                if not cmds and attempt == 0:
                    cmds = leg.commands  # fallback to nominal commands if already in vicinity

                if not cmds:
                    self.get_logger().info("Already at vantage pose.")
                    leg_success = True
                    break

                self.get_logger().info(
                    f"Leg {i+1} (Attempt {attempt+1}) planned from ({current_start.x:.1f}, {current_start.y:.1f}, {math.degrees(current_start.theta):.0f}°): "
                    f"{' -> '.join(raw_cmds or [f'{c}{v:03d}' for c, v in cmds])}"
                )

                # Update live path in RViz / Foxglove
                sub_plan = FullMissionPlan(start_pose=self._current_pose, legs=[], total_distance_cm=length, all_commands=cmds, all_poses=sampled_poses)
                self._publish_ros_path(sub_plan)

                # Execute with active proximity monitoring and automatic backup on obstruction
                move_ok = self._execute_commands_sync(cmds, label=f"Leg {i+1} Attempt {attempt+1}")
                if move_ok:
                    leg_success = True
                    break
                else:
                    self.get_logger().warn(f"Leg {i+1} attempt {attempt+1} interrupted. Re-planning from new pose...")
                    time.sleep(0.3)

            if not leg_success:
                self.get_logger().error(f"Leg {i+1} could not be completed.")
                self._status_pub.publish(String(data=f"Leg {i+1} Failed"))
                break

            self.get_logger().info(f"Leg {i+1} completed! At vantage pose for Obstacle {leg.obstacle_id}.")
            self._transition_state(MissionState.SAMPLING_TARGET, f"At Obs {leg.obstacle_id}")
            self._status_pub.publish(String(data=f"At Obs {leg.obstacle_id}: Sampling Target"))

            # 1. Query live perception consensus sampler service
            sample_res = self._query_perception_sampler(leg.obstacle_id)
            recognized_symbol = None

            if sample_res:
                sid, sname, conf, is_marker = sample_res
                if not is_marker and 11 <= sid <= 39:
                    recognized_symbol = sid
                    self.get_logger().info(
                        f"⚡ Target Confirmed: Obstacle {leg.obstacle_id} -> Symbol {sid} ({sname}) [Conf: {conf:.2f}]"
                    )
                    self._status_pub.publish(String(data=f"Obs {leg.obstacle_id}: ID {sid} ({sname})"))
                    self._target_pub.publish(String(data=f"{leg.obstacle_id},{sid}"))
                elif is_marker:
                    self.get_logger().warn(
                        f"⚠️ Bull's Eye detected at Obstacle {leg.obstacle_id}! Target image is on adjacent face."
                    )
                    self._status_pub.publish(String(data=f"Obs {leg.obstacle_id}: Marker (Bull's Eye)"))
                    if self._enable_orbit_recovery:
                        target_ob = next((ob for ob in self._obstacles if ob.id == leg.obstacle_id), None)
                        if target_ob:
                            orbit_res = self._inspect_adjacent_faces(leg, target_ob, arena_map)
                            if orbit_res:
                                recognized_symbol = orbit_res[0]
                else:
                    self.get_logger().warn(f"No confident target at Obstacle {leg.obstacle_id}.")
                    self._status_pub.publish(String(data=f"Obs {leg.obstacle_id}: Unrecognized"))
                    if self._enable_orbit_recovery:
                        target_ob = next((ob for ob in self._obstacles if ob.id == leg.obstacle_id), None)
                        if target_ob:
                            orbit_res = self._inspect_adjacent_faces(leg, target_ob, arena_map)
                            if orbit_res:
                                recognized_symbol = orbit_res[0]
            else:
                self.get_logger().warn(f"Perception sampler offline/timed out; checking topic /android/target...")
                recognized_symbol = self._wait_for_recognition(
                    leg.obstacle_id, timeout_s=self._recognition_timeout_s
                )

            if recognized_symbol is None:
                # Standalone fallback if perception node is offline or exhausted
                self.get_logger().info(f"Using nominal fallback ID for Obstacle {leg.obstacle_id}")
                recognized_symbol = 10 + leg.obstacle_id
                self._target_pub.publish(String(data=f"{leg.obstacle_id},{recognized_symbol}"))

        if self._is_executing:
            self._transition_state(MissionState.MISSION_COMPLETE, "All obstacles visited")
            self.get_logger().info("=== ALL TARGETS VISITED SUCCESSFULLY ===")
            self._status_pub.publish(String(data="MISSION COMPLETE"))

        self._is_executing = False

    def reset_mission(self) -> None:
        """Reset mission state."""
        self._is_executing = False
        self._current_plan = None
        self.get_logger().info("Mission reset.")
        self._status_pub.publish(String(data="Mission Reset"))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
