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

import math
import os
import sys
import threading
import time
from typing import List, Optional

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
    from arena import Config, Obstacle, default_arena
    from planner import (
        FullMissionPlan,
        PlanLeg,
        discretize_waypoints,
        plan_mission,
        sample_reeds_shepp_path,
    )
except ImportError:
    from algorithm.arena import Config, Obstacle, default_arena
    from algorithm.planner import (
        FullMissionPlan,
        PlanLeg,
        discretize_waypoints,
        plan_mission,
        sample_reeds_shepp_path,
    )

import rclpy
from geometry_msgs.msg import PoseStamped
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves
from nav_msgs.msg import Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String


class PlannerNode(Node):
    """Autonomous Mission Planner & Execution Orchestrator Node."""

    def __init__(self) -> None:
        super().__init__("planner_node")

        self.declare_parameter("turning_radius_cm", 42.0)
        self.declare_parameter("camera_view_dist_cm", 25.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("enable_collision_avoidance", True)
        self.declare_parameter("safety_stop_dist_cm", 12.0)
        self.declare_parameter("recovery_backup_cm", 8.0)
        self.declare_parameter("recognition_timeout_s", 3.0)

        self._radius = float(self.get_parameter("turning_radius_cm").value)
        self._view_dist = float(self.get_parameter("camera_view_dist_cm").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._enable_avoidance = bool(self.get_parameter("enable_collision_avoidance").value)
        self._safety_dist_cm = float(self.get_parameter("safety_stop_dist_cm").value)
        self._recovery_backup_cm = float(self.get_parameter("recovery_backup_cm").value)
        self._recognition_timeout_s = float(self.get_parameter("recognition_timeout_s").value)

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

    def _on_estop(self, msg: Empty) -> None:
        """Emergency stop handler."""
        self.get_logger().warn("Planner received E-STOP: Halting mission execution.")
        self._is_executing = False

    def _on_cmd(self, msg: String) -> None:
        """Handle incoming commands from the Android tablet or CLI."""
        raw = msg.data.strip()
        self.get_logger().info(f"Planner received command: {raw}")

        if raw.startswith("ALG|"):
            self._parse_and_plan(raw)
        elif raw.upper() == "START":
            self.start_mission()
        elif raw.upper() == "RESET":
            self.reset_mission()

    def _parse_and_plan(self, alg_str: str) -> None:
        """Parse obstacle list string (e.g. ALG|1,60,60,N|2,130,60,E) and compute plan."""
        tokens = alg_str.split("|")[1:]
        obstacles: List[Obstacle] = []

        for tok in tokens:
            parts = tok.split(",")
            if len(parts) >= 4:
                try:
                    obs_id = int(parts[0])
                    ox = int(parts[1])
                    oy = int(parts[2])
                    face = parts[3].strip().upper()
                    obstacles.append(Obstacle(id=obs_id, x=ox, y=oy, face=face))
                except ValueError as e:
                    self.get_logger().warn(f"Failed to parse obstacle token '{tok}': {e}")

        if not obstacles:
            self.get_logger().warn("No valid obstacles parsed from ALG command.")
            return

        self._obstacles = obstacles
        self.get_logger().info(f"Parsed {len(obstacles)} obstacles. Computing optimal Reeds-Shepp TSP plan...")

        self._current_plan = plan_mission(
            self._obstacles,
            start_pose=self._current_pose,
            radius=self._radius,
            d_view=self._view_dist
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

    def _execute_mission_loop(self) -> None:
        """Main autonomous execution loop running in worker thread."""
        self.get_logger().info("--- STARTING AUTONOMOUS MISSION ---")
        self._status_pub.publish(String(data="MISSION RUNNING"))

        if not self._move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Hardware bridge /execute_moves unavailable.")
            self._status_pub.publish(String(data="Hardware bridge offline"))
            self._is_executing = False
            return

        for i, leg in enumerate(self._current_plan.legs):
            if not self._is_executing:
                self.get_logger().warn("Mission execution interrupted.")
                break

            self.get_logger().info(
                f"\n>>> Executing Leg {i+1}/{len(self._current_plan.legs)} -> "
                f"Obstacle {leg.obstacle_id} ({leg.target_face} face) <<<"
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
                    current_start, leg.vantage_pose, radius=self._radius
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

                # Update live path in Foxglove
                sub_plan = FullMissionPlan(legs=[], total_distance_cm=length, all_commands=cmds, all_poses=sampled_poses)
                self._publish_ros_path(sub_plan)

                # Build ExecuteMoves request
                req = ExecuteMoves.Request()
                for code, val in cmds:
                    mc = MoveCommand()
                    mc.command = code
                    mc.value = val
                    req.commands.append(mc)

                # Execute with active sensor proximity monitoring
                future = self._move_client.call_async(req)
                interrupted_by_sensor = False

                while rclpy.ok() and not future.done():
                    if not self._is_executing:
                        break

                    # Active Proximity Safety Guard
                    if self._enable_avoidance:
                        min_dist_m = min(self._us_range_m, self._ir_left_range_m, self._ir_right_range_m)
                        if min_dist_m < (self._safety_dist_cm / 100.0):
                            self.get_logger().warn(
                                f"⚠️ PROXIMITY ALERT! Obstacle detected at {min_dist_m*100.0:.1f} cm (< {self._safety_dist_cm} cm). Triggering safety stop."
                            )
                            self._estop_pub.publish(Empty())
                            interrupted_by_sensor = True
                            break

                    time.sleep(0.05)

                if interrupted_by_sensor:
                    self._status_pub.publish(String(data=f"Obstacle Alert: Replanning Leg {i+1}..."))
                    time.sleep(0.3)  # Wait for motors to come to a complete stop

                    # Execute safe reverse recovery to gain turning clearance
                    if self._recovery_backup_cm > 0:
                        self.get_logger().info(f"Executing {self._recovery_backup_cm:.0f} cm reverse recovery move...")
                        backup_req = ExecuteMoves.Request()
                        mc = MoveCommand()
                        mc.command = "BC"
                        mc.value = int(self._recovery_backup_cm)
                        backup_req.commands.append(mc)
                        backup_future = self._move_client.call_async(backup_req)
                        while rclpy.ok() and not backup_future.done():
                            time.sleep(0.05)
                        time.sleep(0.5)  # Allow odometry to settle

                    self.get_logger().info(f"Recovered pose: ({self._current_pose.x:.1f}, {self._current_pose.y:.1f}). Re-planning leg {i+1}...")
                    continue  # Retry loop to re-plan trajectory from new resting pose

                if future.done() and future.result() and future.result().success:
                    leg_success = True
                    break
                else:
                    self.get_logger().error(f"Leg {i+1} move failed or was cancelled.")
                    break

            if not leg_success:
                self.get_logger().error(f"Leg {i+1} could not be completed.")
                self._status_pub.publish(String(data=f"Leg {i+1} Failed"))
                break

            self.get_logger().info(f"Leg {i+1} completed! At vantage pose for Obstacle {leg.obstacle_id}.")
            self._status_pub.publish(String(data=f"At Obs {leg.obstacle_id}: Capturing Image"))

            # Wait for perception_node to report a real recognition result
            # for this obstacle (it publishes on /android/target once it
            # sees a confident detection while we're parked at this vantage
            # pose).
            recognized_symbol = self._wait_for_recognition(
                leg.obstacle_id, timeout_s=self._recognition_timeout_s
            )
            if recognized_symbol is not None:
                self.get_logger().info(
                    f"Recognized Target: Obstacle {leg.obstacle_id} -> Symbol {recognized_symbol}"
                )
            else:
                self.get_logger().warn(
                    f"No recognition result for Obstacle {leg.obstacle_id} "
                    f"within {self._recognition_timeout_s:.1f}s."
                )

        if self._is_executing:
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
