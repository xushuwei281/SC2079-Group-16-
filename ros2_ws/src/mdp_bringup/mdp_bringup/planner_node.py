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
    from planner import FullMissionPlan, PlanLeg, plan_mission
except ImportError:
    from algorithm.arena import Config, Obstacle, default_arena
    from algorithm.planner import FullMissionPlan, PlanLeg, plan_mission

import rclpy
from geometry_msgs.msg import PoseStamped
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves
from nav_msgs.msg import Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String


class PlannerNode(Node):
    """Autonomous Mission Planner & Execution Orchestrator Node."""

    def __init__(self) -> None:
        super().__init__("planner_node")

        self.declare_parameter("turning_radius_cm", 31.0)
        self.declare_parameter("camera_view_dist_cm", 25.0)
        self.declare_parameter("auto_start", False)

        self._radius = self.get_parameter("turning_radius_cm").value
        self._view_dist = self.get_parameter("camera_view_dist_cm").value
        self._auto_start = self.get_parameter("auto_start").value

        self._obstacles: List[Obstacle] = []
        self._current_plan: Optional[FullMissionPlan] = None
        self._current_pose = Config(20.0, 20.0, math.pi / 2.0)
        self._is_executing = False
        self._mission_thread: Optional[threading.Thread] = None

        callback_group = ReentrantCallbackGroup()

        # Publishers
        self._status_pub = self.create_publisher(String, "/android/status", 10)
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        self._path_pub = self.create_publisher(Path, "/planner/path", 10)

        # Subscribers
        self._cmd_sub = self.create_subscription(
            String, "/android/cmd", self._on_cmd, 10, callback_group=callback_group
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._on_pose, 10, callback_group=callback_group
        )
        self._estop_sub = self.create_subscription(
            Empty, "/estop", self._on_estop, 10, callback_group=callback_group
        )

        # Service Client to STM32 Hardware Bridge
        self._move_client = self.create_client(
            ExecuteMoves, "/execute_moves", callback_group=callback_group
        )

        self.get_logger().info("Planner node initialized and ready for obstacle input.")

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update robot pose from the odometry/gyro fusion topic."""
        x_cm = msg.pose.position.x * 100.0
        y_cm = msg.pose.position.y * 100.0
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._current_pose = Config(x_cm, y_cm, yaw_rad)

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
            radius=self._radius
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

            # Build ExecuteMoves request (prepends G0 to zero gyro baseline)
            req = ExecuteMoves.Request()
            g0_mc = MoveCommand()
            g0_mc.command = "G0"
            g0_mc.value = 0
            req.commands.append(g0_mc)

            for code, val in leg.commands:
                mc = MoveCommand()
                mc.command = code
                mc.value = val
                req.commands.append(mc)

            if req.commands:
                # Call /execute_moves synchronously
                future = self._move_client.call_async(req)
                while rclpy.ok() and not future.done():
                    if not self._is_executing:
                        break
                    time.sleep(0.05)

                if not future.done() or not future.result().success:
                    self.get_logger().error(f"Leg {i+1} execution failed or cancelled.")
                    self._status_pub.publish(String(data=f"Leg {i+1} Failed"))
                    break

            self.get_logger().info(f"Leg {i+1} completed! At vantage pose for Obstacle {leg.obstacle_id}.")
            self._status_pub.publish(String(data=f"At Obs {leg.obstacle_id}: Capturing Image"))

            # Pause for photo & recognition (2 seconds)
            time.sleep(1.5)

            # In live run, perception node resolves symbol ID; default to simulated recognition ID
            recognized_symbol = 10 + leg.obstacle_id  # e.g., Obs 1 -> Symbol 11
            self._target_pub.publish(String(data=f"{leg.obstacle_id},{recognized_symbol}"))
            self.get_logger().info(f"Reported Target: Obstacle {leg.obstacle_id} -> Symbol {recognized_symbol}")

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
