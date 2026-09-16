#!/usr/bin/env python3
"""Simulation Hardware Bridge Node for SC2079 MDP.

Acts as a drop-in simulated replacement for serial_bridge_node:
- Bridges Gazebo odometry to /robot_pose, /robot_pose/raw and broadcasts odom->base_link TF.
- Bridges Gazebo LaserScan range sensors to sensor_msgs/Range for ultrasonic, ir_left, and ir_right.
- Provides /hardware/maintenance service returning FIN.
- Listens to /android/cmd for RESET.
- Ensures all published sensor/pose messages are freshly stamped for motion_controller_node.
"""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TransformStamped
from mdp_interfaces.srv import ExecuteMoves
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Empty, String
import tf2_ros


class SimBridgeNode(Node):
    """Bridges Gazebo simulation sensor and odometry streams to MDP ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__("sim_bridge_node")

        self.declare_parameter("initial_x", 0.20)
        self.declare_parameter("initial_y", 0.20)
        self.declare_parameter("initial_yaw", math.pi / 2.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self._initial_x = float(self.get_parameter("initial_x").value)
        self._initial_y = float(self.get_parameter("initial_y").value)
        self._initial_yaw = float(self.get_parameter("initial_yaw").value)
        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        # Coordinate offsets if Gazebo odometry starts at (0, 0)
        # We map Gazebo odom to arena coordinates starting at initial_x, initial_y, initial_yaw
        self._offset_x = self._initial_x
        self._offset_y = self._initial_y
        self._offset_yaw = self._initial_yaw
        self._first_odom_received = False
        self._gz_origin_x = 0.0
        self._gz_origin_y = 0.0
        self._gz_origin_yaw = 0.0

        callback_group = ReentrantCallbackGroup()

        # Pose & Range Publishers matching serial_bridge_node
        self._pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self._raw_pose_pub = self.create_publisher(PoseStamped, "/robot_pose/raw", 1)
        self._us_pub = self.create_publisher(Range, "/sensors/ultrasonic", 10)
        self._raw_us_pub = self.create_publisher(Range, "/sensors/ultrasonic/raw", 1)
        self._ir_left_pub = self.create_publisher(Range, "/sensors/ir_left", 10)
        self._raw_ir_left_pub = self.create_publisher(Range, "/sensors/ir_left/raw", 1)
        self._ir_right_pub = self.create_publisher(Range, "/sensors/ir_right", 10)
        self._raw_ir_right_pub = self.create_publisher(Range, "/sensors/ir_right/raw", 1)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Maintenance service for compatibility
        self._maint_srv = self.create_service(
            ExecuteMoves,
            "/hardware/maintenance",
            self._handle_maintenance,
            callback_group=callback_group,
        )

        # Subscriptions to Gazebo bridge topics
        self._odom_sub = self.create_subscription(
            Odometry,
            "/model/mdp_car/odometry",
            self._on_odom,
            10,
            callback_group=callback_group,
        )

        self._us_scan_sub = self.create_subscription(
            LaserScan,
            "/model/mdp_car/sensors/ultrasonic",
            self._on_us_scan,
            10,
            callback_group=callback_group,
        )

        self._ir_l_scan_sub = self.create_subscription(
            LaserScan,
            "/model/mdp_car/sensors/ir_left",
            self._on_ir_left_scan,
            10,
            callback_group=callback_group,
        )

        self._ir_r_scan_sub = self.create_subscription(
            LaserScan,
            "/model/mdp_car/sensors/ir_right",
            self._on_ir_right_scan,
            10,
            callback_group=callback_group,
        )

        # Android commands (e.g. RESET)
        self._cmd_sub = self.create_subscription(
            String,
            "/android/cmd",
            self._on_android_cmd,
            10,
            callback_group=callback_group,
        )

        self._last_us_dist = 3.0
        self._last_ir_l_dist = 0.80
        self._last_ir_r_dist = 0.80

        # Heartbeat timer (50 Hz) to ensure fresh telemetry is always published
        # even if vehicle is stationary
        self._latest_pose: Optional[PoseStamped] = None
        self._heartbeat_timer = self.create_timer(
            0.02, self._heartbeat_tick, callback_group=callback_group
        )

        self.get_logger().info("SimBridgeNode initialized and ready.")

    def _handle_maintenance(
        self, request: ExecuteMoves.Request, response: ExecuteMoves.Response
    ) -> ExecuteMoves.Response:
        self.get_logger().info("Simulation /hardware/maintenance request received: acknowledging.")
        response.success = True
        response.status = "FIN"
        return response

    def _on_android_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd == "RESET":
            self.get_logger().info("RESET received: re-aligning odometry to initial pose.")
            self._first_odom_received = False

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self._first_odom_received:
            self._gz_origin_x = pos.x
            self._gz_origin_y = pos.y
            self._gz_origin_yaw = current_yaw
            self._first_odom_received = True

        # Compute displacement in Gazebo local frame and rotate into arena frame
        dx_gz = pos.x - self._gz_origin_x
        dy_gz = pos.y - self._gz_origin_y
        dyaw = current_yaw - self._gz_origin_yaw

        # Note: If spawned at initial_yaw, Gazebo odometry X is body-forward
        # We transform according to initial_yaw
        cos_init = math.cos(self._initial_yaw)
        sin_init = math.sin(self._initial_yaw)

        arena_x = self._initial_x + (dx_gz * cos_init - dy_gz * sin_init)
        arena_y = self._initial_y + (dx_gz * sin_init + dy_gz * cos_init)
        arena_yaw = (self._initial_yaw + dyaw + math.pi) % (2.0 * math.pi) - math.pi

        # Create PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self._odom_frame
        pose_msg.pose.position.x = arena_x
        pose_msg.pose.position.y = arena_y
        pose_msg.pose.position.z = 0.0

        half_yaw = arena_yaw / 2.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_yaw)

        self._latest_pose = pose_msg

        # Publish pose
        self._pose_pub.publish(pose_msg)
        self._raw_pose_pub.publish(pose_msg)

        # Broadcast TF
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = pose_msg.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = arena_x
            tf.transform.translation.y = arena_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = pose_msg.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    def _extract_min_range(self, scan: LaserScan, default_val: float) -> float:
        valid_ranges = [
            r for r in scan.ranges
            if not math.isnan(r) and not math.isinf(r) and scan.range_min <= r <= scan.range_max
        ]
        return min(valid_ranges) if valid_ranges else default_val

    def _on_us_scan(self, msg: LaserScan) -> None:
        dist = self._extract_min_range(msg, 3.0)
        self._last_us_dist = dist
        self._publish_range(dist, "/sensors/ultrasonic", 0.02, 3.00, 0.26, self._us_pub, self._raw_us_pub)

    def _on_ir_left_scan(self, msg: LaserScan) -> None:
        dist = self._extract_min_range(msg, 0.80)
        self._last_ir_l_dist = dist
        self._publish_range(dist, "/sensors/ir_left", 0.10, 0.80, 0.05, self._ir_left_pub, self._raw_ir_left_pub)

    def _on_ir_right_scan(self, msg: LaserScan) -> None:
        dist = self._extract_min_range(msg, 0.80)
        self._last_ir_r_dist = dist
        self._publish_range(dist, "/sensors/ir_right", 0.10, 0.80, 0.05, self._ir_right_pub, self._raw_ir_right_pub)

    def _publish_range(
        self,
        dist: float,
        frame_id: str,
        min_r: float,
        max_r: float,
        fov: float,
        pub: rclpy.publisher.Publisher,
        raw_pub: rclpy.publisher.Publisher,
    ) -> None:
        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.header.frame_id = frame_id
        rng.radiation_type = Range.ULTRASOUND if "ultrasonic" in frame_id else Range.INFRARED
        rng.field_of_view = fov
        rng.min_range = min_r
        rng.max_range = max_r
        rng.range = float(dist)
        pub.publish(rng)
        raw_pub.publish(rng)

    def _heartbeat_tick(self) -> None:
        """Publishes fresh stamped copies of the latest telemetry at 50 Hz."""
        now = self.get_clock().now().to_msg()
        if self._latest_pose is not None:
            self._latest_pose.header.stamp = now
            self._raw_pose_pub.publish(self._latest_pose)
            if self._publish_tf:
                tf = TransformStamped()
                tf.header.stamp = now
                tf.header.frame_id = self._odom_frame
                tf.child_frame_id = self._base_frame
                tf.transform.translation.x = self._latest_pose.pose.position.x
                tf.transform.translation.y = self._latest_pose.pose.position.y
                tf.transform.translation.z = 0.0
                tf.transform.rotation = self._latest_pose.pose.orientation
                self._tf_broadcaster.sendTransform(tf)

        # Heartbeat for ranges
        self._publish_range(self._last_us_dist, "ultrasonic_link", 0.02, 3.00, 0.26, self._us_pub, self._raw_us_pub)
        self._publish_range(self._last_ir_l_dist, "ir_left_link", 0.10, 0.80, 0.05, self._ir_left_pub, self._raw_ir_left_pub)
        self._publish_range(self._last_ir_r_dist, "ir_right_link", 0.10, 0.80, 0.05, self._ir_right_pub, self._raw_ir_right_pub)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
