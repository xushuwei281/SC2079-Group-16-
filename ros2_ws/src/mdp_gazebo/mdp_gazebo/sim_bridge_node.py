"""Gazebo stand-in for serial_bridge_node's ROS-facing contract.

serial_bridge_node (mdp_hardware_bridge) is the only thing on real hardware
that publishes /robot_pose(/raw), broadcasts the odom->base_link TF, and
publishes the ultrasonic/IR Range topics motion_controller_node's safety
interlock reads. Nothing else in the stack — motion_controller_node,
planner_node, RViz/Foxglove layouts — talks to the STM32 directly; they all
consume that contract. This node reproduces exactly that contract, sourced
from Gazebo's DiffDrive plugin odometry (bridged in as nav_msgs/Odometry on
/sim/odom) instead of UART telemetry, so the rest of the stack runs
unmodified against the simulated robot.

The Range topics are a stub, not a simulated sensor: they always report
"clear" (max_range) on a fresh timestamp. Nothing in this package models the
ultrasonic/IR beams themselves — this stub exists only so
motion_controller_node's SENSOR_STALE / PROXIMITY interlock (which requires
recent Range data before it will command any forward motion) doesn't block
movement in sim. Swap it for a real simulated sensor plugin if you need
obstacle-triggered stopping in sim.

gz-sim's DiffDrive plugin zeroes its odometry at wherever the model was
spawned, not at the world origin — so its raw output is spawn-relative, not
the arena-frame pose planner_node expects (planner_node._on_pose multiplies
/robot_pose's position directly by 100 to get arena cm, exactly like the
real robot's calibrated-at-boot pose already is). The spawn_x/spawn_y/
spawn_yaw parameters (set by gazebo.launch.py to match the same values it
passes to `ros_gz_sim create`) correct for that by re-applying the spawn
pose as a fixed offset before publishing.
"""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster


class SimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_bridge_node")
        for name, default in (("odom_frame_id", "odom"), ("base_frame_id", "base_link"),
                              ("publish_tf", True), ("sensor_publish_rate_hz", 20.0),
                              ("spawn_x", 0.0), ("spawn_y", 0.0), ("spawn_yaw", 0.0)):
            self.declare_parameter(name, default)
        self._odom_frame_id = self.get_parameter("odom_frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        rate = float(self.get_parameter("sensor_publish_rate_hz").value)
        self._spawn_x = float(self.get_parameter("spawn_x").value)
        self._spawn_y = float(self.get_parameter("spawn_y").value)
        self._spawn_yaw = float(self.get_parameter("spawn_yaw").value)
        self._cos_spawn_yaw = math.cos(self._spawn_yaw)
        self._sin_spawn_yaw = math.sin(self._spawn_yaw)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self._raw_pose_pub = self.create_publisher(PoseStamped, "/robot_pose/raw", 1)
        self._us_pub = self.create_publisher(Range, "/sensors/ultrasonic/raw", 10)
        self._ir_left_pub = self.create_publisher(Range, "/sensors/ir_left/raw", 10)
        self._ir_right_pub = self.create_publisher(Range, "/sensors/ir_right/raw", 10)

        self._odom_sub = self.create_subscription(Odometry, "/sim/odom", self._on_odom, 10)
        self._sensor_timer = self.create_timer(1.0 / rate, self._publish_clear_ranges)

    def _on_odom(self, msg: Odometry) -> None:
        now = self.get_clock().now().to_msg()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Re-apply the spawn pose the plugin zeroed out, so this reports the
        # same arena-frame pose the real robot's calibrated-at-boot pose does.
        x = self._spawn_x + self._cos_spawn_yaw * p.x - self._sin_spawn_yaw * p.y
        y = self._spawn_y + self._sin_spawn_yaw * p.x + self._cos_spawn_yaw * p.y
        yaw = math.atan2(math.sin(yaw + self._spawn_yaw), math.cos(yaw + self._spawn_yaw))
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self._odom_frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_pub.publish(pose_msg)
        self._raw_pose_pub.publish(pose_msg)

        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self._odom_frame_id
            tf_msg.child_frame_id = self._base_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_clear_ranges(self) -> None:
        now = self.get_clock().now().to_msg()

        us = Range()
        us.header.stamp, us.header.frame_id = now, "ultrasonic_link"
        us.radiation_type, us.field_of_view = Range.ULTRASOUND, 0.26
        us.min_range, us.max_range = 0.02, 3.00
        us.range = us.max_range
        self._us_pub.publish(us)

        for pub, frame_id in ((self._ir_left_pub, "ir_left_link"),
                              (self._ir_right_pub, "ir_right_link")):
            ir = Range()
            ir.header.stamp, ir.header.frame_id = now, frame_id
            ir.radiation_type, ir.field_of_view = Range.INFRARED, 0.10
            ir.min_range, ir.max_range = 0.10, 0.80
            ir.range = ir.max_range
            pub.publish(ir)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
