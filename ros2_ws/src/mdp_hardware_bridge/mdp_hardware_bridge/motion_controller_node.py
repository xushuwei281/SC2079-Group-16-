"""Measured primitive controller and teleoperation arbiter publishing ROS Twist.

Only this node owns ExecuteMoves. The UART driver consumes /cmd_vel and never
interprets movement primitives. Autonomous completion and the central forward
safety check both read the same Kalman-filtered /sensors/* topics used for
monitoring/display, so there is a single sensor pipeline (with its own
outlier gating, see kalman_filter.py) rather than a second raw path that can
drift out of sync with it.
"""
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves

_VALID_COMMANDS = {"FC", "BC", "FL", "FR", "BL", "BR", "FU", "BU", "GC", "G0", "TO"}
_MAX_BATCH = 40
_CURVATURE_EPSILON_RPS = 0.001
_LATEST_VALUE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MotionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_controller_node")
        for name, default in (("velocity_speed_mps", 0.15),
                              ("velocity_max_speed_mps", 0.35),
                              ("velocity_turn_radius_m", 0.21),
                              ("velocity_max_yaw_rps", 1.75), ("batch_timeout_sec", 30.0),
                              ("telemetry_timeout_sec", 0.4), ("cmd_vel_timeout_sec", 0.2),
                              ("front_stop_distance_m", 0.12),
                              ("ir_stop_distance_m", 0.10)):
            self.declare_parameter(name, default)
        self._speed = float(self.get_parameter("velocity_speed_mps").value)
        self._max_speed = float(self.get_parameter("velocity_max_speed_mps").value)
        self._radius = float(self.get_parameter("velocity_turn_radius_m").value)
        self._max_yaw = float(self.get_parameter("velocity_max_yaw_rps").value)
        self._batch_timeout_sec = float(self.get_parameter("batch_timeout_sec").value)
        self._telemetry_timeout = float(self.get_parameter("telemetry_timeout_sec").value)
        self._cmd_timeout = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self._front_stop = float(self.get_parameter("front_stop_distance_m").value)
        self._ir_stop = float(self.get_parameter("ir_stop_distance_m").value)
        if not (0 < self._speed <= self._max_speed <= 0.35 and self._radius >= 0.21
                and 0 < self._max_yaw <= 1.75 and 0 < self._telemetry_timeout <= 0.4
                and 0 < self._cmd_timeout <= 0.2 and self._batch_timeout_sec > 0
                and self._front_stop >= 0.12 and self._ir_stop >= 0.10):
            raise ValueError("Unsafe motion controller configuration")
        self._control_lock = threading.RLock()
        self._busy = threading.Event()
        self._estop_event = threading.Event()
        self._shutdown_event = threading.Event()
        self._generation = 0
        self._stop_reason = "ESTOPPED"
        self._raw_pose = None
        self._us_range_m = float("nan")
        self._ir_left_range_m = float("nan")
        self._ir_right_range_m = float("nan")
        self._telemetry_stamp = 0.0
        self._range_stamp = 0.0
        self._teleop_target = None
        self._teleop_stamp = 0.0
        group = ReentrantCallbackGroup()
        self._velocity_pub = self.create_publisher(Twist, "/cmd_vel", _LATEST_VALUE_QOS)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose/raw", self._on_pose, _LATEST_VALUE_QOS,
            callback_group=group)
        self._range_sub = self.create_subscription(
            Range, "/sensors/ultrasonic", self._on_range, _LATEST_VALUE_QOS,
            callback_group=group)
        self._ir_left_sub = self.create_subscription(
            Range, "/sensors/ir_left", self._on_ir_left, _LATEST_VALUE_QOS,
            callback_group=group)
        self._ir_right_sub = self.create_subscription(
            Range, "/sensors/ir_right", self._on_ir_right, _LATEST_VALUE_QOS,
            callback_group=group)
        self._estop_sub = self.create_subscription(
            Empty, "/estop", self._on_estop, 10, callback_group=group)
        self._reset_sub = self.create_subscription(
            String, "/android/cmd", self._on_android_cmd, 10, callback_group=group)
        self._teleop_sub = self.create_subscription(
            Twist, "/cmd_vel/teleop", self._on_teleop, _LATEST_VALUE_QOS,
            callback_group=group)
        self._service = self.create_service(
            ExecuteMoves, "/execute_moves", self._handle_execute_moves, callback_group=group)
        self._maintenance_client = self.create_client(
            ExecuteMoves, "/hardware/maintenance", callback_group=group)
        self._timer = self.create_timer(0.05, self._teleop_tick, callback_group=group)

    def _validate_command(self, mc: MoveCommand) -> None:
        if mc.command not in _VALID_COMMANDS or not 0 <= mc.value <= 999:
            raise ValueError("Unknown primitive or value outside 0..999")

    def _recent_header(self, msg: object) -> bool:
        stamp = msg.header.stamp
        age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        return 0 <= age < self._telemetry_timeout

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        if not self._recent_header(msg) or not all(
                math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        with self._control_lock:
            self._raw_pose = (p.x, p.y, yaw)
            self._telemetry_stamp = time.monotonic()

    def _on_range(self, msg: Range) -> None:
        if not self._recent_header(msg):
            return
        with self._control_lock:
            self._us_range_m = msg.range
            self._range_stamp = time.monotonic()

    def _on_ir_left(self, msg: Range) -> None:
        if self._recent_header(msg):
            with self._control_lock:
                self._ir_left_range_m = msg.range
                self._range_stamp = time.monotonic()

    def _on_ir_right(self, msg: Range) -> None:
        if self._recent_header(msg):
            with self._control_lock:
                self._ir_right_range_m = msg.range
                self._range_stamp = time.monotonic()

    def _telemetry_fresh(self) -> bool:
        return (self._raw_pose is not None and self._telemetry_stamp > 0
                and time.monotonic() - self._telemetry_stamp < self._telemetry_timeout)

    def _forward_safety_error(self, forward: bool) -> str:
        """Return a central safety fault for a requested forward velocity.

        Reads the same Kalman-filtered ranges published for monitoring/display
        (see kalman_filter.py) rather than a second raw pipeline, so spike
        rejection is defined in exactly one place.
        """
        if not forward:
            return ""
        if (self._range_stamp <= 0
                or time.monotonic() - self._range_stamp >= self._telemetry_timeout):
            return "SENSOR_STALE"
        for name, value, threshold in (
                ("ULTRASONIC", self._us_range_m, self._front_stop),
                ("IR_LEFT", self._ir_left_range_m, self._ir_stop),
                ("IR_RIGHT", self._ir_right_range_m, self._ir_stop)):
            if math.isfinite(value) and 0 < value <= threshold:
                return f"PROXIMITY:{name}"
        return ""

    def _publish_velocity(self, speed: float, yaw: float) -> None:
        # ROS launch may invalidate the context before node destruction runs.
        # Never publish through an invalid rclpy publisher during teardown.
        if not rclpy.ok():
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = float(speed), float(yaw)
        try:
            self._velocity_pub.publish(msg)
        except (InvalidHandle, RCLError):
            # A concurrent ROS shutdown can invalidate the publisher between
            # the context check and publish call.
            return

    def _latch_stop(self, reason: str, publish_estop: bool = True) -> None:
        with self._control_lock:
            already_stopped = self._estop_event.is_set()
            self._estop_event.set()
            self._generation += 1
            self._stop_reason = reason
            self._teleop_target = None
            self._publish_velocity(0.0, 0.0)
            if not already_stopped:
                if publish_estop:
                    self._estop_pub.publish(Empty())
                self.get_logger().warn(f"Motion stopped: {reason}")

    def _on_estop(self, _msg: Empty) -> None:
        if not self._estop_event.is_set():
            self._latch_stop("ESTOPPED", publish_estop=False)

    def _on_android_cmd(self, msg: String) -> None:
        if msg.data.strip().upper() not in {"RESET", "ALG:RESET"}:
            return
        with self._control_lock:
            self._generation += 1
            self._estop_event.clear()
            self._teleop_target = None
            self._us_range_m = self._ir_left_range_m = self._ir_right_range_m = float("nan")
            self._raw_pose = None
            self._telemetry_stamp = self._range_stamp = 0.0
            self._publish_velocity(0.0, 0.0)

    def _on_teleop(self, msg: Twist) -> None:
        with self._control_lock:
            if self._busy.is_set() or self._estop_event.is_set():
                return
            speed, yaw = msg.linear.x, msg.angular.z
            if (not math.isfinite(speed) or not math.isfinite(yaw)
                    or abs(speed) > self._max_speed or abs(yaw) > self._max_yaw
                    or abs(yaw) > abs(speed) / self._radius + _CURVATURE_EPSILON_RPS
                    or any(v != 0 for v in (msg.linear.y, msg.linear.z,
                                           msg.angular.x, msg.angular.y))):
                self._teleop_target = None
                self._publish_velocity(0.0, 0.0)
                return
            if speed == 0.0 and yaw == 0.0:
                self._teleop_target = None
                self._publish_velocity(0.0, 0.0)
                return
            self._teleop_target = (speed, yaw)
            self._teleop_stamp = time.monotonic()

    def _teleop_tick(self) -> None:
        with self._control_lock:
            if self._busy.is_set() or self._estop_event.is_set() or self._teleop_target is None:
                return
            if time.monotonic() - self._teleop_stamp >= self._cmd_timeout:
                self._teleop_target = None
                self._publish_velocity(0.0, 0.0)
            elif not self._telemetry_fresh():
                # RESET clears cached feedback in both control layers.  A
                # joystick sample can arrive before the first post-reset pose;
                # keep the vehicle stopped while waiting rather than latching
                # a false E-stop.  Loss after feedback was acquired is still
                # treated as a safety fault.
                if self._raw_pose is None:
                    self._publish_velocity(0.0, 0.0)
                else:
                    self._latch_stop("STALE_TELEMETRY")
            else:
                error = self._forward_safety_error(self._teleop_target[0] > 0)
                if error:
                    self._latch_stop(error)
                else:
                    self._publish_velocity(*self._teleop_target)

    def _handle_execute_moves(self, request: ExecuteMoves.Request,
                              response: ExecuteMoves.Response) -> ExecuteMoves.Response:
        commands = request.commands
        try:
            if len(commands) > _MAX_BATCH:
                raise ValueError("Too many primitives")
            for mc in commands:
                self._validate_command(mc)
                if mc.command in {"FU", "BU"} and mc.value < 12:
                    raise ValueError("Ultrasonic target must be at least 12 cm")
            maintenance = any(mc.command in {"GC", "G0", "TO"} for mc in commands)
            if maintenance and any(mc.command not in {"GC", "G0", "TO"} for mc in commands):
                raise ValueError("Maintenance and movement require separate requests")
        except ValueError as exc:
            response.success, response.status = False, f"INVALID_CMD ({exc})"
            return response
        with self._control_lock:
            if self._estop_event.is_set():
                response.success, response.status = False, self._stop_reason
                return response
            if self._busy.is_set() or self._teleop_target is not None:
                response.success, response.status = False, "BUSY_LOCAL"
                return response
            self._busy.set()
            generation = self._generation
        try:
            status = "FIN"
            for mc in commands:
                status = (self._execute_maintenance(mc, generation) if maintenance
                          else self._execute_primitive(mc, generation))
                if status != "FIN":
                    break
            response.success, response.status = status == "FIN", status
        finally:
            with self._control_lock:
                self._publish_velocity(0.0, 0.0)
                self._busy.clear()
        return response

    def _motion_error(self, generation: int, deadline: float) -> str:
        if self._shutdown_event.is_set() or not rclpy.ok():
            return "SHUTDOWN"
        if generation != self._generation:
            return self._stop_reason if self._estop_event.is_set() else "CANCELLED"
        if self._estop_event.is_set():
            return self._stop_reason
        if time.monotonic() >= deadline:
            self._latch_stop("TIMEOUT")
            return "TIMEOUT"
        if not self._telemetry_fresh():
            self._latch_stop("STALE_TELEMETRY")
            return "STALE_TELEMETRY"
        return ""

    def _execute_primitive(self, mc: MoveCommand, generation: int) -> str:
        deadline = time.monotonic() + self._batch_timeout_sec
        with self._control_lock:
            error = self._motion_error(generation, deadline)
            if error:
                return error
            start = self._raw_pose
        previous_yaw = start[2]
        accumulated_yaw = 0.0
        direction = 1.0 if mc.command.startswith("F") else -1.0
        turning = mc.command[1] in {"L", "R"}
        turn_sign = direction * (1.0 if mc.command[1] == "L" else -1.0)
        goal = math.radians(mc.value) if turning else mc.value / 100.0
        while True:
            with self._control_lock:
                error = self._motion_error(generation, deadline)
                if error:
                    return error
                x, y, yaw = self._raw_pose
                accumulated_yaw += math.atan2(math.sin(yaw - previous_yaw),
                                              math.cos(yaw - previous_yaw))
                previous_yaw = yaw
                if turning:
                    remaining = goal - turn_sign * accumulated_yaw
                    complete = remaining <= min(math.radians(1.0), goal * 0.1)
                elif mc.command in {"FU", "BU"}:
                    if (time.monotonic() - self._range_stamp >= self._telemetry_timeout
                            or not math.isfinite(self._us_range_m) or self._us_range_m <= 0):
                        self._latch_stop("INVALID_RANGE")
                        return "INVALID_RANGE"
                    remaining = (self._us_range_m - goal) * direction
                    complete = remaining <= 0
                else:
                    progress = direction * ((x - start[0]) * math.cos(start[2])
                                            + (y - start[1]) * math.sin(start[2]))
                    remaining = goal - progress
                    complete = remaining <= min(0.005, goal * 0.1)
                if complete:
                    self._publish_velocity(0.0, 0.0)
                    return "FIN"
                error = self._forward_safety_error(direction > 0)
                if error:
                    self._latch_stop(error)
                    return error
                # Slow near the measured goal without claiming time-based progress.
                distance_left = remaining * self._radius if turning else remaining
                speed = direction * min(self._speed, max(0.04, distance_left * 1.5))
                yaw_rate = turn_sign * abs(speed) / self._radius if turning else 0.0
                yaw_rate = max(-self._max_yaw, min(self._max_yaw, yaw_rate))
                self._publish_velocity(speed, yaw_rate)
            time.sleep(0.05)

    def _execute_maintenance(self, mc: MoveCommand, generation: int) -> str:
        if not self._maintenance_client.service_is_ready():
            return "MAINTENANCE_UNAVAILABLE"
        self._publish_velocity(0.0, 0.0)
        future = self._maintenance_client.call_async(ExecuteMoves.Request(commands=[mc]))
        deadline = time.monotonic() + self._batch_timeout_sec
        while time.monotonic() < deadline:
            with self._control_lock:
                if self._shutdown_event.is_set() or not rclpy.ok():
                    future.cancel()
                    return "SHUTDOWN"
                if generation != self._generation or self._estop_event.is_set():
                    future.cancel()
                    return self._stop_reason if self._estop_event.is_set() else "CANCELLED"
                if future.done():
                    try:
                        result = future.result()
                    except Exception as exc:
                        self.get_logger().error(f"Maintenance service failed: {exc}")
                        self._latch_stop("MAINTENANCE_ERROR")
                        return "MAINTENANCE_ERROR"
                    self._telemetry_stamp = self._range_stamp = 0.0
                    return "FIN" if result.success else result.status
            time.sleep(0.05)
        future.cancel()
        self._latch_stop("TIMEOUT")
        return "TIMEOUT"

    def request_shutdown(self) -> None:
        """Interrupt active motion before the executor is joined."""
        self._shutdown_event.set()
        with self._control_lock:
            self._generation += 1
            self._estop_event.set()
            self._teleop_target = None
            self._publish_velocity(0.0, 0.0)

    def destroy_node(self) -> None:
        self.request_shutdown()
        self._timer.cancel()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotionControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_shutdown()
        node.destroy_node()
        executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()
