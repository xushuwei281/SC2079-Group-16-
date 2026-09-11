"""Continuous velocity UART driver with stationary firmware maintenance.

The sole serial reader runs independently of movement service callbacks. Motion
is streamed as five-byte V packets; FIN is used only for stationary maintenance.
"""

import math
import os
import struct
import termios
import threading
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import serial.tools.list_ports
from std_msgs.msg import Empty, String
import tf2_ros

from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves
from mdp_hardware_bridge.kalman_filter import KalmanFilter1D, PoseKalmanFilter

# Confirmed valid 2-char command codes from the firmware's comm_task
# switch statement (stm32/Core/Src/main.c).
_VALID_COMMANDS = {"FC", "BC", "FL", "FR", "BL", "BR", "FU", "BU", "GC", "G0", "TO"}

# Firmware queue capacity: instrList[40][5] (stm32/src/main.c). The '#'
# trigger is NOT stored by the firmware, so all 40 slots take real moves.
_MAX_BATCH = 40

# Retries when the firmware answers BUS (a previous batch was still
# executing or in mechanical settling when our trigger landed).
_BUS_RETRIES = 10

# Standard candidate device nodes for STM32 USB-CDC / UART
_DEFAULT_CANDIDATE_PORTS = [
    "/dev/ttySTM32",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]


class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("serial_bridge_node")

        # Configured serial port: explicit path (e.g. /dev/ttyACM1, /dev/ttySTM32)
        # or 'auto' to auto-detect from available devices.
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate", 115200)
        # How long to wait for FIN/BUS/FUL after triggering a batch.
        # Move batches are physically blocking on the STM32 side (real
        # acceleration/deceleration ramps), so this needs real headroom --
        # start generous and tighten once you know real maneuver timing.
        self.declare_parameter("batch_timeout_sec", 30.0)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("auto_reconnect", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("initial_x", 0.20)
        self.declare_parameter("initial_y", 0.20)
        self.declare_parameter("initial_yaw", math.pi / 2.0)
        self.declare_parameter("enable_kalman_filter", True)
        self.declare_parameter("enable_sensor_kalman", True)
        self.declare_parameter("enable_pose_kalman", True)
        self.declare_parameter("sensor_kalman_q", 1e-3)
        self.declare_parameter("sensor_kalman_r", 1e-2)
        self.declare_parameter("pose_kalman_q_pos", 5e-4)
        self.declare_parameter("pose_kalman_r_pos", 2e-3)
        self.declare_parameter("pose_kalman_q_yaw", 5e-4)
        self.declare_parameter("pose_kalman_r_yaw", 5e-3)

        self.declare_parameter("velocity_speed_mps", 0.15)
        self.declare_parameter("velocity_max_speed_mps", 0.30)
        self.declare_parameter("velocity_max_yaw_rps", 1.20)
        self.declare_parameter("velocity_turn_radius_m", 0.21)
        self.declare_parameter("cmd_vel_timeout_sec", 0.20)
        self.declare_parameter("telemetry_timeout_sec", 0.40)
        self._speed = float(self.get_parameter("velocity_speed_mps").value)
        self._max_speed = float(self.get_parameter("velocity_max_speed_mps").value)
        self._max_yaw = float(self.get_parameter("velocity_max_yaw_rps").value)
        self._radius = float(self.get_parameter("velocity_turn_radius_m").value)
        self._cmd_timeout = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self._telemetry_timeout = float(self.get_parameter("telemetry_timeout_sec").value)
        if not (0 < self._speed <= self._max_speed <= 0.30 and self._radius >= 0.21
                and 0 < self._max_yaw <= 1.20 and 0 < self._cmd_timeout <= 0.20
                and 0 < self._telemetry_timeout <= 0.40):
            raise ValueError("Unsafe continuous velocity configuration")

        self._configured_port = self.get_parameter("serial_port").value
        self._baud = self.get_parameter("baud_rate").value
        self._batch_timeout_sec = self.get_parameter("batch_timeout_sec").value
        self._auto_reconnect = self.get_parameter("auto_reconnect").value
        reconnect_interval = self.get_parameter("reconnect_interval_sec").value
        self._publish_tf = self.get_parameter("publish_tf").value
        self._odom_frame_id = self.get_parameter("odom_frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._initial_x = float(self.get_parameter("initial_x").value)
        self._initial_y = float(self.get_parameter("initial_y").value)
        self._initial_yaw = float(self.get_parameter("initial_yaw").value)

        # Kalman filtering for telemetry
        self._enable_kalman = bool(self.get_parameter("enable_kalman_filter").value)
        self._enable_sensor_kalman = (
            bool(self.get_parameter("enable_sensor_kalman").value) and self._enable_kalman
        )
        self._enable_pose_kalman = (
            bool(self.get_parameter("enable_pose_kalman").value) and self._enable_kalman
        )

        sq = float(self.get_parameter("sensor_kalman_q").value)
        sr = float(self.get_parameter("sensor_kalman_r").value)
        self._us_filter = KalmanFilter1D(
            q=sq, r=sr, outlier_threshold=0.30, min_val=0.02, max_val=3.00
        )
        self._ir_left_filter = KalmanFilter1D(
            q=sq, r=sr, outlier_threshold=0.25, min_val=0.10, max_val=0.80
        )
        self._ir_right_filter = KalmanFilter1D(
            q=sq, r=sr, outlier_threshold=0.25, min_val=0.10, max_val=0.80
        )

        pq_pos = float(self.get_parameter("pose_kalman_q_pos").value)
        pr_pos = float(self.get_parameter("pose_kalman_r_pos").value)
        pq_yaw = float(self.get_parameter("pose_kalman_q_yaw").value)
        pr_yaw = float(self.get_parameter("pose_kalman_r_yaw").value)
        self._pose_filter = PoseKalmanFilter(
            q_pos=pq_pos,
            r_pos=pr_pos,
            q_yaw=pq_yaw,
            r_yaw=pr_yaw,
            outlier_dist_threshold=0.60,
        )

        # Robot dead-reckoning state (in meters and radians, start zone center by default)
        self._x: float = self._initial_x
        self._y: float = self._initial_y
        self._yaw: float = self._initial_yaw

        self._serial: Optional[serial.Serial] = None
        self._active_port: Optional[str] = None
        self._last_warn_time: float = 0.0

        # Guards raw writes only -- never held across a blocking read, so
        # e-stop can always preempt a batch that's mid-execution.
        self._write_lock = threading.Lock()
        self._busy = threading.Event()
        self._estop_event = threading.Event()

        self._control_lock = threading.RLock()
        self._read_lock = threading.Lock()
        self._rx_buffer = b""
        self._generation = 0
        self._teleop_target = None
        self._teleop_stamp = 0.0
        self._raw_pose = None
        self._raw_us = float("nan")
        self._telemetry_stamp = 0.0
        self._status_seq = 0
        self._last_status = ""
        self._stop_reason = "ESTOPPED"

        # The service call blocks for a long time (real motion), so it
        # needs its own thread -- otherwise a running call would starve
        # the e-stop subscription's callback from ever running.
        callback_group = ReentrantCallbackGroup()

        self._pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self._raw_pose_pub = self.create_publisher(PoseStamped, "/robot_pose/raw", 1)
        self._raw_us_pub = self.create_publisher(Range, "/sensors/ultrasonic/raw", 1)
        self._us_pub = self.create_publisher(Range, "/sensors/ultrasonic", 10)
        self._ir_left_pub = self.create_publisher(Range, "/sensors/ir_left", 10)
        self._ir_right_pub = self.create_publisher(Range, "/sensors/ir_right", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._estop_pub = self.create_publisher(Empty, "/estop", 10)
        self._velocity_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 1, callback_group=callback_group
        )
        self._velocity_timer = self.create_timer(
            0.05, self._velocity_tick, callback_group=callback_group
        )

        self._estop_sub = self.create_subscription(
            Empty, "estop", self._on_estop, 10, callback_group=callback_group
        )
        self._cmd_sub = self.create_subscription(
            String, "/android/cmd", self._on_android_cmd, 10, callback_group=callback_group
        )
        self._service = self.create_service(
            ExecuteMoves,
            "/hardware/maintenance",
            self._handle_maintenance,
            callback_group=callback_group,
        )

        # Attempt initial connection
        self._try_connect()

        # Background periodic reconnection check
        if self._auto_reconnect:
            self._timer = self.create_timer(
                reconnect_interval, self._check_connection, callback_group=callback_group
            )
        else:
            self._timer = None

        # Background periodic telemetry polling (when idle)
        self._telemetry_timer = self.create_timer(
            0.05, self._poll_telemetry, callback_group=callback_group
        )

    def _find_candidate_ports(self) -> list[str]:
        candidates: list[str] = []
        if self._configured_port and self._configured_port.lower() != "auto":
            candidates.append(self._configured_port)

        for dev in _DEFAULT_CANDIDATE_PORTS:
            if dev not in candidates:
                candidates.append(dev)

        try:
            for port_info in serial.tools.list_ports.comports():
                dev = port_info.device
                if dev not in candidates:
                    candidates.append(dev)
        except Exception:
            pass

        return candidates

    def _try_connect(self) -> bool:
        with self._write_lock:
            if self._serial is not None and self._serial.is_open:
                return True

            self._close_serial_locked()

            candidates = self._find_candidate_ports()
            for port in candidates:
                if not os.path.exists(port):
                    continue
                try:
                    s = serial.Serial(port, self._baud, timeout=0, write_timeout=0.05)
                    self._rx_buffer = b""
                    self._telemetry_stamp = 0.0
                    self._teleop_target = None
                    self._serial = s
                    self._active_port = port
                    self.get_logger().info(f"Connected to STM32 on {port} @ {self._baud}")
                    return True
                except (serial.SerialException, OSError, termios.error) as exc:
                    self.get_logger().debug(f"Failed to open {port}: {exc}")

            now = time.monotonic()
            if now - self._last_warn_time > 5.0:
                self._last_warn_time = now
                if candidates:
                    self.get_logger().warn(
                        f"STM32 not connected. Tried candidates: {candidates}. Will keep retrying."
                    )
                else:
                    self.get_logger().warn(
                        "STM32 not connected (no candidate serial ports found). Will keep retrying."
                    )
            return False

    def _close_serial_locked(self) -> None:
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._active_port = None

    def _check_connection(self) -> None:
        if self._serial is None or not self._serial.is_open:
            self._try_connect()

    def _encode(self, command: str, value: int) -> bytes:
        if command not in _VALID_COMMANDS:
            raise ValueError(f"unknown command code {command!r}")
        if not (0 <= value <= 999):
            # The firmware parses this field as unsigned via atoi() on a
            # fixed 3-digit slice -- direction is encoded in the command
            # code itself (FL vs FR, FC vs BC), so negative values here
            # don't map to anything the firmware expects.
            raise ValueError(f"value {value} out of range 0-999")
        return f"{command}{value:03d}".encode("ascii")

    def _write_packet(self, packet: bytes) -> bool:
        """Serialize bounded writes without ever holding the UART reader lock."""
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            try:
                if self._serial.write(packet) != len(packet):
                    raise serial.SerialTimeoutException("Partial velocity packet write")
                return True
            except (serial.SerialException, serial.SerialTimeoutException,
                    OSError, termios.error) as exc:
                self.get_logger().error(f"Serial write error: {exc}")
                self._close_serial_locked()
                return False

    def _velocity_packet(self, speed: float, yaw: float) -> bytes:
        if not (math.isfinite(speed) and math.isfinite(yaw)):
            raise ValueError("Velocity must be finite")
        if abs(speed) > self._max_speed or abs(yaw) > self._max_yaw:
            raise ValueError("Velocity exceeds configured limits")
        if abs(yaw) > abs(speed) / self._radius + 1e-9:
            raise ValueError("Ackermann curvature limit exceeded")
        return b"V" + struct.pack("<hh", round(speed * 1000), round(yaw * 1000))

    def _latch_stop(self, reason: str) -> None:
        with self._control_lock:
            was_stopped = self._estop_event.is_set()
            self._stop_reason = reason
            self._estop_event.set()
            self._generation += 1
            self._teleop_target = None
            self._write_packet(b"Q\x00\x00\x00\x00")
            if not was_stopped:
                self.get_logger().warn(f"Motion stopped: {reason}")
                self._estop_pub.publish(Empty())

    def _on_estop(self, _msg: Empty) -> None:
        if not self._estop_event.is_set():
            self._latch_stop("ESTOPPED")

    def _on_android_cmd(self, msg: String) -> None:
        if msg.data.strip().upper() not in ("RESET", "ALG:RESET"):
            return
        with self._control_lock:
            # A reset cancels the old owner; its finally block releases ownership.
            self._generation += 1
            self._teleop_target = None
            if not self._write_packet(b"R\x00\x00\x00\x00"):
                self._estop_event.set()
                return
            self._estop_event.clear()
            self._stop_reason = "ESTOPPED"
            self._telemetry_stamp = 0.0
            self._raw_pose = None

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._control_lock:
            if self._busy.is_set() or self._estop_event.is_set():
                return
            try:
                if any(v != 0.0 for v in (msg.linear.y, msg.linear.z,
                                         msg.angular.x, msg.angular.y)):
                    raise ValueError("Only linear.x and angular.z are supported")
                target = self._velocity_packet(msg.linear.x, msg.angular.z)
            except ValueError as exc:
                self._teleop_target = None
                self._write_packet(self._velocity_packet(0.0, 0.0))
                self.get_logger().warn(f"Rejected cmd_vel: {exc}", throttle_duration_sec=1.0)
                return
            if msg.linear.x == 0.0 and msg.angular.z == 0.0:
                self._teleop_target = None
                self._write_packet(target)
                return
            self._teleop_target = target
            self._teleop_stamp = time.monotonic()
            self._velocity_tick()

    def _velocity_tick(self) -> None:
        with self._control_lock:
            if self._busy.is_set() or self._estop_event.is_set():
                return
            if self._teleop_target is None:
                return
            if time.monotonic() - self._teleop_stamp >= self._cmd_timeout:
                self._teleop_target = None
                self._write_packet(self._velocity_packet(0.0, 0.0))
                return
            if not self._telemetry_fresh():
                self._latch_stop("STALE_TELEMETRY")
                return
            if not self._write_packet(self._teleop_target):
                self._latch_stop("DISCONNECTED")

    def _telemetry_fresh(self) -> bool:
        return (self._raw_pose is not None and self._telemetry_stamp > 0
                and time.monotonic() - self._telemetry_stamp < self._telemetry_timeout)

    def _handle_maintenance(self, request: ExecuteMoves.Request,
                            response: ExecuteMoves.Response) -> ExecuteMoves.Response:
        if not request.commands or any(mc.command not in {"GC", "G0", "TO"}
                                       for mc in request.commands):
            response.success, response.status = False, "INVALID_MAINTENANCE"
            return response
        try:
            for mc in request.commands:
                self._encode(mc.command, mc.value)
        except ValueError:
            response.success, response.status = False, "INVALID_MAINTENANCE"
            return response
        with self._control_lock:
            if self._estop_event.is_set() or self._busy.is_set():
                response.success, response.status = False, "ESTOPPED_OR_BUSY"
                return response
            if not self._try_connect():
                response.success, response.status = False, "DISCONNECTED"
                return response
            self._busy.set()
            self._teleop_target = None
            generation = self._generation
        try:
            status = "FIN"
            for mc in request.commands:
                status = self._execute_maintenance(mc, generation)
                if status != "FIN":
                    break
            response.success, response.status = status == "FIN", status
        finally:
            with self._control_lock:
                self._write_packet(self._velocity_packet(0.0, 0.0))
                self._busy.clear()
        return response

    def _execute_maintenance(self, mc: MoveCommand, generation: int) -> str:
        with self._control_lock:
            if generation != self._generation or self._estop_event.is_set():
                return "CANCELLED"
            sequence = self._status_seq
            for packet in (self._velocity_packet(0.0, 0.0), self._encode(mc.command, mc.value),
                           b"#\x00\x00\x00\x00"):
                if not self._write_packet(packet):
                    self._latch_stop("DISCONNECTED")
                    return "DISCONNECTED"
        deadline = time.monotonic() + self._batch_timeout_sec
        while time.monotonic() < deadline:
            with self._control_lock:
                if generation != self._generation or self._estop_event.is_set():
                    return self._stop_reason if self._estop_event.is_set() else "CANCELLED"
                if self._status_seq > sequence:
                    sequence = self._status_seq
                    if self._last_status.startswith("FIN"):
                        self._telemetry_stamp = 0.0
                        return "FIN"
                    if self._last_status != "RUN":
                        return self._last_status
            time.sleep(0.05)
        self._latch_stop("TIMEOUT")
        return "TIMEOUT"

    def _publish_sensor_ranges(self, us_m: float, ir1_m: float, ir2_m: float) -> None:
        """Publish sensor_msgs/Range messages for Ultrasonic and IR sensors."""
        now = self.get_clock().now().to_msg()

        us = Range()
        us.header.stamp = now
        us.header.frame_id = "ultrasonic_link"
        us.radiation_type = Range.ULTRASOUND
        us.field_of_view = 0.26  # ~15 degrees cone
        us.min_range = 0.02
        us.max_range = 3.00
        us.range = float(us_m) if us_m > 0.0 else float("inf")
        self._us_pub.publish(us)

        r1 = Range()
        r1.header.stamp = now
        r1.header.frame_id = "ir_left_link"
        r1.radiation_type = Range.INFRARED
        r1.field_of_view = 0.1
        r1.min_range = 0.10
        r1.max_range = 0.80
        r1.range = float(ir1_m) if ir1_m > 0.0 else float("inf")
        self._ir_left_pub.publish(r1)

        r2 = Range()
        r2.header.stamp = now
        r2.header.frame_id = "ir_right_link"
        r2.radiation_type = Range.INFRARED
        r2.field_of_view = 0.1
        r2.min_range = 0.10
        r2.max_range = 0.80
        r2.range = float(ir2_m) if ir2_m > 0.0 else float("inf")
        self._ir_right_pub.publish(r2)

        self.get_logger().debug(
            f"Sensors: US={us_m*100.0:.1f}cm, IR1={ir1_m*100.0:.1f}cm, IR2={ir2_m*100.0:.1f}cm"
        )

    def _publish_current_pose(self) -> None:
        """Publish PoseStamped and broadcast TF transform for the current pose."""
        now = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self._odom_frame_id
        pose_msg.pose.position.x = float(self._x)
        pose_msg.pose.position.y = float(self._y)
        pose_msg.pose.position.z = 0.0

        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self._pose_pub.publish(pose_msg)

        # Broadcast TF odom -> base_link
        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self._odom_frame_id
            tf_msg.child_frame_id = self._base_frame_id
            tf_msg.transform.translation.x = float(self._x)
            tf_msg.transform.translation.y = float(self._y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self._tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().debug(
            f"Pose updated: x={self._x:.3f}m, y={self._y:.3f}m, yaw={math.degrees(self._yaw):.1f}°"
        )

    def _handle_telemetry_line(self, line: str) -> None:
        """Keep raw feedback for control; filtering is only for published displays."""
        try:
            parts = line[4:].strip().split(",")
            if len(parts) != 6:
                raise ValueError("Expected six telemetry fields")
            x_cm, y_cm, world_deg, us_cm, ir1_cm, ir2_cm = map(float, parts)
            if not all(math.isfinite(v) for v in (x_cm, y_cm, world_deg,
                                                  us_cm, ir1_cm, ir2_cm)):
                raise ValueError("Non-finite telemetry")
            raw_x, raw_y = x_cm / 100.0, y_cm / 100.0
            # Preserve the measured STM32 CW-positive to ROS CCW-positive convention.
            raw_yaw = math.radians(180.0 - world_deg)
            raw_yaw = math.atan2(math.sin(raw_yaw), math.cos(raw_yaw))
            with self._control_lock:
                self._raw_pose = (raw_x, raw_y, raw_yaw)
                self._raw_us = us_cm / 100.0
                self._telemetry_stamp = time.monotonic()
                if self._enable_pose_kalman:
                    self._x, self._y, self._yaw = self._pose_filter.update(
                        raw_x, raw_y, raw_yaw)
                else:
                    self._x, self._y, self._yaw = self._raw_pose
            raw_pose = PoseStamped()
            raw_pose.header.stamp = self.get_clock().now().to_msg()
            raw_pose.header.frame_id = self._odom_frame_id
            raw_pose.pose.position.x, raw_pose.pose.position.y = raw_x, raw_y
            raw_pose.pose.orientation.z = math.sin(raw_yaw / 2)
            raw_pose.pose.orientation.w = math.cos(raw_yaw / 2)
            self._raw_pose_pub.publish(raw_pose)
            raw_range = Range()
            raw_range.header = raw_pose.header
            raw_range.header.frame_id = "ultrasonic_link"
            raw_range.range = self._raw_us
            raw_range.radiation_type = Range.ULTRASOUND
            raw_range.min_range, raw_range.max_range = 0.02, 3.0
            raw_range.field_of_view = 0.26
            self._raw_us_pub.publish(raw_range)
            self._publish_current_pose()
            ranges = (us_cm / 100.0, ir1_cm / 100.0, ir2_cm / 100.0)
            if self._enable_sensor_kalman:
                ranges = tuple(f.update(v) for f, v in zip(
                    (self._us_filter, self._ir_left_filter, self._ir_right_filter), ranges))
            self._publish_sensor_ranges(*ranges)
        except (ValueError, TypeError) as exc:
            self.get_logger().debug(f"Invalid telemetry: {exc}")

    def _poll_telemetry(self) -> None:
        """Single nonblocking reader, active during both service and teleoperation."""
        if not self._read_lock.acquire(blocking=False):
            return
        try:
            port = self._serial
            if port is None or not port.is_open:
                return
            waiting = port.in_waiting
            if not isinstance(waiting, int) or waiting <= 0:
                return
            chunk = port.read(min(waiting, 4096))
            self._rx_buffer += chunk
            if len(self._rx_buffer) > 8192:
                self._rx_buffer = b""
                self._latch_stop("INVALID_TELEMETRY")
                return
            while b"\n" in self._rx_buffer:
                raw, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("TLM:"):
                    self._handle_telemetry_line(line)
                elif line.startswith("STOP:"):
                    self._latch_stop(line)
                elif line:
                    with self._control_lock:
                        self._last_status = line
                        self._status_seq += 1
        except (serial.SerialException, serial.SerialTimeoutException,
                OSError, termios.error) as exc:
            self.get_logger().error(f"Serial read error: {exc}")
            self._latch_stop("DISCONNECTED")
            with self._write_lock:
                if self._serial is port:
                    self._close_serial_locked()
        finally:
            self._read_lock.release()

    def destroy_node(self) -> None:
        try:
            self._generation += 1
            self._estop_event.set()
            self._write_packet(b"Q\x00\x00\x00\x00")
            self._velocity_timer.cancel()
            if self._timer is not None:
                self._timer.cancel()
            if self._telemetry_timer is not None:
                self._telemetry_timer.cancel()
            with self._write_lock:
                self._close_serial_locked()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            return super().destroy_node()
        except (Exception, KeyboardInterrupt):
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
