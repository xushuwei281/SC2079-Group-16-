"""UART bridge to the STM32 motor controller.

Speaks the STM32's actual protocol directly (confirmed by reading the
ported STM_Ref firmware's comm_task dispatch, see stm32/src/main.c) --
NOT a continuous ros2_control interface. The firmware executes discrete,
blocking maneuvers (drive N cm, turn N degrees, ...) and gives no
progress signal while one is running, so this is exposed as a plain
blocking ROS2 service rather than an action: an action's two real
advantages (mid-flight cancellation, incremental feedback) don't hold up
here -- the firmware has no graceful "abort this one move" (only the
blunt e-stop, which is handled separately below), and there's no
progress to report between "packets written" (near-instant) and "FIN"
(the only signal that anything physical actually finished).

Packet format: 5 bytes = 2-char command code + 3-digit zero-padded ASCII
value (e.g. b"FC050" = drive forward 50cm). A batch of move packets is
followed by a '#'-prefixed trigger packet; the STM32 replies b"RUN\\r\\n"
when it starts, executes every queued move in sequence, then replies
b"FIN\\r\\n" (or b"BUS\\r\\n" / b"FUL\\r\\n" if rejected). A 'Q'-prefixed
packet triggers immediate e-stop independent of anything queued -- this
node mirrors that by never gating the e-stop write behind the lock used
for a long-running batch send/wait.
"""

import math
import os
import threading
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped, TransformStamped
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
    def __init__(self):
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

        # The service call blocks for a long time (real motion), so it
        # needs its own thread -- otherwise a running call would starve
        # the e-stop subscription's callback from ever running.
        callback_group = ReentrantCallbackGroup()

        self._pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self._us_pub = self.create_publisher(Range, "/sensors/ultrasonic", 10)
        self._ir_left_pub = self.create_publisher(Range, "/sensors/ir_left", 10)
        self._ir_right_pub = self.create_publisher(Range, "/sensors/ir_right", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._estop_sub = self.create_subscription(
            Empty, "estop", self._on_estop, 10, callback_group=callback_group
        )
        self._cmd_sub = self.create_subscription(
            String, "/android/cmd", self._on_android_cmd, 10, callback_group=callback_group
        )
        self._service = self.create_service(
            ExecuteMoves,
            "execute_moves",
            self._handle_execute_moves,
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
                    s = serial.Serial(port, self._baud, timeout=0.5)
                    self._serial = s
                    self._active_port = port
                    self.get_logger().info(f"Connected to STM32 on {port} @ {self._baud}")
                    return True
                except (serial.SerialException, OSError) as exc:
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

    def _on_estop(self, _msg: Empty) -> None:
        self.get_logger().warn("E-STOP requested: Halting motors and aborting active batch")
        self._estop_event.set()
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                self.get_logger().error("E-STOP requested but STM32 is not connected")
                return
            try:
                self._serial.write(b"Q\x00\x00\x00\x00")
                self._serial.flush()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Failed to write E-STOP to STM32: {exc}")
                self._close_serial_locked()

    def _on_android_cmd(self, msg: String) -> None:
        raw = msg.data.strip().upper()
        if raw == "RESET" or raw.startswith("ALG|"):
            self._x = self._initial_x
            self._y = self._initial_y
            self._yaw = self._initial_yaw
            self.get_logger().info(
                f"Pose reset to start zone: x={self._x:.3f}m, y={self._y:.3f}m, yaw={math.degrees(self._yaw):.1f}°"
            )
            self._publish_current_pose()

    def _handle_execute_moves(self, request, response):
        if self._busy.is_set():
            # If an E-stop was active, clear it so new move can proceed
            if self._estop_event.is_set():
                self._busy.clear()
            else:
                response.success = False
                response.status = "BUSY_LOCAL"
                return response

        if len(request.commands) > _MAX_BATCH:
            response.success = False
            response.status = f"TOO_MANY ({len(request.commands)} > {_MAX_BATCH})"
            return response

        # Pre-validate and encode all commands
        try:
            encoded_packets = [self._encode(mc.command, mc.value) for mc in request.commands]
        except ValueError as exc:
            self.get_logger().warn(f"Invalid command in request: {exc}")
            response.success = False
            response.status = f"INVALID_CMD ({exc})"
            return response

        if not self._try_connect():
            response.success = False
            response.status = "DISCONNECTED"
            return response

        self._busy.set()
        self._estop_event.clear()
        try:
            status = ""
            with self._write_lock:
                if self._serial is not None and self._serial.is_open:
                    self._serial.reset_input_buffer()

            for _ in range(_BUS_RETRIES):
                if self._estop_event.is_set():
                    response.success = False
                    response.status = "ESTOPPED"
                    return response

                with self._write_lock:
                    if self._serial is None or not self._serial.is_open:
                        response.success = False
                        response.status = "DISCONNECTED"
                        return response
                    try:
                        for pkt in encoded_packets:
                            self._serial.write(pkt)
                            self._serial.flush()
                            time.sleep(0.015)
                        # Trigger packet -- starts execution of everything just queued.
                        self._serial.write(b"#\x00\x00\x00\x00")
                        self._serial.flush()
                    except (serial.SerialException, OSError) as exc:
                        self.get_logger().error(f"Serial write error: {exc}")
                        self._close_serial_locked()
                        response.success = False
                        response.status = "SERIAL_ERROR"
                        return response

                status = self._read_status_line(deadline_sec=5.0)
                if status == "ESTOP":
                    response.success = False
                    response.status = "ESTOPPED"
                    return response
                if status == "BUS":
                    # Firmware was mid-batch or in mechanical settling;
                    # wait and drain any trailing response before retrying.
                    time.sleep(0.08)
                    with self._write_lock:
                        if self._serial is not None and self._serial.is_open:
                            self._serial.reset_input_buffer()
                    continue
                break

            if status != "RUN":
                response.success = False
                response.status = status or "NO_RESPONSE"
                return response

            status = self._read_status_line(deadline_sec=self._batch_timeout_sec)
            if status == "ESTOP":
                response.success = False
                response.status = "ESTOPPED"
                return response

            response.success = status.startswith("FIN")
            response.status = status or "TIMEOUT"
            if response.success:
                self._update_and_publish_pose(status, request.commands)
        finally:
            self._busy.clear()
            self._estop_event.clear()
        return response

    def _update_and_publish_pose(self, status: str, commands: list[MoveCommand]) -> None:
        """Update dead-reckoning pose and broadcast TF transform.

        If the firmware sent measured feedback ('FIN:<dist_cm>,<heading_deg>'),
        we use the real sensor-fused distance and gyro heading. Otherwise,
        we fall back to nominal kinematics from the commanded list.
        """
        # Only re-initialize pose if explicit GC (calibration at start zone) is commanded
        if any(mc.command == "GC" for mc in commands):
            self._x = self._initial_x
            self._y = self._initial_y
            self._yaw = self._initial_yaw
            self._publish_current_pose()
            return

        if status.startswith("FIN:POS,"):
            payload = status[8:].strip()
            try:
                parts = payload.split(",")
                self._x = float(parts[0]) / 100.0
                self._y = float(parts[1]) / 100.0
                self._yaw = math.radians(float(parts[2]))
                if len(parts) >= 6:
                    us_m = float(parts[3]) / 100.0
                    ir1_m = float(parts[4]) / 100.0
                    ir2_m = float(parts[5]) / 100.0
                    self._publish_sensor_ranges(us_m, ir1_m, ir2_m)
                elif len(parts) >= 5:
                    ir1_m = float(parts[3]) / 100.0
                    ir2_m = float(parts[4]) / 100.0
                    self._publish_sensor_ranges(0.0, ir1_m, ir2_m)
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse POS feedback from {status!r}: {exc}")
                self._accumulate_nominal(commands)
        elif status.startswith("FIN:"):
            payload = status[4:].strip()
            try:
                dist_str, heading_str = payload.split(",")
                dist_m = float(dist_str) / 100.0
                measured_yaw = math.radians(float(heading_str))

                # If the batch contained purely backward moves, distance delta is negative
                is_backward = all(mc.command in {"BC", "BL", "BR", "BU"} for mc in commands)
                if is_backward and dist_m > 0:
                    dist_m = -dist_m

                # STM32 gyro heading is relative to the calibrated G0 baseline (North = initial_yaw)
                world_yaw = self._initial_yaw + measured_yaw
                self._x += dist_m * math.cos(world_yaw)
                self._y += dist_m * math.sin(world_yaw)
                self._yaw = world_yaw
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse sensor feedback from {status!r}: {exc}")
                self._accumulate_nominal(commands)
        else:
            self._accumulate_nominal(commands)

        # Normalize yaw to (-pi, pi]
        self._yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._publish_current_pose()
        self.get_logger().info(
            f"Batch completed pose: x={self._x:.3f}m, y={self._y:.3f}m, yaw={math.degrees(self._yaw):.1f}°"
        )

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

    def _accumulate_nominal(self, commands: list[MoveCommand]) -> None:
        """Nominal kinematic integration when raw sensor feedback is unavailable."""
        for mc in commands:
            cmd = mc.command
            val = float(mc.value)
            if cmd in {"FC", "FU"}:
                d = val / 100.0
                self._x += d * math.cos(self._yaw)
                self._y += d * math.sin(self._yaw)
            elif cmd in {"BC", "BU"}:
                d = val / 100.0
                self._x -= d * math.cos(self._yaw)
                self._y -= d * math.sin(self._yaw)
            elif cmd == "FL":
                self._yaw += math.radians(val)
            elif cmd == "FR":
                self._yaw -= math.radians(val)
            elif cmd == "BL":
                self._yaw -= math.radians(val)
            elif cmd == "BR":
                self._yaw += math.radians(val)

    def _handle_telemetry_line(self, line: str) -> None:
        """Parse incoming real-time telemetry line (TLM:<x>,<y>,<yaw>,<us>,<ir1>,<ir2>)."""
        try:
            payload = line[4:].strip()
            parts = payload.split(",")
            if len(parts) >= 3:
                self._x = float(parts[0]) / 100.0
                self._y = float(parts[1]) / 100.0
                self._yaw = math.radians(float(parts[2]))
                self._yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi
                self._publish_current_pose()
            if len(parts) >= 6:
                us_m = float(parts[3]) / 100.0
                ir1_m = float(parts[4]) / 100.0
                ir2_m = float(parts[5]) / 100.0
                self._publish_sensor_ranges(us_m, ir1_m, ir2_m)
        except Exception as exc:
            self.get_logger().debug(f"Failed to parse telemetry {line!r}: {exc}")

    def _poll_telemetry(self) -> None:
        """Poll incoming telemetry when idle."""
        if self._busy.is_set():
            return
        line_bytes = None
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                return
            try:
                in_waiting = getattr(self._serial, "in_waiting", 0)
                if isinstance(in_waiting, int) and in_waiting <= 0:
                    return
                line_bytes = self._serial.readline()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"Serial read error in poll: {exc}")
                self._close_serial_locked()
                return

        if line_bytes:
            line = line_bytes.decode("ascii", errors="replace").strip()
            if line.startswith("TLM:"):
                self._handle_telemetry_line(line)

    def _read_status_line(self, deadline_sec: float) -> str:
        buf = b""
        deadline = time.monotonic() + deadline_sec
        while time.monotonic() < deadline:
            if self._estop_event.is_set():
                return "ESTOP"

            with self._write_lock:
                if self._serial is None or not self._serial.is_open:
                    return ""
                try:
                    chunk = self._serial.readline()  # bounded by serial timeout=0.5s
                except (serial.SerialException, OSError) as exc:
                    self.get_logger().error(f"Serial read error: {exc}")
                    self._close_serial_locked()
                    return ""

            if chunk:
                buf += chunk
                if buf.endswith(b"\n"):
                    line = buf.decode("ascii", errors="replace").strip()
                    buf = b""
                    if line.startswith("TLM:"):
                        self._handle_telemetry_line(line)
                        continue
                    if line:
                        return line
        return ""

    def destroy_node(self):
        try:
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


def main(args=None):
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
