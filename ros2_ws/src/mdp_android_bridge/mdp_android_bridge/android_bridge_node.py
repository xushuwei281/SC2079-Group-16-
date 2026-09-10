#!/usr/bin/env python3
"""Bluetooth RFCOMM bridge to the Android tablet remote controller.

Features:
- Resilient Auto-Connect: Gracefully waits for /dev/rfcomm0 and auto-reconnects on drop.
- Bidirectional Comms:
    * Inbound (Tablet -> ROS):
        - Movement: FW/BW/TL/TR/BL/BR or FC/BC/FL/FR -> calls /execute_moves
        - Emergency: STP/STOP/Q -> publishes to /estop
        - Arena/Planner: ALG|..., START, RESET -> publishes to /android/cmd
    * Outbound (ROS -> Tablet):
        - Status updates: STATUS,<text>
        - Move completion: DONE
        - Live Robot Pose (Checklist C.10): ROBOT,<x>,<y>,<dir> from
          /robot_pose -- x/y are grid cells [0..19], dir is N/E/S/W.
        - Live Robot Pose (supplemental, full precision): POSE,<x_cm>,
          <y_cm>,<yaw_deg>, sent alongside ROBOT at up to
          hires_pose_rate_hz so the tablet's arena view can track
          smoothly through turns instead of only seeing 4 discrete
          headings and 10cm-quantized positions.
        - Image Detections: TARGET,<obstacle_id>,<symbol_id> from /android/target
"""

from __future__ import annotations

import math
import os
import termios
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
import serial
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

# Inbound Movement Mapping
# Maps tablet command prefixes to STM32 command codes
_MOVEMENT_MAP = {
    # Standard MDP format
    "FW": "FC",  # Forward straight
    "BW": "BC",  # Backward straight
    "TL": "FL",  # Turn left
    "TR": "FR",  # Turn right
    "BL": "BL",  # Backward left
    "BR": "BR",  # Backward right
    # Direct STM32 2-letter codes
    "FC": "FC",
    "BC": "BC",
    "FL": "FL",
    "FR": "FR",
}

# Mission / Planner / Arena / Task 2 commands forwarded verbatim to /android/cmd
_MISSION_COMMAND_PREFIXES = (
    "ALG", "START", "RESET", "ADD", "SUB", "FACE", "STM", "SP", "TASK2", "START_TASK2", "MODE"
)


class AndroidBridgeNode(Node):
    """ROS 2 bridge managing Bluetooth RFCOMM communication with the Android tablet."""

    def __init__(self) -> None:
        super().__init__("android_bridge_node")

        self.declare_parameter("rfcomm_device", "/dev/rfcomm0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("service_wait_sec", 5.0)
        self.declare_parameter("distance_in_mm", False)  # True if tablet sends mm, False if cm
        self.declare_parameter("use_angle_brackets_for_pose", True)  # Format: ROBOT,<x>,<y>,<dir>
        self.declare_parameter("robot_coords_in_cm", False)  # False: 0-19 grid cells, True: cm
        self.declare_parameter("direction_as_cardinal", True)  # True: N/S/E/W, False: degrees
        # Supplemental, non-checklist pose stream: full-precision cm/degrees,
        # sent alongside (not instead of) the Checklist C.10 ROBOT line so
        # the arena view can track smoothly through turns. 0 disables it.
        self.declare_parameter("hires_pose_rate_hz", 10.0)

        self._device = self.get_parameter("rfcomm_device").value
        self._baud = self.get_parameter("baud_rate").value
        self._service_wait_sec = self.get_parameter("service_wait_sec").value
        self._distance_in_mm = self.get_parameter("distance_in_mm").value
        self._use_brackets = self.get_parameter("use_angle_brackets_for_pose").value
        self._coords_in_cm = self.get_parameter("robot_coords_in_cm").value
        self._direction_as_cardinal = self.get_parameter("direction_as_cardinal").value
        hires_rate = self.get_parameter("hires_pose_rate_hz").value
        self._hires_pose_interval_sec = (1.0 / hires_rate) if hires_rate > 0 else None

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        self._move_lock = threading.Lock()
        self._is_moving = False
        self._pending_move: Optional[Tuple[str, int]] = None
        self._last_sent_pose_str: str = ""
        self._last_pose_time: float = 0.0
        self._last_hires_pose_time: float = 0.0

        # Sensor distance tracking (cm)
        self._us_cm: float = -1.0
        self._ir_l_cm: float = -1.0
        self._ir_r_cm: float = -1.0
        self._last_sensor_time: float = 0.0

        callback_group = ReentrantCallbackGroup()

        # Publishers
        self._cmd_pub = self.create_publisher(String, "/android/cmd", 10)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        # Subscribers
        self._status_sub = self.create_subscription(
            String, "/android/status", self._on_status, 10, callback_group=callback_group
        )
        self._target_sub = self.create_subscription(
            String, "/android/target", self._on_target, 10, callback_group=callback_group
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._on_pose, 10, callback_group=callback_group
        )
        self._telemetry_sub = self.create_subscription(
            String, "/android/telemetry", self._on_telemetry, 10, callback_group=callback_group
        )
        self._us_sub = self.create_subscription(
            Range, "/sensors/ultrasonic", self._on_us_range, 10, callback_group=callback_group
        )
        self._ir_l_sub = self.create_subscription(
            Range, "/sensors/ir_left", self._on_ir_l_range, 10, callback_group=callback_group
        )
        self._ir_r_sub = self.create_subscription(
            Range, "/sensors/ir_right", self._on_ir_r_range, 10, callback_group=callback_group
        )

        # Service Clients
        self._move_client = self.create_client(
            ExecuteMoves, "/execute_moves", callback_group=callback_group
        )

        # Background worker thread for connection and read loop
        self._comm_thread = threading.Thread(target=self._connection_worker, daemon=True)
        self._comm_thread.start()

        self.get_logger().info(
            f"Android bridge started. Monitoring {self._device} for tablet connection..."
        )

    # -------------------------------------------------------------------------
    # Connection Management & Read Loop
    # -------------------------------------------------------------------------

    def _connection_worker(self) -> None:
        """Main loop that ensures persistent connection to /dev/rfcomm0."""
        while self._running.is_set():
            if not self._is_connected():
                self._try_connect()
                if not self._is_connected():
                    time.sleep(1.0)
                    continue

            # Connected: read loop
            try:
                raw_line = self._serial.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").strip()
                if line:
                    self.get_logger().info(f"[Tablet -> Pi]: {line}")
                    self._dispatch(line)

            except (serial.SerialException, OSError, termios.error, TypeError, AttributeError) as exc:
                if not self._running.is_set():
                    break
                self.get_logger().warn(f"RFCOMM connection lost: {exc}")
                self._close_serial()
                time.sleep(1.0)

    def _is_connected(self) -> bool:
        with self._serial_lock:
            return self._serial is not None and self._serial.is_open

    def _try_connect(self) -> bool:
        """Attempt to open /dev/rfcomm0 if it exists."""
        if not os.path.exists(self._device):
            return False

        with self._serial_lock:
            try:
                self._serial = serial.Serial(self._device, self._baud, timeout=0.5)
                self.get_logger().info(f"Connected to Android tablet on {self._device}!")
                self._send_locked("STATUS,Connected to Robot")
                return True
            except (serial.SerialException, OSError, termios.error) as exc:
                self._serial = None
                return False

    def _close_serial(self) -> None:
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

    # -------------------------------------------------------------------------
    # Inbound Message Dispatching (Tablet -> ROS)
    # -------------------------------------------------------------------------

    def _dispatch(self, line: str) -> None:
        """Parse incoming line from the tablet and route to appropriate ROS component."""
        # 1. Emergency Stop (Fast Path)
        if line.upper() in ("STP", "STOP", "Q"):
            self.get_logger().warn("Tablet requested E-STOP")
            with self._move_lock:
                self._pending_move = None
                self._is_moving = False
            self._estop_pub.publish(Empty())
            self.send_to_tablet("STATUS,E-STOP TRIGGERED")
            return

        # 2. Mission & Arena Commands (ALG|..., START, RESET, ADD, SUB, FACE, STM:sp, SP, TASK2)
        if line.upper().startswith(_MISSION_COMMAND_PREFIXES):
            self.get_logger().info(f"Forwarding mission command: {line}")
            with self._move_lock:
                self._pending_move = None
            self._cmd_pub.publish(String(data=line))
            self.send_to_tablet(f"STATUS,Received {line.split('|')[0]}")
            return

        # 3. Movement Commands (e.g. FW:20, TL:90, FC:20, FR:90)
        verb, sep, arg = line.partition(":")
        verb_upper = verb.strip().upper()

        if verb_upper in _MOVEMENT_MAP:
            self._handle_movement(verb_upper, arg.strip(), line)
        else:
            self.get_logger().warn(f"Unrecognized tablet command: {line}")
            self.send_to_tablet(f"STATUS,Unknown command: {verb}")

    def _handle_movement(self, verb: str, arg: str, raw_line: str) -> None:
        """Convert tablet movement command and trigger /execute_moves asynchronously."""
        code = _MOVEMENT_MAP[verb]

        try:
            raw_val = int(arg)
        except ValueError:
            self.get_logger().warn(f"Non-numeric value in movement command: {raw_line}")
            self.send_to_tablet(f"STATUS,Invalid value: {arg}")
            return

        # Distance unit conversion
        if code in ("FC", "BC") and (self._distance_in_mm or raw_val >= 100):
            # If value is >= 100 on a linear move or config says mm, divide by 10
            value = max(1, round(raw_val / 10.0))
        else:
            value = raw_val

        # Range validation (0–999)
        if not (0 <= value <= 999):
            self.get_logger().warn(f"Value out of range (0-999): {value}")
            self.send_to_tablet(f"STATUS,Value out of range: {value}")
            return

        with self._move_lock:
            if self._is_moving:
                # Buffer the latest streaming command (overwriting prior pending move)
                self._pending_move = (code, value)
                self.get_logger().debug(
                    f"Move in progress; buffered pending command {code}{value:03d}"
                )
                return

            self._is_moving = True
            self._dispatch_move_locked(code, value)

    def _dispatch_move_locked(self, code: str, value: int) -> None:
        """Dispatch a move to /execute_moves service. Must be called with _move_lock held."""
        if not self._move_client.wait_for_service(timeout_sec=self._service_wait_sec):
            self.get_logger().error("/execute_moves service unavailable")
            self.send_to_tablet("STATUS,Hardware bridge offline")
            self._is_moving = False
            self._pending_move = None
            return

        req = ExecuteMoves.Request()
        mc = MoveCommand()
        mc.command = code
        mc.value = value
        req.commands = [mc]

        self.send_to_tablet(f"STATUS,Moving {code}{value:03d}")
        future = self._move_client.call_async(req)
        future.add_done_callback(self._on_move_complete)

    def _on_move_complete(self, future: rclpy.task.Future) -> None:
        """Callback when /execute_moves completes."""
        try:
            resp: ExecuteMoves.Response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Move service exception: {exc}")
            with self._move_lock:
                self._is_moving = False
                self._pending_move = None
            self.send_to_tablet("STATUS,Move error")
            return

        with self._move_lock:
            if resp.success:
                if self._pending_move is not None:
                    # Immediately chain into the pending move without idling
                    next_code, next_value = self._pending_move
                    self._pending_move = None
                    self.get_logger().debug(
                        f"Move finished; immediately dispatching buffered {next_code}{next_value:03d}"
                    )
                    self._dispatch_move_locked(next_code, next_value)
                else:
                    self._is_moving = False
                    self.send_to_tablet("DONE")
            else:
                # Suppress noisy error alerts if stream caused BUSY_LOCAL
                if resp.status == "BUSY_LOCAL":
                    self.get_logger().info("Move rejected due to BUSY_LOCAL")
                else:
                    self.get_logger().warn(f"Move rejected or failed: {resp.status}")
                    self.send_to_tablet(f"STATUS,Move failed: {resp.status}")
                self._is_moving = False
                self._pending_move = None

    # -------------------------------------------------------------------------
    # Outbound Message Streaming (ROS -> Tablet)
    # -------------------------------------------------------------------------

    def _on_status(self, msg: String) -> None:
        """Forward status updates to the tablet."""
        self.send_to_tablet(f"STATUS,{msg.data}")

    def _on_target(self, msg: String) -> None:
        """Forward target recognition results to the tablet (format: TARGET,<obs_id>,<symbol_id>)."""
        self.send_to_tablet(f"TARGET,{msg.data}")

    def _on_telemetry(self, msg: String) -> None:
        """Forward raw structured telemetry directly to the tablet."""
        self.send_to_tablet(msg.data)

    def _on_us_range(self, msg: Range) -> None:
        """Track front ultrasonic sensor range and emit throttled SENSORS update."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._us_cm = msg.range * 100.0
        else:
            self._us_cm = -1.0
        self._maybe_send_sensors()

    def _on_ir_l_range(self, msg: Range) -> None:
        """Track front-left IR sensor range and emit throttled SENSORS update."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._ir_l_cm = msg.range * 100.0
        else:
            self._ir_l_cm = -1.0
        self._maybe_send_sensors()

    def _on_ir_r_range(self, msg: Range) -> None:
        """Track front-right IR sensor range and emit throttled SENSORS update."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._ir_r_cm = msg.range * 100.0
        else:
            self._ir_r_cm = -1.0
        self._maybe_send_sensors()

    def _maybe_send_sensors(self) -> None:
        """Stream SENSORS,<us_cm>,<ir_left_cm>,<ir_right_cm> at up to 4 Hz."""
        now = time.monotonic()
        if now - self._last_sensor_time >= 0.25:
            self._last_sensor_time = now
            self.send_to_tablet(
                f"SENSORS,{self._us_cm:.1f},{self._ir_l_cm:.1f},{self._ir_r_cm:.1f}"
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        """Convert ROS PoseStamped to tablet format: ROBOT,<x>,<y>,<direction>."""
        x_cm = msg.pose.position.x * 100.0
        y_cm = msg.pose.position.y * 100.0

        # Quaternion to yaw (degrees, 0-359)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = round(math.degrees(yaw_rad)) % 360

        # Cardinal direction mapping according to Checklist C.10 (N, S, E, W)
        if self._direction_as_cardinal:
            if 45 <= yaw_deg < 135:
                direction_str = "N"
            elif 135 <= yaw_deg < 225:
                direction_str = "W"
            elif 225 <= yaw_deg < 315:
                direction_str = "S"
            else:
                direction_str = "E"
        else:
            direction_str = str(yaw_deg)

        # Coordinate domain: grid cells [0..19] for Android canvas, or cm
        if self._coords_in_cm:
            px = round(x_cm)
            py = round(y_cm)
        else:
            px = max(0, min(19, int(round(x_cm / 10.0))))
            py = max(0, min(19, int(round(y_cm / 10.0))))

        now = time.monotonic()

        # Supplemental full-precision line, independent of the ROBOT
        # dedup/throttle below -- it's meant to change on every call
        # (that's the whole point, for smooth tracking through turns), just
        # rate-limited so a busy TLM feed doesn't flood the RFCOMM link.
        if self._hires_pose_interval_sec is not None and (
            now - self._last_hires_pose_time >= self._hires_pose_interval_sec
        ):
            self._last_hires_pose_time = now
            self.send_to_tablet(f"POSE,{x_cm:.1f},{y_cm:.1f},{yaw_deg}")

        # Formatting with or without angle brackets
        if self._use_brackets:
            pose_str = f"ROBOT,<{px}>,<{py}>,<{direction_str}>"
        else:
            pose_str = f"ROBOT,{px},{py},{direction_str}"

        if pose_str == self._last_sent_pose_str and (now - self._last_pose_time) < 0.5:
            return
        self._last_sent_pose_str = pose_str
        self._last_pose_time = now

        self.send_to_tablet(pose_str)

    def send_to_tablet(self, text: str) -> None:
        """Send a single line of text to the tablet over Bluetooth."""
        with self._serial_lock:
            self._send_locked(text)

    def _send_locked(self, text: str) -> None:
        """Internal write when lock is already acquired."""
        if self._serial is None or not self._serial.is_open:
            return
        try:
            line = f"{text}\n".encode("utf-8")
            self._serial.write(line)
            self._serial.flush()
        except (serial.SerialException, OSError, termios.error) as exc:
            # termios.error (pyserial's posix backend raises this from
            # tcdrain() inside flush()) is NOT an OSError subclass -- it
            # slipped through this catch entirely and took the whole node
            # down with it (confirmed from a real crash: a transient RFCOMM
            # I/O error during a POSE send killed android_bridge_node via
            # an uncaught exception propagating out of the executor).
            self.get_logger().warn(f"RFCOMM write error: {exc}")
            # Inlined _close_serial()'s body rather than calling it: every
            # caller of _send_locked already holds _serial_lock (it's a
            # plain Lock, not reentrant), so calling the lock-acquiring
            # _close_serial() from here would deadlock.
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def destroy_node(self) -> None:
        try:
            self._running.clear()
            with self._move_lock:
                self._pending_move = None
                self._is_moving = False
            self._close_serial()
            if hasattr(self, "_comm_thread") and self._comm_thread.is_alive():
                self._comm_thread.join(timeout=0.5)
        except (Exception, KeyboardInterrupt):
            pass
        try:
            return super().destroy_node()
        except (Exception, KeyboardInterrupt):
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AndroidBridgeNode()
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
