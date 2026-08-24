"""UART bridge to the STM32 motor controller.

Speaks the STM32's actual protocol directly (confirmed by reading the
ported STM_Ref firmware's comm_task dispatch, see stm32/src/main.c) --
NOT a continuous ros2_control interface. The firmware executes discrete,
blocking maneuvers (drive N cm, turn N degrees, ...), not a live
velocity/steering stream, so this node exposes that as a ROS2 action
instead of a ros2_control hardware_interface.

Packet format: 5 bytes = 2-char command code + 3-digit zero-padded ASCII
value (e.g. b"FC050" = drive forward 50cm). A batch of move packets is
followed by a '#'-prefixed trigger packet; the STM32 replies b"RUN\\r\\n"
when it starts, executes every queued move in sequence, then replies
b"FIN\\r\\n" (or b"BUS\\r\\n" / b"FUL\\r\\n" if rejected). A 'Q'-prefixed
packet triggers immediate e-stop independent of anything queued -- this
node mirrors that by never gating the e-stop write behind the lock used
for a long-running batch send/wait.
"""

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
from std_msgs.msg import Empty

from mdp_interfaces.action import ExecuteMoves

# Confirmed valid 2-char command codes from the firmware's comm_task
# switch statement (stm32/src/main.c).
_VALID_COMMANDS = {"FC", "BC", "FL", "FR", "BL", "BR", "FU", "BU"}


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("serial_bridge_node")

        # TODO verify on real hardware -- the course's RPi-InfraSetup guide
        # describes the STM32 link as USB-serial (ttyACMx), not raw GPIO
        # UART pins, even though the firmware config is USART3 @ 115200.
        # Confirm the actual device path once the Pi is wired up.
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        # How long to wait for FIN/BUS/FUL after triggering a batch.
        # Move batches are physically blocking on the STM32 side (real
        # acceleration/deceleration ramps), so this needs real headroom --
        # start generous and tighten once you know real maneuver timing.
        self.declare_parameter("batch_timeout_sec", 30.0)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self._batch_timeout_sec = self.get_parameter("batch_timeout_sec").value

        self._serial = serial.Serial(port, baud, timeout=0.5)
        # Guards raw writes only -- never held across a blocking read, so
        # e-stop can always preempt a batch that's mid-execution.
        self._write_lock = threading.Lock()
        self._busy = threading.Event()

        # Actions block for a long time (real motion), so they need their
        # own thread -- otherwise a running goal would starve the e-stop
        # subscription's callback from ever running.
        callback_group = ReentrantCallbackGroup()

        self._estop_sub = self.create_subscription(
            Empty, "estop", self._on_estop, 10, callback_group=callback_group
        )
        self._action_server = ActionServer(
            self,
            ExecuteMoves,
            "execute_moves",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(f"Connected to STM32 on {port} @ {baud}")

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
        self.get_logger().warn("E-STOP requested")
        with self._write_lock:
            self._serial.write(b"Q\x00\x00\x00\x00")
            self._serial.flush()

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._busy.is_set():
            self.get_logger().warn("Rejecting goal -- a batch is already executing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        result = ExecuteMoves.Result()
        commands = goal_handle.request.commands
        feedback = ExecuteMoves.Feedback()
        feedback.commands_total = len(commands)

        self._busy.set()
        try:
            with self._write_lock:
                self._serial.reset_input_buffer()
                for i, mc in enumerate(commands):
                    packet = self._encode(mc.command, mc.value)
                    self._serial.write(packet)
                    feedback.commands_sent = i + 1
                    goal_handle.publish_feedback(feedback)
                # Trigger packet -- starts execution of everything just queued.
                self._serial.write(b"#\x00\x00\x00\x00")
                self._serial.flush()

            # Wait for RUN, then FIN/BUS/FUL. Read with a loop of short
            # timeouts (not one long blocking read) so a cancel request
            # is noticed promptly instead of only after the whole wait
            # elapses.
            status = self._read_status_line(
                deadline_sec=5.0, goal_handle=goal_handle
            )
            if status != "RUN":
                result.success = False
                result.status = status or "NO_RESPONSE"
                goal_handle.abort()
                return result

            status = self._read_status_line(
                deadline_sec=self._batch_timeout_sec, goal_handle=goal_handle
            )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.status = "CANCELED"
                return result

            result.success = status == "FIN"
            result.status = status or "TIMEOUT"
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        finally:
            self._busy.clear()
        return result

    def _read_status_line(self, deadline_sec: float, goal_handle) -> str:
        """Poll for a \\r\\n-terminated status line without blocking so long
        that a cancel request or node shutdown can't be noticed."""
        remaining = deadline_sec
        buf = b""
        while remaining > 0:
            if goal_handle.is_cancel_requested:
                return ""
            chunk = self._serial.readline()  # bounded by serial timeout=0.5s
            remaining -= 0.5
            if chunk:
                buf += chunk
                if buf.endswith(b"\n"):
                    return buf.decode("ascii", errors="replace").strip()
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
