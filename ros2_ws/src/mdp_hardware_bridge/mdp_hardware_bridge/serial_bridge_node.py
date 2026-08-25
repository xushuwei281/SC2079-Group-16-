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

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
from std_msgs.msg import Empty

from mdp_interfaces.srv import ExecuteMoves

# Confirmed valid 2-char command codes from the firmware's comm_task
# switch statement (stm32/src/main.c).
_VALID_COMMANDS = {"FC", "BC", "FL", "FR", "BL", "BR", "FU", "BU"}


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("serial_bridge_node")

        # Confirmed 2026-08-25: the STM32 link to the Pi is USB-serial
        # (ttyACMx), not raw GPIO UART pins, even though the firmware's own
        # UART peripheral config is USART3 @ 115200 -- the USB-CDC layer
        # just carries those same bytes. Exact device number (ACM0 vs ACM1,
        # if anything else on the Pi also enumerates as ACM) still TBD once
        # the Pi is wired up alongside everything else.
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

        # The service call blocks for a long time (real motion), so it
        # needs its own thread -- otherwise a running call would starve
        # the e-stop subscription's callback from ever running.
        callback_group = ReentrantCallbackGroup()

        self._estop_sub = self.create_subscription(
            Empty, "estop", self._on_estop, 10, callback_group=callback_group
        )
        self._service = self.create_service(
            ExecuteMoves,
            "execute_moves",
            self._handle_execute_moves,
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

    def _handle_execute_moves(self, request, response):
        if self._busy.is_set():
            response.success = False
            response.status = "BUSY_LOCAL"
            return response

        self._busy.set()
        try:
            with self._write_lock:
                self._serial.reset_input_buffer()
                for mc in request.commands:
                    self._serial.write(self._encode(mc.command, mc.value))
                # Trigger packet -- starts execution of everything just queued.
                self._serial.write(b"#\x00\x00\x00\x00")
                self._serial.flush()

            status = self._read_status_line(deadline_sec=5.0)
            if status != "RUN":
                response.success = False
                response.status = status or "NO_RESPONSE"
                return response

            status = self._read_status_line(deadline_sec=self._batch_timeout_sec)
            response.success = status == "FIN"
            response.status = status or "TIMEOUT"
        finally:
            self._busy.clear()
        return response

    def _read_status_line(self, deadline_sec: float) -> str:
        remaining = deadline_sec
        buf = b""
        while remaining > 0:
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
