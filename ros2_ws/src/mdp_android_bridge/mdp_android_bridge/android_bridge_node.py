"""Bluetooth RFCOMM bridge to the Android remote controller tablet.

The tablet is the second "driver" of the robot, alongside the autonomous
planner. It does NOT get its own path to the STM32: movement commands are
translated into calls on the same `execute_moves` service the planner
calls, so there is exactly one code path that ever writes to the UART.
See ARCHITECTURE.md, "Command arbitration" -- the busy-check inside that
service is what keeps the tablet and the planner from interleaving.

Three kinds of traffic come off the tablet:

  movement      FW/BW/TL/TR  -> one execute_moves call
  emergency     STP          -> publish on `estop`, NOT a service call
  everything    ADD/SUB/FACE -> republished on /android/cmd for the planner

STP deliberately bypasses the service. The firmware checks for a
'Q'-prefixed packet on every received packet regardless of what it is
doing, so e-stop is genuinely asynchronous on the hardware side; routing
it through a blocking service call would queue it behind the very move
it is meant to interrupt.

SCAFFOLDED, NOT YET RUN AGAINST A REAL TABLET. The wire format below is
implemented from docs/protocol.md's "Android <-> Raspberry Pi" table,
which that file itself marks as a negotiable draft. Two things to settle
with the Android subteam before trusting this (see _MOVEMENT_MAP):
whether the tablet really sends millimetres, and what it expects back.

This node also assumes /dev/rfcommN already exists. Binding it is done
outside ROS entirely -- pair the tablet with bluetoothctl, then
`sudo rfcomm bind 0 <MAC> 1`. If that has not been done, this node fails
at startup on the serial open, which is the correct place to fail.
"""

import threading

from mdp_interfaces.srv import ExecuteMoves
from mdp_interfaces.msg import MoveCommand
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
from std_msgs.msg import Empty, String

# Tablet command -> STM32 command code, plus how to convert the tablet's
# units into the firmware's.
#
# The STM32 takes whole centimetres and whole degrees (3-digit ASCII, see
# MoveCommand.msg). docs/protocol.md has the tablet sending MILLIMETRES
# for distance ("FW:50" = move forward 50mm), so distances are divided by
# ten here. Flagged because it is the most likely thing in this file to
# be wrong: at 1cm firmware resolution anything under FW:5 rounds to a
# no-op, and a 50mm default step is small enough to suggest the Android
# side may actually mean centimetres. Confirm before the first real
# tablet test rather than debugging it through the robot.
_MOVEMENT_MAP = {
    "FW": ("FC", 10),  # forward straight,  mm -> cm
    "BW": ("BC", 10),  # backward straight, mm -> cm
    "TL": ("FL", 1),   # turn left,  degrees, no conversion
    "TR": ("FR", 1),   # turn right, degrees, no conversion
}

# Non-movement commands relayed verbatim to the planner. These are the
# ARCM checklist's arena-editing messages (obstacle placement/removal and
# target-face annotation); this node has no opinion on their payloads.
_PLANNER_COMMANDS = {"ADD", "SUB", "FACE"}


class AndroidBridgeNode(Node):
    def __init__(self):
        super().__init__("android_bridge_node")

        self.declare_parameter("rfcomm_device", "/dev/rfcomm0")
        self.declare_parameter("baud_rate", 115200)
        # How long to wait for the hardware bridge to come up before
        # giving up on a single movement command. The service call itself
        # can then block far longer than this -- it is waiting on real
        # physical motion -- which is why the reply is handled in a
        # callback rather than by waiting on the future here.
        self.declare_parameter("service_wait_sec", 5.0)

        device = self.get_parameter("rfcomm_device").value
        baud = self.get_parameter("baud_rate").value
        self._service_wait_sec = self.get_parameter("service_wait_sec").value

        self._serial = serial.Serial(device, baud, timeout=0.5)
        # Guards writes back to the tablet only. The status subscription
        # and the reader thread both write, and they are not otherwise
        # synchronised.
        self._write_lock = threading.Lock()

        callback_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(String, "/android/cmd", 10)
        self._estop_pub = self.create_publisher(Empty, "estop", 10)
        self._status_sub = self.create_subscription(
            String, "/android/status", self._on_status, 10, callback_group=callback_group
        )
        self._move_client = self.create_client(
            ExecuteMoves, "execute_moves", callback_group=callback_group
        )

        # Plain thread rather than a ROS timer: the read is blocking, and
        # it must keep running while a movement command's service call is
        # still outstanding, so that STP stays reachable.
        self._running = threading.Event()
        self._running.set()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(f"Listening to tablet on {device} @ {baud}")

    # ---- tablet -> ROS ----------------------------------------------------

    def _read_loop(self) -> None:
        while self._running.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f"RFCOMM read failed: {exc}")
                return
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self._dispatch(line)

    def _dispatch(self, line: str) -> None:
        # STP carries no value and must stay on the fast path.
        if line == "STP":
            self.get_logger().warn("Tablet requested E-STOP")
            self._estop_pub.publish(Empty())
            self._send("STATUS,estop")
            return

        verb, _, arg = line.partition(":")

        if verb in _MOVEMENT_MAP:
            self._handle_movement(verb, arg, line)
        elif verb in _PLANNER_COMMANDS:
            self._cmd_pub.publish(String(data=line))
        else:
            self.get_logger().warn(f"Unrecognised tablet command: {line!r}")
            self._send(f"STATUS,unknown command {verb}")

    def _handle_movement(self, verb: str, arg: str, line: str) -> None:
        code, divisor = _MOVEMENT_MAP[verb]
        try:
            value = round(int(arg) / divisor)
        except ValueError:
            self.get_logger().warn(f"Non-numeric value in {line!r}")
            self._send(f"STATUS,bad value in {line}")
            return

        if not (0 <= value <= 999):
            # Same range the firmware's 3-digit ASCII field can express;
            # rejecting here gives the tablet a reason instead of letting
            # the hardware bridge raise on encode.
            self.get_logger().warn(f"{line!r} out of range after conversion ({value})")
            self._send(f"STATUS,out of range {line}")
            return

        if not self._move_client.wait_for_service(timeout_sec=self._service_wait_sec):
            self.get_logger().error("execute_moves unavailable -- is the hardware bridge running?")
            self._send("STATUS,hardware bridge offline")
            return

        request = ExecuteMoves.Request()
        request.commands = [MoveCommand(command=code, value=value)]

        self._send(f"STATUS,moving {code}{value:03d}")
        future = self._move_client.call_async(request)
        future.add_done_callback(self._on_move_complete)

    def _on_move_complete(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- surface any transport failure to the tablet
            self.get_logger().error(f"execute_moves call failed: {exc}")
            self._send("STATUS,move failed")
            return

        if response.success:
            # The protocol's completion signal, kept distinct from STATUS
            # so the tablet can drive UI state off it rather than parsing
            # free text.
            self._send("DONE")
        else:
            # BUSY_LOCAL / BUS / FUL / TIMEOUT -- pass the reason through
            # rather than flattening it, the tablet is a debugging surface
            # as much as a controller.
            self.get_logger().warn(f"Move rejected: {response.status}")
            self._send(f"STATUS,rejected {response.status}")

    # ---- ROS -> tablet ----------------------------------------------------

    def _on_status(self, msg: String) -> None:
        # Curated status text from the planner. Prefixed here rather than
        # at the publisher so nothing upstream has to know the tablet's
        # wire format -- and so the tablet's status view only ever shows
        # STATUS,/DONE lines, never the raw stream (checklist item C.4).
        self._send(f"STATUS,{msg.data}")

    def _send(self, text: str) -> None:
        with self._write_lock:
            try:
                self._serial.write(f"{text}\n".encode("ascii"))
                self._serial.flush()
            except serial.SerialException as exc:
                self.get_logger().error(f"RFCOMM write failed: {exc}")

    def destroy_node(self):
        self._running.clear()
        self._reader.join(timeout=2.0)
        if self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AndroidBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
