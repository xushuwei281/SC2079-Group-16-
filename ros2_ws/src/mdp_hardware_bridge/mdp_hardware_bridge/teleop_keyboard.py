#!/usr/bin/env python3
"""Interactive keyboard teleoperation node for the MDP robot.

Captures single keystrokes without requiring <Enter> and sends discrete move
commands (FC, BC, FL, FR, BL, BR) or emergency stop to the robot.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Optional

import rclpy
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves
from rclpy.node import Node
from std_msgs.msg import Empty

BANNER = """
====================================================================
               MDP ROBOT KEYBOARD TELEOP
====================================================================
  [W] / [↑] : Forward (FC)         [S] / [↓] : Backward (BC)
  [A] / [←] : Turn Left (FL)       [D] / [→] : Turn Right (FR)
  [Q]       : Reverse Left (BL)    [E]       : Reverse Right (BR)

  [SPACE] / [X] : Emergency Stop (Instant Motor Brake)

  [+] / [-] : Adjust Distance Step (+/- 5 cm)
  [>] / [<] : Adjust Angle Step    (+/- 15 deg)
  [R]       : Reset to Defaults (20 cm, 90 deg)
  [Ctrl+C]  : Quit
====================================================================
"""


class TeleopKeyboardNode(Node):
    """ROS 2 node that interacts with /execute_moves and /estop."""

    def __init__(self) -> None:
        super().__init__("teleop_keyboard")

        self._move_client = self.create_client(ExecuteMoves, "/execute_moves")
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        self.dist_step: int = 20  # cm
        self.angle_step: int = 90  # deg
        self.busy: bool = False
        self.last_cmd: str = "None"
        self.last_status: str = "Ready"

    def send_move(self, command: str, value: int) -> None:
        """Send a move command asynchronously."""
        if self.busy:
            self.last_status = "IGNORED (Motion in progress)"
            self.refresh_display()
            return

        if not self._move_client.service_is_ready():
            self.last_status = "ERROR (/execute_moves service unavailable)"
            self.refresh_display()
            return

        self.busy = True
        self.last_cmd = f"{command} {value}"
        self.last_status = "Executing..."
        self.refresh_display()

        req = ExecuteMoves.Request()
        mc = MoveCommand()
        mc.command = command
        mc.value = value
        req.commands = [mc]

        future = self._move_client.call_async(req)
        future.add_done_callback(self._on_move_done)

    def _on_move_done(self, future: rclpy.task.Future) -> None:
        self.busy = False
        try:
            resp: ExecuteMoves.Response = future.result()
            if resp.success:
                self.last_status = f"DONE ({resp.status})"
            else:
                self.last_status = f"FAILED ({resp.status})"
        except Exception as exc:
            self.last_status = f"EXCEPTION ({exc})"
        self.refresh_display()

    def send_estop(self) -> None:
        """Publish an immediate emergency stop."""
        self._estop_pub.publish(Empty())
        self.last_cmd = "ESTOP"
        self.last_status = "EMERGENCY STOP SENT"
        self.refresh_display()

    def refresh_display(self) -> None:
        """Print current telemetry status line."""
        state_str = "BUSY" if self.busy else "READY"
        sys.stdout.write(
            f"\r\033[K[State: {state_str}] | "
            f"Step: {self.dist_step}cm / {self.angle_step}° | "
            f"Last: {self.last_cmd} -> {self.last_status} "
        )
        sys.stdout.flush()


def get_key(settings: list) -> str:
    """Read a single key (or arrow sequence) from stdin non-blocking."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        ch = sys.stdin.read(1)
        # Handle ANSI escape sequence for arrow keys
        if ch == "\x1b":
            # Peek for next chars
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist2:
                ch2 = sys.stdin.read(1)
                if ch2 == "[":
                    rlist3, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if rlist3:
                        ch3 = sys.stdin.read(1)
                        if ch3 == "A":
                            return "UP"
                        if ch3 == "B":
                            return "DOWN"
                        if ch3 == "C":
                            return "RIGHT"
                        if ch3 == "D":
                            return "LEFT"
            return "ESC"
        return ch
    return ""


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    # Spin ROS 2 in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Save terminal settings for clean restore
    old_settings = termios.tcgetattr(sys.stdin.fileno())

    print(BANNER)
    print("Waiting for /execute_moves service...")
    if node._move_client.wait_for_service(timeout_sec=5.0):
        print("Connected to hardware bridge! Ready for input.\n")
    else:
        print("Warning: /execute_moves is not available yet (start serial_bridge_node).\n")

    node.refresh_display()

    try:
        while rclpy.ok():
            key = get_key(old_settings)
            if not key:
                continue

            k = key.lower()

            if key in ("\x03", "ESC") or k == "c" and key == "\x03":  # Ctrl+C
                break

            elif key == "UP" or k == "w":
                node.send_move("FC", node.dist_step)
            elif key == "DOWN" or k == "s":
                node.send_move("BC", node.dist_step)
            elif key == "LEFT" or k == "a":
                node.send_move("FL", node.angle_step)
            elif key == "RIGHT" or k == "d":
                node.send_move("FR", node.angle_step)
            elif k == "q":
                node.send_move("BL", node.angle_step)
            elif k == "e":
                node.send_move("BR", node.angle_step)
            elif key == " " or k == "x":
                node.send_estop()

            # Step adjustments
            elif key in ("+", "="):
                node.dist_step = min(100, node.dist_step + 5)
                node.refresh_display()
            elif key in ("-", "_"):
                node.dist_step = max(5, node.dist_step - 5)
                node.refresh_display()
            elif key in (">", "."):
                node.angle_step = min(180, node.angle_step + 15)
                node.refresh_display()
            elif key in ("<", ","):
                node.angle_step = max(15, node.angle_step - 15)
                node.refresh_display()
            elif k == "r":
                node.dist_step = 20
                node.angle_step = 90
                node.last_status = "Reset to 20cm / 90°"
                node.refresh_display()

    except Exception as exc:
        print(f"\nTeleop error: {exc}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        print("\n\nTeleop keyboard closed.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
