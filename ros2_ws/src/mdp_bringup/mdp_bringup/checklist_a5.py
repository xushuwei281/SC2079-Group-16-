#!/usr/bin/env python3
"""Checklist A.5: Obstacle Ultrasonic Approach and Face Recognition Routine.

Autonomous execution procedure for NTU SC2079 MDP Checklist Item A.5:
1. Approaches an obstacle using forward movement monitored by the forward ultrasonic sensor
   until reaching a safe camera viewing distance (~25 cm).
2. Attempts to sample and recognize the target image on the current obstacle face.
3. If no target image is detected (or a Bull's Eye marker is detected), executes the calibrated
   Orbit Right macro ([BC 30, FR 90, FC 18, FL 180]) to inspect adjacent faces until a valid
   target symbol (IDs 11-39) is identified.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

# Default calibrated Orbit Right macro: [BC 30, FR 90, FC 18, FL 180]
DEFAULT_ORBIT_MACRO: List[Tuple[str, int]] = [
    ("BC", 30),
    ("FR", 90),
    ("FC", 18),
    ("FL", 180),
]


class ChecklistA5Node(Node):
    """Checklist A.5 Orchestrator Node."""

    def __init__(
        self,
        target_distance_cm: float = 25.0,
        max_faces: int = 4,
        approach_step_cm: float = 15.0,
        macro: Optional[List[Tuple[str, int]]] = None,
    ) -> None:
        super().__init__("checklist_a5_node")

        self.declare_parameter("target_distance_cm", target_distance_cm)
        self.declare_parameter("max_faces", max_faces)
        self.declare_parameter("approach_step_cm", approach_step_cm)
        self.declare_parameter("stop_threshold_margin_cm", 2.0)
        self.declare_parameter("sample_timeout_s", 3.0)

        self._target_dist_cm = float(self.get_parameter("target_distance_cm").value)
        self._max_faces = int(self.get_parameter("max_faces").value)
        self._approach_step_cm = float(self.get_parameter("approach_step_cm").value)
        self._margin_cm = float(self.get_parameter("stop_threshold_margin_cm").value)
        self._sample_timeout_s = float(self.get_parameter("sample_timeout_s").value)

        self._macro = macro or DEFAULT_ORBIT_MACRO
        self._is_running = True
        self._worker_thread: Optional[threading.Thread] = None

        # Live sensor feedback
        self._lock = threading.Lock()
        self._us_range_m = float("nan")
        self._us_stamp = 0.0

        # Callback groups
        cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self._us_sub = self.create_subscription(
            Range,
            "/sensors/ultrasonic",
            self._on_ultrasonic,
            10,
            callback_group=cb_group,
        )

        # Service clients
        self._move_client = self.create_client(
            ExecuteMoves,
            "/execute_moves",
            callback_group=cb_group,
        )
        self._sample_client = self.create_client(
            SampleTarget,
            "/perception/sample_target",
            callback_group=cb_group,
        )

        # Publishers
        self._status_pub = self.create_publisher(String, "/android/status", 10)
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        self._cmd_pub = self.create_publisher(String, "/android/cmd", 10)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        self.get_logger().info(
            f"Checklist A.5 Node initialized. Safe target distance: {self._target_dist_cm:.1f} cm, "
            f"Macro: {' -> '.join([f'{c}{v:03d}' for c, v in self._macro])}"
        )

    def _on_ultrasonic(self, msg: Range) -> None:
        """Cache latest Kalman-filtered ultrasonic distance."""
        with self._lock:
            self._us_range_m = msg.range
            self._us_stamp = time.monotonic()

    @property
    def current_distance_cm(self) -> float:
        """Return the current ultrasonic measurement in centimeters."""
        with self._lock:
            if not math.isfinite(self._us_range_m) or self._us_range_m <= 0:
                return float("inf")
            return self._us_range_m * 100.0

    def start_routine(self) -> None:
        """Start the A.5 checklist routine in a background thread."""
        self._is_running = True
        self._worker_thread = threading.Thread(target=self._run_a5_loop, daemon=True)
        self._worker_thread.start()

    def stop_routine(self) -> None:
        """Signal routine to stop immediately."""
        self._is_running = False

    def _execute_move_command_sync(self, cmd: str, val: int) -> bool:
        """Synchronously execute a single movement primitive."""
        if not self._is_running:
            return False

        req = ExecuteMoves.Request()
        mc = MoveCommand()
        mc.command = cmd
        mc.value = int(val)
        req.commands.append(mc)

        future = self._move_client.call_async(req)
        while rclpy.ok() and self._is_running and not future.done():
            time.sleep(0.02)

        if future.done() and future.result() is not None:
            res: ExecuteMoves.Response = future.result()
            return bool(res.success)
        return False

    def _execute_macro_sync(self) -> bool:
        """Execute the calibrated Orbit Right macro primitive-by-primitive."""
        self.get_logger().info(
            f"🔄 Executing Orbit Right macro ({len(self._macro)} steps): "
            f"{' -> '.join([f'{c}{v:03d}' for c, v in self._macro])}"
        )
        self._status_pub.publish(String(data="A.5: Orbiting to next face..."))

        for idx, (code, val) in enumerate(self._macro):
            if not self._is_running:
                return False
            self.get_logger().info(f"  Step {idx+1}/{len(self._macro)}: {code}{val:03d}")
            ok = self._execute_move_command_sync(code, val)
            if not ok:
                self.get_logger().warn(f"Macro step {code}{val:03d} failed or was interrupted.")
                return False
            time.sleep(0.05)

        self.get_logger().info("✅ Orbit Right macro completed.")
        return True

    def _approach_obstacle(self) -> bool:
        """Move forward in increments using ultrasonic sensor until within safe viewing distance."""
        self.get_logger().info(
            f"🎯 Approaching obstacle until distance <= {self._target_dist_cm + self._margin_cm:.1f} cm..."
        )
        self._status_pub.publish(String(data="A.5: Approaching obstacle..."))

        max_steps = 25
        for step in range(max_steps):
            if not self._is_running:
                return False

            dist_cm = self.current_distance_cm

            # Already at or within the safe viewing window
            if dist_cm <= (self._target_dist_cm + self._margin_cm):
                self.get_logger().info(
                    f"✅ Safe distance reached: {dist_cm:.1f} cm (target: {self._target_dist_cm:.1f} cm)."
                )
                return True

            # Calculate safe incremental step
            if math.isinf(dist_cm):
                step_cm = int(self._approach_step_cm)
            else:
                gap = dist_cm - self._target_dist_cm
                step_cm = max(5, min(int(self._approach_step_cm), int(gap)))

            self.get_logger().info(
                f"  US: {dist_cm:.1f} cm | Gap: {dist_cm - self._target_dist_cm:.1f} cm | Advancing {step_cm} cm (FC{step_cm:03d})"
            )
            ok = self._execute_move_command_sync("FC", step_cm)
            if not ok:
                self.get_logger().warn("Forward approach step failed or was halted.")
                return False

            time.sleep(0.1)

        self.get_logger().warn("Reached maximum approach steps without finding distance threshold.")
        return False

    def _sample_target(self, face_idx: int) -> Optional[Tuple[int, str, float]]:
        """Query perception consensus sampler on the current face.

        Returns (symbol_id, symbol_name, confidence) if a valid target symbol (11-39)
        is confirmed, or None if no symbol or Bull's Eye (ID 40) is seen.
        """
        time.sleep(0.4)  # Allow chassis and camera to settle

        if not (self._sample_client.service_is_ready() or self._sample_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().warn("Perception service /perception/sample_target is unavailable.")
            return None

        self.get_logger().info(f"📷 Scanning face {face_idx} for target image...")
        self._status_pub.publish(String(data=f"A.5: Scanning Face {face_idx}..."))

        req = SampleTarget.Request()
        req.obstacle_id = 1
        future = self._sample_client.call_async(req)

        deadline = time.time() + self._sample_timeout_s
        while rclpy.ok() and self._is_running and not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self.get_logger().info(f"Face {face_idx}: Perception sampling timed out (no consensus).")
            return None

        res: SampleTarget.Response = future.result()
        if res.success and not res.is_marker and 11 <= res.symbol_id <= 39:
            return (res.symbol_id, res.symbol_name, float(res.confidence))

        if res.is_marker or res.symbol_id == 40:
            self.get_logger().info(f"Face {face_idx}: Bull's Eye marker detected (ID {res.symbol_id}). Target is on another face.")
        else:
            self.get_logger().info(f"Face {face_idx}: No target image detected on this face.")
        return None

    def _run_a5_loop(self) -> None:
        """Main execution sequence for Checklist A.5."""
        self.get_logger().info("============================================================")
        self.get_logger().info("       🚀 STARTING CHECKLIST A.5: APPROACH & ORBIT         ")
        self.get_logger().info("============================================================")

        # 1. Wait for hardware bridge
        if not self._move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Hardware bridge /execute_moves unavailable.")
            self._status_pub.publish(String(data="A.5: Hardware bridge offline"))
            self._is_running = False
            return

        # 2. Wait for initial ultrasonic telemetry
        self.get_logger().info("Waiting for ultrasonic sensor feed...")
        t_wait = time.time()
        while rclpy.ok() and self._is_running and math.isinf(self.current_distance_cm):
            if time.time() - t_wait > 5.0:
                self.get_logger().warn("Ultrasonic sensor reading timed out; proceeding with caution.")
                break
            time.sleep(0.05)

        # ---------------------------------------------------------------------
        # PHASE 1: Approach Obstacle to Safe Distance
        # ---------------------------------------------------------------------
        approach_ok = self._approach_obstacle()
        if not approach_ok or not self._is_running:
            self.get_logger().warn("Approach phase did not complete successfully.")

        # ---------------------------------------------------------------------
        # PHASE 2: Face Inspection & Orbit Macro Loop
        # ---------------------------------------------------------------------
        target_found = False
        for face_idx in range(1, self._max_faces + 1):
            if not self._is_running:
                break

            self.get_logger().info(f"\n--- [Face {face_idx}/{self._max_faces}] Inspecting Target ---")
            sample_result = self._sample_target(face_idx)

            if sample_result is not None:
                sid, sname, conf = sample_result
                target_found = True
                dist_cm = self.current_distance_cm
                self.get_logger().info("============================================================")
                self.get_logger().info(f"  🎉 A.5 CHECKLIST SUCCESS: TARGET DETECTED ON FACE {face_idx}!")
                self.get_logger().info(f"  Symbol ID:    {sid}")
                self.get_logger().info(f"  Symbol Name:  {sname}")
                self.get_logger().info(f"  Confidence:   {conf:.2f}")
                self.get_logger().info(f"  Distance:     {dist_cm:.1f} cm")
                self.get_logger().info("============================================================")

                self._status_pub.publish(String(data=f"A.5 SUCCESS: ID {sid} ({sname}) on Face {face_idx}"))
                self._target_pub.publish(String(data=f"1,{sid}"))
                break

            # Face did not have the target image; orbit to the next face if faces remain
            if face_idx < self._max_faces:
                self.get_logger().info(
                    f"Face {face_idx} has no target image. Orbiting right to Face {face_idx + 1}..."
                )
                macro_ok = self._execute_macro_sync()
                if not macro_ok or not self._is_running:
                    self.get_logger().warn("Orbit macro execution was interrupted.")
                    break

                # Brief pause for chassis and camera to settle before inspecting adjacent face
                time.sleep(0.2)

        if not target_found and self._is_running:
            self.get_logger().warn("============================================================")
            self.get_logger().warn(f"❌ A.5 FINISHED: Checked all {self._max_faces} faces; no target symbol confirmed.")
            self.get_logger().warn("============================================================")
            self._status_pub.publish(String(data="A.5 FINISHED: No symbol confirmed"))

        self._is_running = False


def parse_macro_string(macro_str: str) -> List[Tuple[str, int]]:
    """Parse comma-separated macro string like 'BC:30,FR:90,FC:18,FL:180'."""
    items = []
    for token in macro_str.split(","):
        token = token.strip()
        if not token:
            continue
        parts = token.split(":")
        if len(parts) != 2:
            raise argparse.ArgumentTypeError(f"Invalid macro token '{token}', expected 'CMD:VAL' (e.g. 'FR:90')")
        cmd = parts[0].strip().upper()
        val = int(parts[1].strip())
        items.append((cmd, val))
    return items


def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="SC2079 Checklist A.5: Ultrasonic Approach & Face Recognition")
    parser.add_argument(
        "-d",
        "--distance",
        type=float,
        default=25.0,
        help="Target safe viewing distance from obstacle in cm (default: 25.0)",
    )
    parser.add_argument(
        "-m",
        "--max-faces",
        type=int,
        default=4,
        help="Maximum faces to inspect before terminating (default: 4)",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=15.0,
        help="Maximum forward increment step in cm during approach (default: 15.0)",
    )
    parser.add_argument(
        "--macro",
        type=parse_macro_string,
        default=None,
        help="Custom macro as 'CMD:VAL,CMD:VAL' (default: 'BC:30,FR:90,FC:18,FL:180')",
    )

    parsed_args, ros_args = parser.parse_known_args(args=args or sys.argv[1:])

    rclpy.init(args=ros_args)
    node = ChecklistA5Node(
        target_distance_cm=parsed_args.distance,
        max_faces=parsed_args.max_faces,
        approach_step_cm=parsed_args.step,
        macro=parsed_args.macro,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Start routine in background worker thread
    node.start_routine()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_routine()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
