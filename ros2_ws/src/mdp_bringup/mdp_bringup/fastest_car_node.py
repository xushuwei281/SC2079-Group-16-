#!/usr/bin/env python3
"""Task 2: Fastest Car Task using Visual Recognition (SC2079 MDP).

A reactive, speed-optimized autonomous sprint node that:
1. Sprints forward from the Carpark towards Obstacle 1.
2. Stops at vantage distance using ultrasonic sensor feedback.
3. Detects Left Arrow (Symbol 39) vs Right Arrow (Symbol 38) via /perception/sample_target.
4. Executes a calibrated slalom bypass around Obstacle 1 in the indicated direction.
5. Sprints forward towards Obstacle 2 and detects Arrow 2.
6. Slaloms around Obstacle 2, loops around to face South, and sprints back to the Carpark.
7. Reports elapsed sprint time and completion status to the Android tablet.
"""

from enum import Enum
import math
import threading
import time
from typing import List, Optional, Tuple

from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String


class Task2State(str, Enum):
    IDLE = "IDLE"
    APPROACH_OBS1 = "APPROACH_OBS1"
    DETECT_ARROW1 = "DETECT_ARROW1"
    SLALOM_OBS1 = "SLALOM_OBS1"
    APPROACH_OBS2 = "APPROACH_OBS2"
    DETECT_ARROW2 = "DETECT_ARROW2"
    SLALOM_OBS2_AND_RETURN = "SLALOM_OBS2_AND_RETURN"
    PARK = "PARK"
    COMPLETE = "COMPLETE"
    ESTOP = "ESTOP"


def parse_move_command(cmd_str: str) -> MoveCommand:
    """Parse a command string like 'FL045' or 'FC030' into a MoveCommand."""
    cmd_str = cmd_str.strip()
    code = cmd_str[:2].upper()
    val = int(cmd_str[2:])
    mc = MoveCommand()
    mc.command = code
    mc.value = val
    return mc


class FastestCarNode(Node):
    """Task 2 Fastest Car Reactive Sprint Node."""

    def __init__(self) -> None:
        super().__init__("fastest_car_node")

        # Configurable sprint and maneuver parameters
        self.declare_parameter("vantage_dist_cm", 30.0)
        self.declare_parameter("approach_step_cm", 25.0)
        self.declare_parameter("min_approach_step_cm", 5.0)
        self.declare_parameter("max_approach_steps", 8)
        self.declare_parameter("fallback_approach_cm", 60.0)
        self.declare_parameter(
            "slalom_left_cmds", ["FL045", "FC030", "FR090", "FC030", "FL045"]
        )
        self.declare_parameter(
            "slalom_right_cmds", ["FR045", "FC030", "FL090", "FC030", "FR045"]
        )
        self.declare_parameter(
            "return_left_cmds",
            ["FL045", "FC030", "FR090", "FC020", "FR090", "FC030", "FR045"],
        )
        self.declare_parameter(
            "return_right_cmds",
            ["FR045", "FC030", "FL090", "FC020", "FL090", "FC030", "FL045"],
        )
        self.declare_parameter("return_straight_cm", 80.0)
        self.declare_parameter("default_turn", "LEFT")
        self.declare_parameter("sample_retries", 3)

        self._vantage_dist_cm = float(self.get_parameter("vantage_dist_cm").value)
        self._approach_step_cm = float(self.get_parameter("approach_step_cm").value)
        self._min_approach_step_cm = float(
            self.get_parameter("min_approach_step_cm").value
        )
        self._max_approach_steps = int(self.get_parameter("max_approach_steps").value)
        self._fallback_approach_cm = float(
            self.get_parameter("fallback_approach_cm").value
        )
        self._slalom_left_cmds = list(self.get_parameter("slalom_left_cmds").value)
        self._slalom_right_cmds = list(self.get_parameter("slalom_right_cmds").value)
        self._return_left_cmds = list(self.get_parameter("return_left_cmds").value)
        self._return_right_cmds = list(self.get_parameter("return_right_cmds").value)
        self._return_straight_cm = float(self.get_parameter("return_straight_cm").value)
        self._default_turn = str(self.get_parameter("default_turn").value).upper()
        self._sample_retries = int(self.get_parameter("sample_retries").value)

        self._state = Task2State.IDLE
        self._is_running = False
        self._sprint_thread: Optional[threading.Thread] = None

        self._us_range_m = float("inf")
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0

        callback_group = ReentrantCallbackGroup()

        # Publishers
        self._status_pub = self.create_publisher(String, "/android/status", 10)
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        self._estop_pub = self.create_publisher(Empty, "/estop", 10)

        # Subscribers
        self._cmd_sub = self.create_subscription(
            String, "/android/cmd", self._on_cmd, 10, callback_group=callback_group
        )
        self._us_sub = self.create_subscription(
            Range,
            "/sensors/ultrasonic",
            self._on_us_range,
            10,
            callback_group=callback_group,
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._on_pose, 10, callback_group=callback_group
        )
        self._estop_sub = self.create_subscription(
            Empty, "/estop", self._on_estop, 10, callback_group=callback_group
        )

        # Service Clients
        self._move_client = self.create_client(
            ExecuteMoves, "/execute_moves", callback_group=callback_group
        )
        self._sample_client = self.create_client(
            SampleTarget, "/perception/sample_target", callback_group=callback_group
        )

        self.get_logger().info(
            f"FastestCarNode initialized. Vantage dist: {self._vantage_dist_cm:.1f} cm, Default: {self._default_turn}"
        )

    # -------------------------------------------------------------------------
    # ROS Callbacks
    # -------------------------------------------------------------------------

    def _on_us_range(self, msg: Range) -> None:
        """Update live ultrasonic reading (m)."""
        if 0.02 <= msg.range <= 4.0:
            self._us_range_m = float(msg.range)
        else:
            self._us_range_m = float("inf")

    def _on_pose(self, msg: PoseStamped) -> None:
        """Update live dead-reckoned pose."""
        self._current_x = msg.pose.position.x * 100.0
        self._current_y = msg.pose.position.y * 100.0
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_estop(self, msg: Empty) -> None:
        """Emergency stop handler."""
        self.get_logger().warn("FastestCarNode received E-STOP!")
        self._transition(Task2State.ESTOP, "E-STOP received")
        self._is_running = False

    def _on_cmd(self, msg: String) -> None:
        """Handle tablet / CLI triggers for Task 2."""
        raw = msg.data.strip().upper()
        self.get_logger().info(f"FastestCar received cmd: {raw}")

        if raw in ("STM:SP", "SP", "START_TASK2", "TASK2", "START_TASK_2", "FASTEST"):
            self.start_sprint()
        elif raw in ("RESET", "ALG:RESET", "STOP"):
            self.reset_sprint()

    def _transition(self, new_state: Task2State, reason: str = "") -> None:
        old_state = self._state
        self._state = new_state
        msg = f"Task 2 FSM: {old_state.value} -> {new_state.value}"
        if reason:
            msg += f" ({reason})"
        self.get_logger().info(msg)

    # -------------------------------------------------------------------------
    # Sprint Control & Worker Thread
    # -------------------------------------------------------------------------

    def start_sprint(self) -> None:
        """Trigger the fastest car sprint."""
        if self._is_running:
            self.get_logger().warn("Task 2 sprint already running.")
            return

        self._is_running = True
        self._sprint_thread = threading.Thread(
            target=self._run_task2_sprint, daemon=True
        )
        self._sprint_thread.start()

    def reset_sprint(self) -> None:
        """Reset sprint state."""
        self._is_running = False
        self._transition(Task2State.IDLE, "User reset")
        self._status_pub.publish(String(data="Task 2 Reset"))

    def _execute_move_list_sync(
        self, cmd_strings: List[str], label: str = ""
    ) -> bool:
        """Execute a list of move command strings (e.g. ['FL045', 'FC030'])."""
        if not self._is_running:
            return False

        req = ExecuteMoves.Request()
        for s in cmd_strings:
            req.commands.append(parse_move_command(s))

        future = self._move_client.call_async(req)
        while rclpy.ok() and not future.done():
            if not self._is_running:
                return False
            time.sleep(0.02)

        if future.done() and future.result() and future.result().success:
            return True
        return False

    def _advance_to_obstacle(self, obs_num: int) -> bool:
        """Advance towards obstacle using ultrasonic sensor until within vantage distance."""
        self.get_logger().info(f"Advancing to Obstacle {obs_num}...")
        self._status_pub.publish(
            String(data=f"Sprinting to Obstacle {obs_num}...")
        )

        for step_idx in range(self._max_approach_steps):
            if not self._is_running:
                return False

            us_dist_cm = self._us_range_m * 100.0

            # If ultrasonic has valid reading and is already within vantage distance (+ margin)
            if us_dist_cm <= (self._vantage_dist_cm + 3.0):
                self.get_logger().info(
                    f"Obstacle {obs_num} reached at {us_dist_cm:.1f} cm (<= {self._vantage_dist_cm:.1f} cm)."
                )
                return True

            if math.isinf(us_dist_cm):
                # Sensor reading unavailable or beyond max range; use calibrated step
                dist_to_move = self._approach_step_cm
                self.get_logger().info(
                    f"US reading inf; advancing {dist_to_move:.0f} cm."
                )
            else:
                gap = us_dist_cm - self._vantage_dist_cm
                dist_to_move = max(
                    self._min_approach_step_cm, min(self._approach_step_cm, gap)
                )
                self.get_logger().info(
                    f"US: {us_dist_cm:.1f} cm -> Gap: {gap:.1f} cm -> Advancing {dist_to_move:.0f} cm."
                )

            move_ok = self._execute_move_list_sync(
                [f"FC{int(dist_to_move):03d}"],
                label=f"Approach Obs {obs_num} step {step_idx+1}",
            )
            if not move_ok:
                return False

            time.sleep(0.1)

        # Finished loop steps
        return True

    def _sample_arrow(self, obs_num: int) -> Tuple[str, int, float]:
        """Sample target detection to determine Left vs Right Arrow.

        Returns (direction, symbol_id, confidence) where direction is 'LEFT' or 'RIGHT'.
        """
        self._status_pub.publish(
            String(data=f"Scanning Obstacle {obs_num} Arrow...")
        )

        for attempt in range(self._sample_retries):
            if not self._is_running:
                break

            if not (
                self._sample_client.service_is_ready()
                or self._sample_client.wait_for_service(timeout_sec=0.4)
            ):
                time.sleep(0.15)
                continue

            req = SampleTarget.Request()
            req.obstacle_id = obs_num
            future = self._sample_client.call_async(req)

            start_t = time.time()
            while rclpy.ok() and not future.done() and (time.time() - start_t < 0.6):
                time.sleep(0.02)

            if future.done() and future.result():
                res: SampleTarget.Response = future.result()
                if res.success:
                    sid = res.symbol_id
                    sname = (res.symbol_name or "").lower()
                    conf = res.confidence

                    # Symbol 39 = Left Arrow, Symbol 38 = Right Arrow
                    if sid == 39 or "left" in sname:
                        self.get_logger().info(
                            f"Obstacle {obs_num} Arrow: LEFT (Symbol {sid}, Conf: {conf:.2f})"
                        )
                        return ("LEFT", sid, conf)
                    elif sid == 38 or "right" in sname:
                        self.get_logger().info(
                            f"Obstacle {obs_num} Arrow: RIGHT (Symbol {sid}, Conf: {conf:.2f})"
                        )
                        return ("RIGHT", sid, conf)

            time.sleep(0.15)

        # Fallback if perception did not return a decisive arrow
        fallback_dir = self._default_turn
        fallback_id = 39 if fallback_dir == "LEFT" else 38
        self.get_logger().warn(
            f"Perception uncertain for Obs {obs_num}. Defaulting to {fallback_dir}."
        )
        return (fallback_dir, fallback_id, 0.50)

    def _run_task2_sprint(self) -> None:
        """Main Task 2 execution procedure."""
        start_time = time.time()
        self.get_logger().info("🏎️  STARTING TASK 2: FASTEST CAR SPRINT 🏎️")
        self._status_pub.publish(String(data="Task 2 Sprint Started!"))

        if not self._move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Hardware bridge /execute_moves unavailable.")
            self._status_pub.publish(String(data="Hardware bridge offline"))
            self._transition(Task2State.IDLE, "Bridge offline")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 1. Approach Obstacle 1
        # ---------------------------------------------------------------------
        self._transition(Task2State.APPROACH_OBS1, "Approaching Obstacle 1")
        if not self._advance_to_obstacle(obs_num=1):
            self.get_logger().warn("Approach to Obstacle 1 aborted.")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 2. Detect Arrow on Obstacle 1
        # ---------------------------------------------------------------------
        self._transition(Task2State.DETECT_ARROW1, "Scanning Arrow 1")
        dir1, sid1, conf1 = self._sample_arrow(obs_num=1)
        self._status_pub.publish(
            String(data=f"Obs 1: Arrow {dir1} (Conf: {conf1:.2f})")
        )
        self._target_pub.publish(String(data=f"1,{sid1}"))

        # ---------------------------------------------------------------------
        # 3. Slalom Bypass Obstacle 1
        # ---------------------------------------------------------------------
        self._transition(Task2State.SLALOM_OBS1, f"Slalom Obs 1 ({dir1})")
        self._status_pub.publish(String(data=f"Bypassing Obs 1 ({dir1})..."))
        cmds1 = self._slalom_left_cmds if dir1 == "LEFT" else self._slalom_right_cmds
        if not self._execute_move_list_sync(cmds1, label="Slalom Obs 1"):
            self.get_logger().warn("Slalom Obs 1 aborted.")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 4. Approach Obstacle 2
        # ---------------------------------------------------------------------
        self._transition(Task2State.APPROACH_OBS2, "Approaching Obstacle 2")
        if not self._advance_to_obstacle(obs_num=2):
            self.get_logger().warn("Approach to Obstacle 2 aborted.")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 5. Detect Arrow on Obstacle 2
        # ---------------------------------------------------------------------
        self._transition(Task2State.DETECT_ARROW2, "Scanning Arrow 2")
        dir2, sid2, conf2 = self._sample_arrow(obs_num=2)
        self._status_pub.publish(
            String(data=f"Obs 2: Arrow {dir2} (Conf: {conf2:.2f})")
        )
        self._target_pub.publish(String(data=f"2,{sid2}"))

        # ---------------------------------------------------------------------
        # 6. Slalom Bypass Obstacle 2 and Return Loop
        # ---------------------------------------------------------------------
        self._transition(
            Task2State.SLALOM_OBS2_AND_RETURN, f"Rounding Obs 2 ({dir2}) & Loop"
        )
        self._status_pub.publish(
            String(data=f"Rounding Obs 2 ({dir2}) and Returning...")
        )
        cmds2 = self._return_left_cmds if dir2 == "LEFT" else self._return_right_cmds
        if not self._execute_move_list_sync(cmds2, label="Round Obs 2 & Return"):
            self.get_logger().warn("Round Obs 2 aborted.")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 7. Sprint Back to Carpark
        # ---------------------------------------------------------------------
        self._transition(Task2State.PARK, "Returning to Carpark")
        self._status_pub.publish(String(data="Sprinting back to Carpark..."))
        park_cmd = f"FC{int(self._return_straight_cm):03d}"
        if not self._execute_move_list_sync([park_cmd], label="Park Sprint"):
            self.get_logger().warn("Park sprint aborted.")
            self._is_running = False
            return

        # ---------------------------------------------------------------------
        # 8. Complete!
        # ---------------------------------------------------------------------
        elapsed = time.time() - start_time
        self._transition(
            Task2State.COMPLETE, f"Task 2 Completed in {elapsed:.1f}s"
        )
        self.get_logger().info(
            f"🏁🏁🏁 TASK 2 SPRINT COMPLETED IN {elapsed:.2f} SECONDS! 🏁🏁🏁"
        )
        self._status_pub.publish(
            String(data=f"Task 2 Complete! Time: {elapsed:.1f}s")
        )

        self._is_running = False
        self._transition(Task2State.IDLE, "Mission complete, standing by")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FastestCarNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
