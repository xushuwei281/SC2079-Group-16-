#!/usr/bin/env python3
"""Closed-loop bullseye heatseek + orbit-recovery node (SC2079 MDP checklist A.5).

Drives the robot toward a detected bullseye marker using its live bounding-box
center offset as a heading-error signal, holds a fixed standoff distance via
ultrasonic feedback, settles, and confirms via /perception/sample_target. If
the confirmed detection is the bullseye sentinel rather than a real target
(11-40), it arcs continuously around the obstacle -- watching the live
detection stream, not hopping between precomputed poses -- until a valid
numbered face is centered, then re-centers and re-confirms on that face.

Exposes /bullseye/orbit_approach (BullseyeOrbit) for planner_node to call as
a first attempt before falling back to its existing discrete-hop
_inspect_adjacent_faces() candidate search (planner_node.py:518-606), which
is left untouched as the safety net if this node fails or times out.
"""

from __future__ import annotations

from enum import Enum
import math
import os
import sys
import threading
import time
from typing import Optional, Tuple

# Locate algorithm directory (same bootstrap planner_node.py uses) so the
# obstacle/robot footprint constants used for orbit-radius geometry stay in
# sync with compute_vantage_pose()'s clearance math rather than being
# re-hardcoded here.
_curr_dir = os.path.dirname(os.path.abspath(__file__))
for _p in [
    os.path.join(_curr_dir, "../../../../algorithm"),
    os.path.join(_curr_dir, "../../../algorithm"),
    os.path.join(_curr_dir, "../../algorithm"),
    os.path.join(os.getcwd(), "algorithm"),
    os.path.join(os.getcwd(), "../algorithm"),
    "/home/mdp/dev/SC2079-Group-16/algorithm",
]:
    _abs_p = os.path.abspath(_p)
    if os.path.isdir(_abs_p) and _abs_p not in sys.path:
        sys.path.insert(0, _abs_p)

try:
    from arena import OBSTACLE_SIZE_CM, ROBOT_H_CM
except ImportError:
    from algorithm.arena import OBSTACLE_SIZE_CM, ROBOT_H_CM

import rclpy
from geometry_msgs.msg import Twist
from mdp_bringup.bullseye_control import (
    compute_orbit_radius_cm,
    distance_p_control,
    heading_p_control,
    orbit_twist,
)
from mdp_interfaces.msg import TrackedTarget, TrackingControl
from mdp_interfaces.srv import BullseyeOrbit, SampleTarget
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty

# Cardinal progression as the robot orbits; used only to produce a best-effort
# "found_face" label for telemetry -- the arc is continuous, not a discrete
# hop between known faces, so this is an estimate derived from cumulative
# heading change, not ground truth.
_CARDINAL_ORDER = ["N", "E", "S", "W"]


class BullseyeState(str, Enum):
    IDLE = "IDLE"
    SEEKING_MARKER = "SEEKING_MARKER"
    CENTERING = "CENTERING"
    HOLDING_DISTANCE = "HOLDING_DISTANCE"
    CONFIRMING = "CONFIRMING"
    ORBITING = "ORBITING"
    DONE = "DONE"
    FAILED = "FAILED"


class BullseyeOrbitNode(Node):
    """Closed-loop bullseye approach + orbit-recovery service node."""

    def __init__(self) -> None:
        super().__init__("bullseye_orbit_node")

        self.declare_parameter("tracking_rate_hz", 10.0)
        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("heading_kp", 1.2)
        self.declare_parameter("distance_kp", 0.015)
        self.declare_parameter("max_yaw_rps", 1.0)
        self.declare_parameter("max_linear_mps", 0.20)
        self.declare_parameter("orbit_tangential_mps", 0.10)
        self.declare_parameter("seek_scan_yaw_rps", 0.3)
        self.declare_parameter("center_tolerance_norm", 0.05)
        self.declare_parameter("orbit_reacquire_tolerance_norm", 0.15)
        self.declare_parameter("hold_settle_s", 0.3)
        self.declare_parameter("seek_timeout_s", 6.0)
        self.declare_parameter("mission_timeout_s", 25.0)
        self.declare_parameter("orbit_max_arc_deg", 300.0)
        self.declare_parameter("min_safe_range_cm", 15.0)
        self.declare_parameter("hard_abort_range_cm", 10.0)

        self._tracking_rate_hz = float(self.get_parameter("tracking_rate_hz").value)
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._heading_kp = float(self.get_parameter("heading_kp").value)
        self._distance_kp = float(self.get_parameter("distance_kp").value)
        self._max_yaw = float(self.get_parameter("max_yaw_rps").value)
        self._max_linear = float(self.get_parameter("max_linear_mps").value)
        self._orbit_speed = float(self.get_parameter("orbit_tangential_mps").value)
        self._seek_scan_yaw = float(self.get_parameter("seek_scan_yaw_rps").value)
        self._center_tol = float(self.get_parameter("center_tolerance_norm").value)
        self._orbit_tol = float(self.get_parameter("orbit_reacquire_tolerance_norm").value)
        self._hold_settle_s = float(self.get_parameter("hold_settle_s").value)
        self._seek_timeout_s = float(self.get_parameter("seek_timeout_s").value)
        self._mission_timeout_s = float(self.get_parameter("mission_timeout_s").value)
        self._orbit_max_arc_rad = math.radians(float(self.get_parameter("orbit_max_arc_deg").value))
        self._min_safe_range_cm = float(self.get_parameter("min_safe_range_cm").value)
        self._hard_abort_range_cm = float(self.get_parameter("hard_abort_range_cm").value)

        self._state = BullseyeState.IDLE
        self._active_lock = threading.Lock()
        self._abort_event = threading.Event()

        self._latest_track: Optional[TrackedTarget] = None
        self._latest_track_time = 0.0
        self._us_range_cm = float("inf")

        callback_group = ReentrantCallbackGroup()

        self._twist_pub = self.create_publisher(Twist, "/cmd_vel/teleop", 10)
        self._track_ctrl_pub = self.create_publisher(TrackingControl, "/perception/tracking_control", 10)

        self._track_sub = self.create_subscription(
            TrackedTarget, "/perception/tracked_target", self._on_track, 10, callback_group=callback_group
        )
        self._us_sub = self.create_subscription(
            Range, "/sensors/ultrasonic", self._on_us_range, 10, callback_group=callback_group
        )
        self._estop_sub = self.create_subscription(
            Empty, "/estop", self._on_estop, 10, callback_group=callback_group
        )

        self._sample_client = self.create_client(
            SampleTarget, "/perception/sample_target", callback_group=callback_group
        )

        self._service = self.create_service(
            BullseyeOrbit, "/bullseye/orbit_approach", self._handle_orbit_approach,
            callback_group=callback_group
        )

        self.get_logger().info("BullseyeOrbitNode ready.")

    # -------------------------------------------------------------------------
    # ROS Callbacks
    # -------------------------------------------------------------------------

    def _on_track(self, msg: TrackedTarget) -> None:
        self._latest_track = msg
        self._latest_track_time = time.time()

    def _on_us_range(self, msg: Range) -> None:
        """Track front ultrasonic range (cm). Mirrors planner_node's own
        _on_us_range validity check against the message's own bounds."""
        if msg.min_range <= msg.range <= msg.max_range:
            self._us_range_cm = msg.range * 100.0
        else:
            self._us_range_cm = float("inf")

    def _on_estop(self, msg: Empty) -> None:
        self.get_logger().warn("BullseyeOrbitNode received E-STOP!")
        self._abort_event.set()

    # -------------------------------------------------------------------------
    # Motion / perception helpers
    # -------------------------------------------------------------------------

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._twist_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_twist(0.0, 0.0)

    def _query_sample_target(self, obstacle_id: int) -> Optional[Tuple[int, str, float, bool]]:
        """Call /perception/sample_target and return (symbol_id, symbol_name,
        confidence, is_marker) or None. Mirrors planner_node's
        _query_perception_sampler (planner_node.py:498-516) -- reuse the same
        service as the authoritative final confirmation rather than trusting
        the streaming single-frame TrackedTarget for the go/no-go decision."""
        if not (self._sample_client.service_is_ready() or self._sample_client.wait_for_service(timeout_sec=0.3)):
            return None
        req = SampleTarget.Request()
        req.obstacle_id = obstacle_id
        future = self._sample_client.call_async(req)
        deadline = time.time() + 3.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if future.done() and future.result() and future.result().success:
            res = future.result()
            return (res.symbol_id, res.symbol_name, res.confidence, res.is_marker)
        return None

    # -------------------------------------------------------------------------
    # Service handler / FSM
    # -------------------------------------------------------------------------

    def _handle_orbit_approach(
        self, request: BullseyeOrbit.Request, response: BullseyeOrbit.Response
    ) -> BullseyeOrbit.Response:
        if not self._active_lock.acquire(blocking=False):
            response.success = False
            response.status = "BUSY"
            return response

        self._abort_event.clear()
        try:
            result = self._run_fsm(request)
        finally:
            self._stop()
            self._track_ctrl_pub.publish(TrackingControl(enable=False, rate_hz=0.0))
            self._state = BullseyeState.IDLE
            self._active_lock.release()

        response.success = result["success"]
        response.symbol_id = result["symbol_id"]
        response.symbol_name = result["symbol_name"]
        response.confidence = result["confidence"]
        response.found_face = result["found_face"]
        response.status = result["status"]
        return response

    def _run_fsm(self, request: BullseyeOrbit.Request) -> dict:
        obstacle_half = OBSTACLE_SIZE_CM / 2.0
        robot_half = ROBOT_H_CM / 2.0
        clearance = request.target_clearance_cm if request.target_clearance_cm > 0.0 else 25.0
        orbit_radius_cm = compute_orbit_radius_cm(clearance, obstacle_half, robot_half)
        nominal_face = (request.nominal_face or "N").upper()
        if nominal_face not in _CARDINAL_ORDER:
            nominal_face = "N"

        self.get_logger().info(
            f"🎯 BullseyeOrbit: Obs {request.obstacle_id} at ({request.obstacle_x_cm:.1f},"
            f"{request.obstacle_y_cm:.1f}), nominal {nominal_face}, clearance {clearance:.1f}cm, "
            f"orbit radius {orbit_radius_cm:.1f}cm"
        )

        self._track_ctrl_pub.publish(TrackingControl(enable=True, rate_hz=self._tracking_rate_hz))
        self._state = BullseyeState.SEEKING_MARKER

        dt = 1.0 / self._control_rate_hz
        found_face = nominal_face
        cumulative_arc_rad = 0.0
        seek_deadline = time.time() + self._seek_timeout_s
        mission_deadline = time.time() + self._mission_timeout_s
        center_hold_start: Optional[float] = None
        distance_hold_start: Optional[float] = None
        backoff_hold_start: Optional[float] = None

        result = {
            "success": False, "symbol_id": 0, "symbol_name": "", "confidence": 0.0,
            "found_face": found_face, "status": "TIMEOUT",
        }

        while rclpy.ok() and not self._abort_event.is_set():
            if time.time() > mission_deadline:
                result["status"] = "TIMEOUT"
                return result

            # Own conservative abort, ahead of motion_controller's hard
            # E-stop latch (front_stop_distance_m=0.12m / ir_stop_distance_m
            # =0.10m) -- this must trip first so the mission can retreat to
            # the discrete-hop fallback without requiring an operator RESET.
            if self._us_range_cm < self._hard_abort_range_cm:
                result["status"] = "SAFETY_ABORT"
                return result
            back_off = self._us_range_cm < self._min_safe_range_cm
            if back_off:
                backoff_hold_start = backoff_hold_start or time.time()
                if time.time() - backoff_hold_start > 2.0:
                    result["status"] = "SAFETY_ABORT"
                    return result
            else:
                backoff_hold_start = None

            track = self._latest_track
            track_fresh = track is not None and (time.time() - self._latest_track_time) < 0.5
            result["found_face"] = found_face

            if self._state == BullseyeState.SEEKING_MARKER:
                if time.time() > seek_deadline:
                    result["status"] = "LOST_TARGET"
                    return result
                if track_fresh and track.valid:
                    self._state = BullseyeState.CENTERING
                    center_hold_start = None
                else:
                    self._publish_twist(0.0, self._seek_scan_yaw)

            elif self._state == BullseyeState.CENTERING:
                if not track_fresh or not track.valid:
                    self._state = BullseyeState.SEEKING_MARKER
                    seek_deadline = time.time() + self._seek_timeout_s
                else:
                    yaw = 0.0 if back_off else heading_p_control(track.cx_norm, self._heading_kp, self._max_yaw)
                    self._publish_twist(0.0, yaw)
                    if abs(track.cx_norm) < self._center_tol:
                        center_hold_start = center_hold_start or time.time()
                        if time.time() - center_hold_start >= 0.15:
                            self._state = BullseyeState.HOLDING_DISTANCE
                            distance_hold_start = None
                    else:
                        center_hold_start = None

            elif self._state == BullseyeState.HOLDING_DISTANCE:
                if not track_fresh or not track.valid:
                    self._state = BullseyeState.SEEKING_MARKER
                    seek_deadline = time.time() + self._seek_timeout_s
                else:
                    range_cm = self._us_range_cm if not math.isinf(self._us_range_cm) else clearance
                    lin = 0.0 if back_off else distance_p_control(range_cm, clearance, self._distance_kp, self._max_linear)
                    yaw = 0.0 if back_off else heading_p_control(track.cx_norm, self._heading_kp, self._max_yaw)
                    self._publish_twist(lin, yaw)
                    on_target = abs(range_cm - clearance) < 2.0 and abs(track.cx_norm) < self._center_tol
                    if on_target:
                        distance_hold_start = distance_hold_start or time.time()
                        if time.time() - distance_hold_start >= self._hold_settle_s:
                            self._stop()
                            self._state = BullseyeState.CONFIRMING
                    else:
                        distance_hold_start = None

            elif self._state == BullseyeState.CONFIRMING:
                sample = self._query_sample_target(request.obstacle_id)
                if sample is None:
                    result["status"] = "LOST_TARGET"
                    return result
                sid, sname, conf, is_marker = sample
                if not is_marker and 11 <= sid <= 40:
                    result.update(
                        success=True, symbol_id=int(sid), symbol_name=str(sname),
                        confidence=float(conf), found_face=found_face, status="OK",
                    )
                    return result
                self._state = BullseyeState.ORBITING

            elif self._state == BullseyeState.ORBITING:
                if cumulative_arc_rad >= self._orbit_max_arc_rad:
                    result["status"] = "NO_VALID_FACE"
                    return result
                if track_fresh and track.valid and not track.is_marker and abs(track.cx_norm) < self._orbit_tol:
                    steps = int(round(cumulative_arc_rad / (math.pi / 2.0))) % len(_CARDINAL_ORDER)
                    start_idx = _CARDINAL_ORDER.index(nominal_face)
                    found_face = _CARDINAL_ORDER[(start_idx + steps) % len(_CARDINAL_ORDER)]
                    self._state = BullseyeState.CENTERING
                    center_hold_start = None
                else:
                    lin, ang = (0.0, 0.0) if back_off else orbit_twist(orbit_radius_cm, self._orbit_speed)
                    self._publish_twist(lin, ang)
                    cumulative_arc_rad += abs(ang) * dt

            time.sleep(dt)

        result["status"] = "SAFETY_ABORT"
        return result


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BullseyeOrbitNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=2.0)
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
