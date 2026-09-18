#!/usr/bin/env python3
"""Unit tests for BullseyeOrbitNode (SC2079 MDP closed-loop bullseye orbit)."""

import threading
import time
import unittest
from unittest.mock import MagicMock

from mdp_bringup.bullseye_orbit_node import BullseyeOrbitNode, BullseyeState
from mdp_interfaces.msg import TrackedTarget
from mdp_interfaces.srv import BullseyeOrbit
import rclpy
from sensor_msgs.msg import Range


class TestBullseyeOrbitNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BullseyeOrbitNode()
        # Speed up hold/settle/timeout windows for fast, deterministic tests.
        self.node._hold_settle_s = 0.02
        self.node._seek_timeout_s = 2.0
        self.node._mission_timeout_s = 3.0
        self.node._twist_pub.publish = MagicMock()
        self.node._track_ctrl_pub.publish = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def _make_request(self, nominal_face="N", clearance_cm=25.0):
        req = BullseyeOrbit.Request()
        req.obstacle_id = 1
        req.obstacle_x_cm = 60.0
        req.obstacle_y_cm = 60.0
        req.nominal_face = nominal_face
        req.target_clearance_cm = clearance_cm
        return req

    def _make_track(self, valid=True, is_marker=False, cx_norm=0.0, symbol_id=0):
        t = TrackedTarget()
        t.valid = valid
        t.is_marker = is_marker
        t.cx_norm = cx_norm
        t.cy_norm = 0.0
        t.symbol_id = symbol_id
        t.confidence = 0.9
        return t

    def test_initial_state(self):
        self.assertEqual(self.node._state, BullseyeState.IDLE)

    def test_safety_threshold_ordering(self):
        """The soft back-off distance must be strictly above both our own hard
        abort and motion_controller's independent front_stop_distance_m
        (0.12m/12cm, motion_controller_node.py) so this node's own back-off
        (commands zeroed) engages before motion_controller's hard E-stop
        latch would ever see a close-range forward command."""
        self.assertLess(self.node._hard_abort_range_cm, self.node._min_safe_range_cm)
        self.assertGreater(self.node._min_safe_range_cm, 12.0)

    def test_on_track_updates_latest(self):
        track = self._make_track(valid=True, cx_norm=0.2)
        self.node._on_track(track)
        self.assertIs(self.node._latest_track, track)
        self.assertGreater(self.node._latest_track_time, 0.0)

    def test_on_us_range_validity(self):
        msg = Range()
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = 0.20
        self.node._on_us_range(msg)
        self.assertAlmostEqual(self.node._us_range_cm, 20.0, places=3)

        msg.range = 10.0  # out of [min_range, max_range]
        self.node._on_us_range(msg)
        self.assertTrue(self.node._us_range_cm == float("inf"))

    def test_busy_rejects_concurrent_call(self):
        acquired = self.node._active_lock.acquire(blocking=False)
        self.assertTrue(acquired)
        try:
            response = BullseyeOrbit.Response()
            result = self.node._handle_orbit_approach(self._make_request(), response)
            self.assertFalse(result.success)
            self.assertEqual(result.status, "BUSY")
        finally:
            self.node._active_lock.release()

    def test_safety_abort_below_hard_threshold(self):
        self.node._us_range_cm = self.node._hard_abort_range_cm - 0.5
        result = self.node._run_fsm(self._make_request())
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "SAFETY_ABORT")

    def _run_with_live_track(self, request, track_factory, query_sample_side_effect, timeout_s=3.0):
        """Run _run_fsm in the calling thread while a background thread keeps
        _latest_track fresh (the FSM treats a track older than 0.5s as
        stale), mirroring how a live perception stream would keep publishing."""
        self.node._us_range_cm = request.target_clearance_cm or 25.0
        self.node._query_sample_target = MagicMock(side_effect=query_sample_side_effect)

        stop_refresh = threading.Event()

        def _refresh():
            while not stop_refresh.is_set():
                self.node._latest_track = track_factory(self.node._state)
                self.node._latest_track_time = time.time()
                time.sleep(0.02)

        t = threading.Thread(target=_refresh, daemon=True)
        t.start()
        try:
            result = self.node._run_fsm(request)
        finally:
            stop_refresh.set()
            t.join(timeout=1.0)
        return result

    def test_confirms_real_symbol_on_nominal_face(self):
        """Centering + distance-hold gating both actually elapse (not
        skipped), then CONFIRMING sees a real symbol -> DONE."""
        result = self._run_with_live_track(
            self._make_request(nominal_face="N"),
            track_factory=lambda state: self._make_track(valid=True, cx_norm=0.0),
            query_sample_side_effect=[(15, "3", 0.95, False)],
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["symbol_id"], 15)
        self.assertEqual(result["found_face"], "N")

    def test_orbits_past_marker_to_confirm_adjacent_face(self):
        """First CONFIRMING call sees the bullseye marker again -> ORBITING;
        once a non-marker detection is reacquired, re-centers and confirms."""

        def track_factory(state):
            if state == BullseyeState.ORBITING:
                return self._make_track(valid=True, is_marker=False, cx_norm=0.05)
            return self._make_track(valid=True, is_marker=True, cx_norm=0.0)

        result = self._run_with_live_track(
            self._make_request(nominal_face="N"),
            track_factory=track_factory,
            query_sample_side_effect=[(0, "bullseye", 0.9, True), (21, "b", 0.9, False)],
            timeout_s=4.0,
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["symbol_id"], 21)
        self.assertEqual(self.node._query_sample_target.call_count, 2)

    def test_lost_target_times_out_seeking(self):
        self.node._seek_timeout_s = 0.05
        self.node._latest_track = None
        result = self.node._run_fsm(self._make_request())
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "LOST_TARGET")


if __name__ == "__main__":
    unittest.main()
