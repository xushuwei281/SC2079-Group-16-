"""Offline tests of measured movement and velocity source arbitration."""
import math
import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

from mdp_hardware_bridge.motion_controller_node import MotionControllerNode
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves


class TestMotionController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MotionControllerNode()
        self.node._timer.cancel()
        self.node._velocity_pub = MagicMock()
        self.node._estop_pub = MagicMock()
        self.feedback()

    def tearDown(self):
        self.node.destroy_node()

    def feedback(self, x=0.2, y=0.2, yaw=math.pi / 2, us=0.5):
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node._on_pose(pose)
        sensor = Range()
        sensor.header.stamp = pose.header.stamp
        sensor.range = us
        self.node._on_range(sensor)

    def execute(self, command="FC", value=10):
        request = ExecuteMoves.Request(commands=[MoveCommand(command=command, value=value)])
        return self.node._handle_execute_moves(request, ExecuteMoves.Response())

    def velocities(self):
        return [(c.args[0].linear.x, c.args[0].angular.z)
                for c in self.node._velocity_pub.publish.call_args_list]

    def test_measured_forward_and_reverse_without_fin(self):
        for cmd, y, sign in (("FC", 0.3, 1), ("BC", 0.1, -1)):
            self.feedback()
            self.node._velocity_pub.reset_mock()
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _, end=y: self.feedback(y=end)):
                self.assertTrue(self.execute(cmd).success)
            self.assertGreater(self.velocities()[0][0] * sign, 0)
            self.assertEqual(self.velocities()[-1], (0, 0))

    def test_straight_drive_corrects_heading_drift(self):
        # 1. Test drift to the right: yaw drops below start heading (clockwise) -> must steer left (yaw_rate > 0)
        self.feedback(x=0.0, y=0.0, yaw=math.pi / 2)
        self.node._velocity_pub.reset_mock()
        steps_right = [
            (0.05, math.pi / 2 - math.radians(4.0)),
            (0.10, math.pi / 2 - math.radians(4.0)),
        ]
        def sim_drift_right(_):
            y, yaw = steps_right.pop(0) if steps_right else (0.10, math.pi / 2 - math.radians(4.0))
            self.feedback(x=0.0, y=y, yaw=yaw)

        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=sim_drift_right):
            res = self.execute("FC", 10)
            self.assertTrue(res.success)
        vels = self.velocities()
        self.assertGreater(len(vels), 2)
        self.assertGreater(vels[1][1], 0.0, "Drift to right must produce positive yaw_rate to steer left")

        # 2. Test drift to the left: yaw rises above start heading (counter-clockwise) -> must steer right (yaw_rate < 0)
        self.feedback(x=0.0, y=0.0, yaw=math.pi / 2)
        self.node._velocity_pub.reset_mock()
        steps_left = [
            (0.05, math.pi / 2 + math.radians(4.0)),
            (0.10, math.pi / 2 + math.radians(4.0)),
        ]
        def sim_drift_left(_):
            y, yaw = steps_left.pop(0) if steps_left else (0.10, math.pi / 2 + math.radians(4.0))
            self.feedback(x=0.0, y=y, yaw=yaw)

        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=sim_drift_left):
            res = self.execute("FC", 10)
            self.assertTrue(res.success)
        vels = self.velocities()
        self.assertGreater(len(vels), 2)
        self.assertLess(vels[1][1], 0.0, "Drift to left must produce negative yaw_rate to steer right")

    def test_all_turn_directions_use_measured_heading(self):
        for cmd, delta, sign in (("FL", 1, 1), ("FR", -1, 1),
                                 ("BL", -1, -1), ("BR", 1, -1)):
            self.feedback()
            self.node._velocity_pub.reset_mock()
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _, d=delta: self.feedback(yaw=math.pi / 2 + d)):
                self.assertTrue(self.execute(cmd, 57).success)
            speed, yaw = self.velocities()[0]
            self.assertGreater(speed * sign, 0)
            self.assertGreater(yaw * delta, 0)
            self.assertAlmostEqual(abs(speed / yaw), 0.21)

    def test_turn_across_wrap(self):
        self.feedback(yaw=math.radians(179))
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.feedback(yaw=math.radians(-171))):
            self.assertTrue(self.execute("FL", 10).success)

    def test_transient_pose_staleness_does_not_stop(self):
        """A momentarily stale pose sample (e.g. an overloaded Pi missing one
        telemetry tick) must not latch an E-stop -- report-only, see
        motion_controller_node._motion_error."""
        self.node._telemetry_stamp -= 1
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.feedback(y=0.3)):
            result = self.execute()
        self.assertEqual(result.status, "FIN")
        self.assertTrue(result.success)
        self.assertFalse(self.node._estop_event.is_set())

    def test_sustained_feedback_loss_still_stops(self):
        """If pose feedback genuinely stops updating for good (not just a
        transient staleness blip an overloaded Pi can cause), the robot must
        still safely stop -- via the range-freshness check in
        _forward_safety_error, since pose staleness alone being report-only
        must not let it drive forever with no feedback at all."""
        def lose(_):
            self.node._telemetry_stamp -= 1
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=lose):
            result = self.execute()
        self.assertEqual(result.status, "SENSOR_STALE")
        self.assertTrue(self.node._estop_event.is_set())
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_mid_move_estop_stays_latched(self):
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.node._on_estop(Empty())):
            self.assertFalse(self.execute().success)
        self.node._on_android_cmd(String(data="ALG|1,2,3,N"))
        self.assertFalse(self.execute().success)
        self.assertTrue(self.node._estop_event.is_set())

    def test_external_estop_is_not_republished(self):
        self.node._on_estop(Empty())
        self.assertTrue(self.node._estop_event.is_set())
        self.node._estop_pub.publish.assert_not_called()
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_reset_cancels_active_generation(self):
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.node._on_android_cmd(String(data="RESET"))):
            self.assertEqual(self.execute().status, "CANCELLED")
        self.assertFalse(self.node._busy.is_set())
        self.assertFalse(self.node._telemetry_fresh())

    def test_teleop_expiration_zero_and_no_replay(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.node._teleop_stamp -= 1
        self.node._teleop_tick()
        self.assertEqual(self.velocities()[-1], (0, 0))
        count = len(self.velocities())
        self.feedback()
        self.node._teleop_tick()
        self.assertEqual(len(self.velocities()), count)

    def test_first_post_reset_teleop_holds_zero_until_feedback(self):
        self.node._on_android_cmd(String(data="RESET"))
        self.node._velocity_pub.reset_mock()
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.node._teleop_tick()
        self.assertEqual(self.velocities(), [(0, 0)])
        self.assertFalse(self.node._estop_event.is_set())

        self.feedback()
        self.node._teleop_tick()
        self.assertEqual(self.velocities()[-1], (0.1, 0))

    def test_teleop_continuously_stops_for_proximity(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.node._teleop_tick()
        self.assertGreater(self.velocities()[-1][0], 0)
        # This node trusts the shared Kalman-filtered range topic (see
        # kalman_filter.py / test_kalman_filter.py for spike rejection);
        # it does not re-filter, so one close filtered reading stops it.
        self.feedback(us=0.12)
        self.node._teleop_tick()
        self.assertTrue(self.node._estop_event.is_set())
        self.assertEqual(self.node._stop_reason, "PROXIMITY:ULTRASONIC")
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_zero_no_echo_is_not_a_proximity_fault(self):
        self.feedback(us=0.0)
        self.assertEqual(self.node._forward_safety_error(True), "")

    def test_execute_moves_primitive_stops_for_proximity(self):
        """The /execute_moves path shares _forward_safety_error with teleop."""
        self.feedback(us=0.08)
        response = self.execute("FC", 50)
        self.assertFalse(response.success)
        self.assertEqual(response.status, "PROXIMITY:ULTRASONIC")
        self.assertTrue(self.node._estop_event.is_set())

    def test_service_owns_output_and_drops_teleop_during_motion(self):
        msg = Twist()
        msg.linear.x = -0.3
        def arrive(_):
            self.node._on_teleop(msg)
            self.feedback(y=0.3)
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=arrive):
            self.assertTrue(self.execute().success)
        self.assertFalse(any(speed < 0 for speed, yaw in self.velocities()))
        self.assertIsNone(self.node._teleop_target)

    def test_active_teleop_rejects_service_and_zero_releases(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.assertEqual(self.execute().status, "BUSY_LOCAL")
        self.node._on_teleop(Twist())
        self.assertIsNone(self.node._teleop_target)

    def test_invalid_teleop_stops(self):
        for speed, yaw in ((math.nan, 0), (0, 1), (0.36, 0), (0.1, 1)):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(speed), float(yaw)
            self.node._on_teleop(msg)
            self.assertEqual(self.velocities()[-1], (0, 0))

    def test_fu_bu_live_range_and_invalid_target(self):
        for cmd, initial in (("FU", 0.5), ("BU", 0.15)):
            self.feedback(us=initial)
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _: self.feedback(us=0.2)):
                self.assertTrue(self.execute(cmd, 20).success)
        self.assertFalse(self.execute("FU", 8).success)

    def test_stale_range_cannot_complete_ultrasonic_move(self):
        self.node._range_stamp -= 1
        self.assertEqual(self.execute("FU", 20).status, "INVALID_RANGE")

    def test_old_pose_header_cannot_refresh_watchdog(self):
        stamp = self.node._telemetry_stamp
        self.node._on_pose(PoseStamped())
        self.assertEqual(self.node._telemetry_stamp, stamp)

    def test_maintenance_proxy_and_reject_mixed(self):
        client = MagicMock()
        client.service_is_ready.return_value = True
        future = client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = ExecuteMoves.Response(success=True, status="FIN")
        self.node._maintenance_client = client
        self.assertTrue(self.execute("GC", 0).success)
        self.assertTrue(all(v == (0, 0) for v in self.velocities()))
        req = ExecuteMoves.Request(commands=[MoveCommand(command="GC", value=0),
                                             MoveCommand(command="FC", value=10)])
        self.assertFalse(self.node._handle_execute_moves(req, ExecuteMoves.Response()).success)

    def test_turn_180_runout(self):
        self.feedback(x=0.2, y=0.2, yaw=math.pi / 2)
        self.node._velocity_pub.reset_mock()
        step = 0

        def step_feedback(_):
            nonlocal step
            step += 1
            if step == 1:
                self.feedback(x=0.2, y=0.2, yaw=math.pi)
            elif step == 2:
                self.feedback(x=0.2, y=0.2, yaw=-math.pi / 2)
            elif step == 3:
                self.feedback(x=0.2, y=0.14, yaw=-math.pi / 2)

        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=step_feedback):
            res = self.execute("FL", 180)
            self.assertTrue(res.success)
        vels = self.velocities()
        self.assertGreater(abs(vels[0][1]), 0)
        has_straight_runout = any(v[0] > 0 and v[1] == 0.0 for v in vels)
        self.assertTrue(has_straight_runout)
        self.assertEqual(vels[-1], (0, 0))

    def test_turn_overshoot_lead_stops_early(self):
        self.feedback(yaw=0.0)
        # For a 90 deg turn with default 2.2 deg lead (stop_threshold ~ 0.0384 rad):
        # When yaw reaches 88.0 deg (remaining = 2.0 deg <= 2.2 deg), it completes.
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.feedback(yaw=math.radians(88.0))):
            self.assertTrue(self.execute("FL", 90).success)

    def test_dynamic_parameter_update(self):
        from rclpy.parameter import Parameter
        param = Parameter("turn_overshoot_deg", Parameter.Type.DOUBLE, 3.5)
        self.node.set_parameters([param])
        self.assertAlmostEqual(self.node._turn_overshoot_deg, 3.5)

