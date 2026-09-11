"""UART driver's offline wire, freshness and emergency-stop regressions."""
import math
import struct
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from mdp_hardware_bridge.serial_bridge_node import SerialBridgeNode
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves


class TestSerialBridgeNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch.object(SerialBridgeNode, "_try_connect", return_value=False):
            self.node = SerialBridgeNode()
        self.port = MagicMock()
        self.port.is_open = True
        self.port.write.side_effect = len
        self.node._serial = self.port
        self.node._enable_pose_kalman = self.node._enable_sensor_kalman = False
        self.node._handle_telemetry_line("TLM:20,20,90,50,40,40")

    def tearDown(self):
        self.node.destroy_node()

    def packets(self):
        return [c.args[0] for c in self.port.write.call_args_list]

    def test_driver_exposes_no_movement_service(self):
        self.assertEqual(self.node._service.srv_name, "/hardware/maintenance")
        req = ExecuteMoves.Request(commands=[MoveCommand(command="FC", value=10)])
        self.assertFalse(self.node._handle_maintenance(req, ExecuteMoves.Response()).success)
        self.assertEqual(self.packets(), [])

    def test_wire_units_and_invalid_curvature(self):
        self.assertEqual(self.node._velocity_packet(-0.15, -0.6),
                         b"V" + struct.pack("<hh", -150, -600))
        for speed, yaw in ((math.nan, 0), (0.31, 0), (0, 0.2), (0.1, 0.6)):
            with self.assertRaises(ValueError):
                self.node._velocity_packet(speed, yaw)

    def test_velocity_expires_and_does_not_resume(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_cmd_vel(msg)
        self.node._teleop_stamp -= 1
        self.node._velocity_tick()
        self.assertEqual(self.packets()[-1], b"V\0\0\0\0")
        count = len(self.packets())
        self.node._velocity_tick()
        self.assertEqual(count, len(self.packets()))

    def test_zero_is_accepted_without_feedback(self):
        self.node._telemetry_stamp = 0.0
        self.node._on_cmd_vel(Twist())
        self.assertEqual(self.packets(), [b"V\0\0\0\0"])
        self.assertFalse(self.node._estop_event.is_set())

    def test_stale_feedback_blocks_nonzero(self):
        self.node._telemetry_stamp = 0.0
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_cmd_vel(msg)
        self.assertEqual(self.packets(), [b"Q\0\0\0\0"])
        self.assertTrue(self.node._estop_event.is_set())

    def test_estop_needs_explicit_reset(self):
        self.node._on_estop(Empty())
        self.node._on_android_cmd(String(data="ALG|1,2,3,N"))
        self.assertTrue(self.node._estop_event.is_set())
        self.node._on_android_cmd(String(data="RESET"))
        self.assertFalse(self.node._estop_event.is_set())
        self.assertEqual(self.packets()[-1], b"R\0\0\0\0")
        self.assertFalse(self.node._telemetry_fresh())

    def test_reader_fragments_while_busy_without_write_lock(self):
        self.node._busy.set()
        self.port.in_waiting = 100
        self.port.read.side_effect = [b"TLM:22.5,21,88", b",60,35,42\r\n"]
        with self.node._write_lock:
            self.node._poll_telemetry()
            self.node._poll_telemetry()
        self.assertAlmostEqual(self.node._raw_pose[0], 0.225)
        self.assertAlmostEqual(self.node._raw_pose[2], math.radians(92))

    def test_stop_notification_propagates_estop(self):
        self.port.in_waiting = 100
        self.port.read.return_value = b"STOP:PROXIMITY\r\n"
        with patch.object(self.node._estop_pub, "publish") as publish:
            self.node._poll_telemetry()
            publish.assert_called_once()
        self.assertTrue(self.node._estop_event.is_set())
        count = len(self.packets())
        self.node._on_estop(Empty())
        self.assertEqual(count, len(self.packets()))

    def test_invalid_telemetry_cannot_refresh_watchdog(self):
        self.node._telemetry_stamp = 1.0
        self.node._handle_telemetry_line("TLM:nan,20,90,50,30,30")
        self.assertEqual(self.node._telemetry_stamp, 1.0)

    def test_maintenance_stationary_fin(self):
        def receive(_):
            self.port.in_waiting = 100
            self.port.read.return_value = b"RUN\r\nFIN:POS,20,20,90,50,40,40\r\n"
            self.node._poll_telemetry()
        req = ExecuteMoves.Request(commands=[MoveCommand(command="GC", value=0)])
        with patch("mdp_hardware_bridge.serial_bridge_node.time.sleep", side_effect=receive):
            result = self.node._handle_maintenance(req, ExecuteMoves.Response())
        self.assertTrue(result.success)
        self.assertEqual(self.packets(),
                         [b"V\0\0\0\0", b"GC000", b"#\0\0\0\0", b"V\0\0\0\0"])

    def test_partial_write_disconnects(self):
        self.port.write.side_effect = lambda _: 2
        self.assertFalse(self.node._write_packet(b"V\0\0\0\0"))
        self.assertIsNone(self.node._serial)
