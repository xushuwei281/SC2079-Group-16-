import math
import os
import unittest
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from std_msgs.msg import Empty

from mdp_hardware_bridge.serial_bridge_node import SerialBridgeNode, _VALID_COMMANDS
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
        # Prevent automatic connection to real serial hardware in unit tests
        with patch("serial.Serial"):
            self.node = SerialBridgeNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_valid_command_codes(self):
        expected = {"FC", "BC", "FL", "FR", "BL", "BR", "FU", "BU", "GC", "G0", "TO"}
        self.assertEqual(_VALID_COMMANDS, expected)

    def test_encode_valid(self):
        self.assertEqual(self.node._encode("FC", 50), b"FC050")
        self.assertEqual(self.node._encode("BC", 0), b"BC000")
        self.assertEqual(self.node._encode("FL", 90), b"FL090")
        self.assertEqual(self.node._encode("FR", 180), b"FR180")
        self.assertEqual(self.node._encode("BL", 45), b"BL045")
        self.assertEqual(self.node._encode("BR", 30), b"BR030")
        self.assertEqual(self.node._encode("FU", 15), b"FU015")
        self.assertEqual(self.node._encode("BU", 20), b"BU020")
        self.assertEqual(self.node._encode("FC", 999), b"FC999")

    def test_encode_invalid_command(self):
        with self.assertRaises(ValueError):
            self.node._encode("INVALID", 10)
        with self.assertRaises(ValueError):
            self.node._encode("STP", 0)

    def test_encode_out_of_range_value(self):
        with self.assertRaises(ValueError):
            self.node._encode("FC", -1)
        with self.assertRaises(ValueError):
            self.node._encode("FC", 1000)

    def test_candidate_ports_discovery(self):
        candidates = self.node._find_candidate_ports()
        self.assertIn("/dev/ttyACM0", candidates)
        self.assertIn("/dev/ttySTM32", candidates)

    @patch("serial.Serial")
    def test_handle_execute_moves_success(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # First readline: RUN\r\n, Second readline: FIN\r\n
        mock_serial.readline.side_effect = [b"RUN\r\n", b"FIN\r\n"]
        self.node._serial = mock_serial

        req = ExecuteMoves.Request()
        req.commands = [
            MoveCommand(command="FC", value=50),
            MoveCommand(command="FL", value=90),
        ]
        resp = ExecuteMoves.Response()

        result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN")
        # Verify packets written: FC050, FL090, #\x00\x00\x00\x00
        calls = mock_serial.write.call_args_list
        self.assertEqual(calls[0][0][0], b"FC050")
        self.assertEqual(calls[1][0][0], b"FL090")
        self.assertEqual(calls[2][0][0], b"#\x00\x00\x00\x00")

    @patch("serial.Serial")
    def test_handle_execute_moves_bus_retry(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # First attempt: BUS\r\n; Second attempt: RUN\r\n then FIN\r\n
        mock_serial.readline.side_effect = [b"BUS\r\n", b"RUN\r\n", b"FIN\r\n"]
        self.node._serial = mock_serial

        req = ExecuteMoves.Request()
        req.commands = [MoveCommand(command="FC", value=10)]
        resp = ExecuteMoves.Response()

        with patch("time.sleep", return_value=None):
            result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN")

    def test_handle_execute_moves_disconnected(self):
        self.node._serial = None
        with patch.object(self.node, "_try_connect", return_value=False):
            req = ExecuteMoves.Request()
            req.commands = [MoveCommand(command="FC", value=10)]
            resp = ExecuteMoves.Response()

            result = self.node._handle_execute_moves(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(result.status, "DISCONNECTED")

    @patch("serial.Serial")
    def test_on_estop_sends_q_packet(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        self.node._serial = mock_serial

        self.node._on_estop(Empty())

        mock_serial.write.assert_called_with(b"Q\x00\x00\x00\x00")
        mock_serial.flush.assert_called()

    @patch("serial.Serial")
    def test_sensor_fused_fin_updates_pose(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # STM32 replies RUN, then FIN:50.2,0.4 (50.2 cm, 0.4 degrees)
        mock_serial.readline.side_effect = [b"RUN\r\n", b"FIN:50.2,0.4\r\n"]
        self.node._serial = mock_serial

        # Reset initial pose
        self.node._x = 0.0
        self.node._y = 0.0
        self.node._yaw = 0.0
        self.node._initial_yaw = 0.0

        req = ExecuteMoves.Request()
        req.commands = [MoveCommand(command="FC", value=50)]
        resp = ExecuteMoves.Response()

        result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN:50.2,0.4")
        self.assertAlmostEqual(self.node._x, 0.502, places=3)
        self.assertAlmostEqual(self.node._yaw, math.radians(0.4), places=3)

    @patch("serial.Serial")
    def test_plain_fin_falls_back_to_nominal_pose(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # STM32 replies RUN, then plain FIN\r\n
        mock_serial.readline.side_effect = [b"RUN\r\n", b"FIN\r\n"]
        self.node._serial = mock_serial

        self.node._x = 0.0
        self.node._y = 0.0
        self.node._yaw = 0.0
        self.node._initial_yaw = 0.0

        req = ExecuteMoves.Request()
        req.commands = [
            MoveCommand(command="FC", value=50),
            MoveCommand(command="FL", value=90),
        ]
        resp = ExecuteMoves.Response()

        result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN")
        self.assertAlmostEqual(self.node._x, 0.50, places=2)
    @patch("serial.Serial")
    def test_pos_fin_updates_pose(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # STM32 replies RUN, then FIN:POS,65.4,120.2,89.5,25,18,45 (in cm, degrees, US, IR1, IR2)
        mock_serial.readline.side_effect = [b"RUN\r\n", b"FIN:POS,65.4,120.2,89.5,25,18,45\r\n"]
        self.node._serial = mock_serial

        req = ExecuteMoves.Request()
        req.commands = [MoveCommand(command="FC", value=50)]
        resp = ExecuteMoves.Response()

        result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN:POS,65.4,120.2,89.5,25,18,45")
        self.assertAlmostEqual(self.node._x, 0.654, places=3)
        self.assertAlmostEqual(self.node._y, 1.202, places=3)
        self.assertAlmostEqual(self.node._yaw, math.radians(89.5), places=3)

    @patch("serial.Serial")
    def test_tlm_stream_during_moves(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        # In-flight telemetry arrives before RUN and during movement before FIN
        mock_serial.readline.side_effect = [
            b"TLM:20.0,20.0,90.0,50,40,30\r\n",
            b"RUN\r\n",
            b"TLM:25.0,20.0,90.0,45,40,30\r\n",
            b"FIN:POS,30.0,20.0,90.0,40,40,30\r\n",
        ]
        self.node._serial = mock_serial

        req = ExecuteMoves.Request()
        req.commands = [MoveCommand(command="FC", value=10)]
        resp = ExecuteMoves.Response()

        result = self.node._handle_execute_moves(req, resp)

        self.assertTrue(result.success)
        self.assertEqual(result.status, "FIN:POS,30.0,20.0,90.0,40,40,30")
        self.assertAlmostEqual(self.node._x, 0.30, places=2)

    @patch("serial.Serial")
    def test_poll_telemetry_idle(self, mock_serial_cls):
        mock_serial = MagicMock()
        mock_serial.is_open = True
        mock_serial.in_waiting = 35
        mock_serial.readline.return_value = b"TLM:22.5,21.0,88.0,60,35,42\r\n"
        self.node._serial = mock_serial

        self.node._poll_telemetry()

        self.assertAlmostEqual(self.node._x, 0.225, places=3)
        self.assertAlmostEqual(self.node._y, 0.210, places=3)
        self.assertAlmostEqual(self.node._yaw, math.radians(88.0), places=3)


if __name__ == "__main__":
    unittest.main()
