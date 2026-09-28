"""Startup boundaries and owned-process cleanup without robot hardware."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import start_task
from run import load_config


class StartupTests(unittest.TestCase):
    def test_low_speed_session_keeps_proximity_enabled(self):
        config, _ = load_config(ROOT/'config/mission.yaml')
        with patch.object(start_task, 'executable', side_effect=lambda p, n: '/installed/'+n):
            commands = start_task.component_commands(config, '/dev/confirmed-stm32')
        self.assertEqual(set(commands), {'camera', 'serial', 'controller'})
        for component in ('serial', 'controller'):
            self.assertIn('velocity_max_speed_mps:=0.1', commands[component])
            self.assertIn('disable_proximity_estop:=false', commands[component])
            self.assertIn('velocity_turn_radius_m:=0.21', commands[component])
        self.assertIn('serial_port:=/dev/confirmed-stm32', commands['serial'])

    def test_incomplete_setup_never_starts_processes(self):
        report = {'dependency_errors': [], 'configuration_errors': ['calibration missing'],
                  'model_verified': True, 'serial_device_present': True}
        with tempfile.TemporaryDirectory() as directory, patch.object(start_task, 'ROOT', Path(directory)), \
                patch.object(start_task, 'inspect_setup', return_value=({}, report, {})), \
                patch.object(start_task, 'run_task') as run:
            self.assertEqual(start_task.main([]), 2)
            run.assert_not_called()

    def test_preview_can_run_without_motion_calibration_or_model(self):
        report = {'dependency_errors': [], 'configuration_errors': ['calibration missing'],
                  'model_verified': False, 'serial_device_present': False}
        with tempfile.TemporaryDirectory() as directory, patch.object(start_task, 'ROOT', Path(directory)), \
                patch.object(start_task, 'inspect_setup', return_value=({}, report, {})), \
                patch.object(start_task.signal, 'signal'), \
                patch.object(start_task, 'run_task', return_value=0) as run:
            self.assertEqual(start_task.main(['--preview']), 0)
            self.assertTrue(run.call_args.args[0].preview)

    def test_competing_hardware_prevents_start(self):
        probe = start_task.Probe.__new__(start_task.Probe)
        probe.node = Mock()
        probe.node.get_node_names.return_value = ['serial_bridge_node']
        probe.node.get_service_names_and_types.return_value = []
        with self.assertRaisesRegex(RuntimeError, 'serial_bridge_node'):
            probe.reject_existing_task(True)
        probe.reject_existing_task(False)  # Preview does not take control.

    def test_unknown_service_owner_prevents_start(self):
        probe = start_task.Probe.__new__(start_task.Probe)
        probe.node = Mock()
        probe.node.get_node_names.return_value = ['renamed_controller']
        probe.node.get_service_names_and_types.return_value = [('/execute_moves', ['custom/Service'])]
        with self.assertRaisesRegex(RuntimeError, 'already provided'):
            probe.reject_existing_task(True)

    def test_cleanup_only_stops_owned_process(self):
        outsider = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        try:
            with tempfile.TemporaryDirectory() as directory, patch.object(start_task, 'WORKSPACE', Path(directory)):
                children = start_task.Children(Path(directory))
                owned = children.start('fixture', [sys.executable, '-c', 'import time; time.sleep(30)'])
                children.check()
                children.close()
                self.assertIsNotNone(owned.poll())
                self.assertIsNone(outsider.poll())
        finally:
            outsider.terminate()
            outsider.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
