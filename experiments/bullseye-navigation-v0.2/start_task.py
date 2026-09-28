#!/usr/bin/env python3
"""Supervise the independent bullseye task; do not edit the existing ROS project."""
import argparse
import fcntl
import importlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
WORKSPACE = Path('/home/mdp/dev/SC2079-Group-16/ros2_ws')
TERMINAL_STATES = {'COMPLETE', 'STAGE_COMPLETE', 'FAILED', 'STOPPED'}


def port_open(port=7447):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=.3):
            return True
    except OSError:
        return False


def executable(package, name):
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix(package))/'lib'/package/name
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f'Missing installed ROS executable: {path}')
    return str(path)


def parameters(values):
    result = ['--ros-args']
    for key, value in values.items():
        result += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
    return result


def component_commands(config, serial_port):
    """Only the three required hardware components, with session-only parameters."""
    motion = config['motion']
    shared = {'velocity_speed_mps': motion['max_controller_speed_mps'],
              'velocity_max_speed_mps': motion['max_controller_speed_mps'],
              'velocity_turn_radius_m': motion['turning_radius_m'],
              'disable_proximity_estop': False}
    return {
        'camera': [executable('mdp_camera_bringup', 'pi_camera_node')] + parameters({'fps': 15.0}),
        'serial': [executable('mdp_hardware_bridge', 'serial_bridge_node')] +
                  parameters({**shared, 'serial_port': serial_port}),
        'controller': [executable('mdp_hardware_bridge', 'motion_controller_node')] + parameters(shared),
    }


class Children:
    """Own only processes started here; never kill a shared router or camera."""
    def __init__(self, directory):
        self.directory = directory
        self.items = []

    def start(self, name, command, env=None):
        log = (self.directory/(name+'.log')).open('w')
        try:
            process = subprocess.Popen(command, cwd=WORKSPACE, env=env,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.items.append((name, process, log))
        print(f'Started {name}. Log: {log.name}', flush=True)
        return process

    def check(self):
        for name, process, log in self.items:
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited ({process.returncode}); see {log.name}')

    def close(self):
        # Stop navigation first, then controller, serial, camera and owned router.
        for name, process, log in reversed(self.items):
            for sig, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    # Kill descendants still in our own group (camera daemon, etc.).
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    break
                except subprocess.TimeoutExpired:
                    continue
            log.close()


class Probe:
    """Read ROS readiness; creates an emergency-stop publisher only for owned hardware."""
    def __init__(self, image_topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = Node('bullseye_task_launcher')
        self.last_image = None
        self.status = None
        self.status_received = None
        self.estop = None
        self.node.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.node.create_subscription(String, '/bullseye/status', self.on_status, 10)

    def on_image(self, msg):
        age = self.node.get_clock().now().nanoseconds/1e9 - (msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        if 0 <= age <= .5 and msg.width == 640 and msg.height == 480 and len(msg.data) > 0:
            self.last_image = time.monotonic()

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_received = time.monotonic()
        except ValueError:
            pass

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=.1)

    def wait(self, predicate, seconds, children, description):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            children.check()
            self.spin()
            if predicate():
                return
        raise RuntimeError('Timed out waiting for '+description)

    def fresh_image(self):
        return self.last_image is not None and time.monotonic()-self.last_image <= 2

    def reject_existing_task(self, motion):
        names = self.node.get_node_names()
        forbidden = ['bullseye_navigation']
        if motion:
            forbidden += ['serial_bridge', 'motion_controller', 'planner_node', 'fastest_car',
                          'teleop', 'android_bridge']
        conflict = sorted({name for name in names if any(word in name for word in forbidden)})
        if motion and any(name == '/execute_moves' for name, _ in self.node.get_service_names_and_types()):
            conflict.append('/execute_moves is already provided')
        if conflict:
            raise RuntimeError('An existing task owns these components: '+', '.join(conflict)+
                               '. Stop that task in its own terminal before retrying.')

    def enable_stop(self):
        from std_msgs.msg import Empty
        self.estop = self.node.create_publisher(Empty, '/estop', 10)

    def request_stop(self):
        if self.estop is not None and self.rclpy.ok():
            from std_msgs.msg import Empty
            for _ in range(3):
                self.estop.publish(Empty())
                self.spin()

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def inspect_setup(args):
    from run import load_config
    from bullseye.geometry import CameraGeometry
    from bullseye.mission import configuration_errors
    config, camera = load_config(args.config)
    if args.stage:
        config['mission_stage'] = args.stage
    errors = configuration_errors(config, CameraGeometry(camera))
    dependencies = []
    for module in ('rclpy', 'cv_bridge', 'sensor_msgs.msg', 'mdp_interfaces.srv', 'onnxruntime'):
        try:
            importlib.import_module(module)
        except Exception as exc:
            dependencies.append(f'{module}: {exc}')
    try:
        commands = component_commands(config, args.serial_port)
        executable('rmw_zenoh_cpp', 'rmw_zenohd')
    except Exception as exc:
        dependencies.append(str(exc))
        commands = {}
    model_ok, model_error = False, None
    try:
        from bullseye.symbols import SymbolReader
        spec = config['recognition']
        SymbolReader(spec['model'], spec['sha256'], spec['threads'])
        model_ok = True
    except Exception as exc:
        model_error = str(exc)
    report = {'files_and_dependencies_ready': not dependencies,
              'model_verified': model_ok, 'model_error': model_error,
              'ready_for_motion_configuration': not errors,
              'configuration_errors': errors, 'dependency_errors': dependencies,
              'serial_port': args.serial_port, 'serial_device_present': Path(args.serial_port).exists(),
              'mission_stage': config['mission_stage'], 'hardware_started': False}
    return config, report, commands


def run_task(args, config, commands):
    import yaml
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('Use start_bullseye.sh to activate the existing Pi Zenoh environment.')
    directory = ROOT/'reports'/('session-'+time.strftime('%Y%m%dT%H%M%S')+'-'+str(os.getpid()))
    directory.mkdir(parents=True)
    (ROOT/'reports/latest-session.txt').write_text(str(directory)+'\n')
    # Preserve user configuration; the stage override exists only in this session.
    config = dict(config)
    config['camera_calibration'] = str((args.config.resolve().parent/config['camera_calibration']).resolve())
    runtime_config = directory/'mission.yaml'
    runtime_config.write_text(yaml.safe_dump(config, sort_keys=False))
    children, probe, owns_motion = Children(directory), None, False
    result = {'mode': 'preview' if args.preview else 'mission', 'result': 'starting'}
    try:
        if not port_open():
            env = os.environ.copy()
            env['ZENOH_SESSION_CONFIG_URI'] = str(WORKSPACE/'config/zenoh_router_pi.json5')
            children.start('router', [executable('rmw_zenoh_cpp', 'rmw_zenohd')], env)
            deadline = time.monotonic()+10
            while not port_open():
                children.check()
                if time.monotonic() > deadline:
                    raise RuntimeError('The local ROS router did not start within 10 seconds')
                time.sleep(.1)
        else:
            print('Using the existing ROS router.', flush=True)
        probe = Probe(config['vision']['image_topic'])
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            probe.spin()
        probe.reject_existing_task(motion=not args.preview)
        count = probe.node.count_publishers(config['vision']['image_topic'])
        if count > 1:
            raise RuntimeError('Multiple camera publishers found; select one before starting.')
        if count == 0:
            if args.existing_camera_only:
                raise RuntimeError('No existing camera publisher; --existing-camera-only prevents camera startup.')
            # Existing camera process without a publisher may still own the sensor.
            if any(n in ('camera', 'pi_camera_node') for n in probe.node.get_node_names()):
                raise RuntimeError('An existing camera node is not publishing the configured image topic.')
            children.start('camera', commands['camera'])
        else:
            print('Using the existing camera publisher.', flush=True)
        probe.wait(probe.fresh_image, 15, children, 'fresh 640 x 480 camera frames')
        if not args.preview:
            # Check again after camera startup, before opening the motor link.
            probe.reject_existing_task(motion=True)
            probe.enable_stop()
            children.start('serial', commands['serial'])
            owns_motion = True
            children.start('controller', commands['controller'])
            probe.wait(lambda: any(n == '/execute_moves' for n, _ in probe.node.get_service_names_and_types()),
                       10, children, 'the motion service')
        nav = [sys.executable, '-u', str(ROOT/'run.py'), '--config', str(runtime_config),
               'preview' if args.preview else 'mission']
        if not args.preview:
            nav += ['--enable-motion']
        if args.display:
            nav += ['--display']
        if args.capture_calibration:
            nav += ['--capture-calibration']
        probe.status = None
        probe.status_received = None
        children.start('navigation', nav)
        print('Bullseye task is running. Press Ctrl+C to stop.', flush=True)
        print('Annotated image: '+str(ROOT/'reports/latest.jpg'), flush=True)
        began, previous, saw_active = time.monotonic(), None, False
        while True:
            children.check()
            probe.spin()
            elapsed = time.monotonic()-began
            if args.seconds is not None and elapsed >= args.seconds:
                result.update(result='duration_limit')
                return 0
            if not probe.fresh_image():
                raise RuntimeError('Camera frames stopped arriving')
            if probe.status_received is None:
                if elapsed > 15:
                    raise RuntimeError('Navigation did not publish startup status within 15 seconds')
                continue
            if time.monotonic()-probe.status_received > 10:
                raise RuntimeError('Navigation status stopped updating')
            status = probe.status
            state = status['mission']['state']
            text = (f"{state} | markers={len(status['preview']['markers'])} | "
                    f"{status['mission']['reason']}")
            if text != previous:
                print(text, flush=True)
                previous = text
            if args.preview:
                continue
            if state != 'IDLE':
                saw_active = True
            if not saw_active and elapsed > 30:
                raise RuntimeError('Live readiness checks did not pass: '+status['mission']['reason'])
            if state in TERMINAL_STATES:
                result.update(result=state, mission=status['mission'])
                print(json.dumps(result, indent=2), flush=True)
                return 0 if state in {'COMPLETE', 'STAGE_COMPLETE'} else 1
            if elapsed > config['mission_timeout_s']+30:
                raise RuntimeError('Task exceeded its total runtime limit')
    except KeyboardInterrupt:
        result.update(result='operator_stop')
        print('\nStopping the bullseye task.', flush=True)
        return 130
    except Exception as exc:
        result.update(result='error', error=str(exc))
        print('Task stopped: '+str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            if probe is not None and owns_motion:
                probe.request_stop()
        finally:
            children.close()
            if probe is not None:
                probe.close()
            (directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            print('Session report: '+str(directory/'result.json'), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/mission.yaml')
    parser.add_argument('--check', action='store_true', help='Read-only readiness report; starts no ROS nodes')
    parser.add_argument('--preview', action='store_true', help='Camera and marker preview only; no motor link')
    parser.add_argument('--display', action='store_true', help='Show the annotated image on the Pi desktop')
    parser.add_argument('--existing-camera-only', action='store_true',
                        help='Use an existing image publisher; never start the physical camera')
    parser.add_argument('--capture-calibration', action='store_true')
    parser.add_argument('--stage', choices=('approach', 'one_face', 'full'))
    parser.add_argument('--seconds', type=float, help='Stop after this many seconds')
    parser.add_argument('--serial-port', default='/dev/ttyACM1')
    args = parser.parse_args(argv)
    if args.seconds is not None and args.seconds <= 0:
        parser.error('--seconds must be positive')
    if args.capture_calibration and not args.preview:
        parser.error('--capture-calibration requires --preview')
    (ROOT/'reports').mkdir(exist_ok=True)
    try:
        config, report, commands = inspect_setup(args)
    except Exception as exc:
        print('Cannot read the bullseye setup: '+str(exc), file=sys.stderr)
        return 2
    (ROOT/'reports/readiness.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.check:
        print(json.dumps(report, indent=2))
        return 0 if (report['files_and_dependencies_ready'] and report['model_verified'] and
                     report['ready_for_motion_configuration'] and report['serial_device_present']) else 2
    errors = list(report['dependency_errors'])
    if not args.preview:
        errors += report['configuration_errors']
        if not report['model_verified']:
            errors.append('Model verification failed: '+str(report['model_error']))
        if not report['serial_device_present']:
            errors.append('STM32 serial device is missing: '+args.serial_port)
    if errors:
        print('Bullseye startup blocked before hardware startup:\n- '+'\n- '.join(errors), file=sys.stderr)
        print('Setup instructions: '+str(ROOT/'START_HERE.md'), file=sys.stderr)
        print('Camera-only command: '+str(ROOT/'start_bullseye.sh')+' --preview', file=sys.stderr)
        return 2
    with (ROOT/'reports/task.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('A bullseye launcher is already running.', file=sys.stderr)
            return 2
        # A signal or a closed SSH terminal follows the same owned-process cleanup.
        def interrupted(signum, frame):
            raise KeyboardInterrupt
        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, interrupted)
        return run_task(args, config, commands)


if __name__ == '__main__':
    raise SystemExit(main())
