#!/usr/bin/env python3
"""Preview, offline validation and explicit opt-in mission entry point."""
import argparse
from dataclasses import asdict
import fcntl
import json
from pathlib import Path
import sys
import time

import cv2
import yaml

ROOT = Path(__file__).resolve().parent


def load_config(path):
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    camera_path = path.parent/config['camera_calibration']
    return config, yaml.safe_load(camera_path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/mission.yaml')
    sub = parser.add_subparsers(dest='command', required=True)
    image = sub.add_parser('image', help='Detect a saved image without ROS or motion')
    image.add_argument('input', type=Path)
    image.add_argument('--output', type=Path, default=ROOT/'reports/offline')
    for name in ('preview', 'mission'):
        command = sub.add_parser(name)
        command.add_argument('--seconds', type=float)
        command.add_argument('--display', action='store_true')
        command.add_argument('--capture-calibration', action='store_true')
        if name == 'mission':
            command.add_argument('--enable-motion', action='store_true', required=True)
    sub.add_parser('preflight', help='Check configured geometry; never creates ROS control interfaces')
    check = sub.add_parser('check-model')
    check.add_argument('--model', type=Path)
    check.add_argument('--sha256')
    args = parser.parse_args()
    config, camera = load_config(args.config)
    from bullseye.geometry import CameraGeometry
    from bullseye.mission import configuration_errors
    errors = configuration_errors(config, CameraGeometry(camera))
    if args.command == 'preflight':
        print(json.dumps({'ready_for_motion_configuration': not errors, 'errors': errors,
                          'note': 'Live telemetry, controller ownership and physical verification are checked separately.'}, indent=2))
        return 0 if not errors else 2
    if args.command == 'check-model':
        from bullseye.symbols import SymbolReader
        spec = config['recognition']
        reader = SymbolReader(args.model or spec['model'], args.sha256 or spec['sha256'], spec['threads'])
        print(json.dumps({'loaded': True, 'names': reader.names, 'marker_assessment_id': None}, indent=2))
        return 0
    if args.command == 'image':
        from bullseye.vision import MarkerDetector
        frame = cv2.imread(str(args.input))
        if frame is None:
            raise ValueError(f'Cannot read {args.input}')
        detector = MarkerDetector(ROOT/config['vision']['reference'],
                                  min_quality=config['vision']['min_quality'])
        started = time.perf_counter()
        markers = detector.detect(frame, time.time())
        elapsed = time.perf_counter()-started
        records = []
        for m in markers:
            records.append({'quality': m.quality, 'centre': m.centre.tolist(), 'corners': m.corners.tolist()})
            cv2.polylines(frame, [m.corners.astype('int32')], True, (0, 200, 0), 2)
        args.output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output/'annotated.png'), frame)
        data = {'input': str(args.input.resolve()), 'markers': records, 'elapsed_s': elapsed,
                'motion_enabled': False}
        (args.output/'result.json').write_text(json.dumps(data, indent=2)+'\n')
        print(json.dumps(data, indent=2))
        return 0
    if args.command == 'mission' and errors:
        print('Motion refused before ROS startup:\n- '+'\n- '.join(errors), file=sys.stderr)
        return 2
    lock_file = (ROOT/'reports/node.lock').open('a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Another instance of this add-on is running.', file=sys.stderr)
        return 2
    import rclpy
    from bullseye.ros_node import NavigationNode
    rclpy.init(args=[])
    node = None
    try:
        node = NavigationNode(ROOT, config, camera, motion=args.command == 'mission',
                              display=args.display, seconds=args.seconds,
                              capture_calibration=args.capture_calibration)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock_file.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
