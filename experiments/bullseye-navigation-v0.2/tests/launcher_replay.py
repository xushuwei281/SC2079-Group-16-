#!/usr/bin/env python3
"""Exercise the one-command supervisor with saved images and no hardware startup."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from run import load_config


def main():
    config, _ = load_config(ROOT/'config/mission.yaml')
    config['camera_calibration'] = str(ROOT/'config/camera_calibration.yaml')
    config['vision']['image_topic'] = '/bullseye/launcher_fixture/image'
    rclpy.init(args=[])
    node = Node('bullseye_launcher_saved_fixture')
    publisher = node.create_publisher(Image, config['vision']['image_topic'], 1)
    statuses = []
    node.create_subscription(String, '/bullseye/status', lambda msg: statuses.append(json.loads(msg.data)), 10)
    bridge = CvBridge()
    frame = np.full((480, 640, 3), 255, np.uint8)
    reference = cv2.imread(str(ROOT/'assets/bullseye-reference.png'))
    h, w = reference.shape[:2]
    frame[100:100+h, 200:200+w] = reference
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='bullseye-launch-fixture-') as directory:
            config_path = Path(directory)/'mission.yaml'
            config_path.write_text(yaml.safe_dump(config))
            log_path = ROOT/'reports/launcher-replay.log'
            with log_path.open('w') as log:
                process = subprocess.Popen(
                    [sys.executable, '-u', str(ROOT/'start_task.py'), '--preview',
                     '--existing-camera-only', '--seconds', '5', '--config', str(config_path)],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                began, last = time.monotonic(), -1
                while process.poll() is None:
                    if time.monotonic()-began > 45:
                        raise AssertionError('Launcher did not finish within 45 seconds')
                    if time.monotonic()-last >= .1:
                        msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
                        msg.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(msg)
                        last = time.monotonic()
                    rclpy.spin_once(node, timeout_sec=.02)
            assert process.returncode == 0, log_path.read_text()
            assert any(s['preview']['markers'] for s in statuses), 'Saved marker was not detected'
            assert all(s['mode'] == 'preview' for s in statuses)
            session = Path((ROOT/'reports/latest-session.txt').read_text().strip())
            logs = sorted(p.name for p in session.glob('*.log'))
            assert not any(name in logs for name in ('serial.log', 'controller.log', 'camera.log')), logs
            result = {'passed': True, 'source': 'Saved reference over ROS; physical camera startup forbidden',
                      'launcher_exit_code': process.returncode, 'status_messages': len(statuses),
                      'saved_marker_seen': True, 'component_logs': logs,
                      'session_result': json.loads((session/'result.json').read_text())}
            (ROOT/'reports/launcher-replay.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
