#!/usr/bin/env python3
"""Real ROS preview with synthetic publisher; no movement services are created."""
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from run import load_config
from bullseye.ros_node import NavigationNode


def main():
    config, camera = load_config(ROOT/'config/mission.yaml')
    config['vision']['image_topic'] = '/bullseye/replay/image'
    rclpy.init(args=[])
    preview = NavigationNode(ROOT, config, camera, motion=False)
    publisher_node = Node('bullseye_saved_image_fixture')
    publisher = publisher_node.create_publisher(Image, '/bullseye/replay/image', 1)
    bridge = CvBridge()
    reference = cv2.imread(str(ROOT/'assets/bullseye-reference.png'))
    blank = np.full((480, 640, 3), 255, np.uint8)
    marked = blank.copy()
    h, w = reference.shape[:2]
    marked[100:100+h, 200:200+w] = reference
    executor = SingleThreadedExecutor()
    executor.add_node(preview); executor.add_node(publisher_node)
    seen_marker = seen_blank_after = False
    started = last_publish = time.monotonic()
    try:
        while time.monotonic()-started < 12:
            elapsed = time.monotonic()-started
            if time.monotonic()-last_publish >= .1:
                frame = marked if 2 <= elapsed < 7 else blank
                msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = publisher_node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                publisher.publish(msg)
                last_publish = time.monotonic()
            executor.spin_once(timeout_sec=.02)
            if 3 <= elapsed < 7 and preview.preview['markers']:
                seen_marker = True
            if elapsed > 9 and not preview.preview['markers']:
                seen_blank_after = True
        assert seen_marker, 'Reference marker not observed through ROS'
        assert seen_blank_after, 'Blank frames did not clear marker'
        assert not hasattr(preview, 'move_client')
        assert not hasattr(preview, 'estop_pub')
        assert not hasattr(preview, 'result_pub')
        clients = preview.get_client_names_and_types_by_node('bullseye_navigation', '/')
        assert not clients, f'Preview unexpectedly has ROS service clients: {clients}'
        result = {'passed': True, 'fixture_type': 'saved-reference and synthetic blank frames over ROS',
                  'marker_seen': seen_marker, 'blank_cleared_marker': seen_blank_after,
                  'motion_interfaces_created': False, 'frames_processed': preview.preview['frames_processed']}
        (ROOT/'reports/ros-replay.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        preview.close(); executor.remove_node(preview); executor.remove_node(publisher_node)
        preview.destroy_node(); publisher_node.destroy_node(); executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
