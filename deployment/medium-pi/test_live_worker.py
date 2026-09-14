"""Exercise worker backpressure and obstacle changes with a blocked inference."""
import tempfile
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

if os.environ.get('MDP_TEST_RUNTIME'):
    sys.path.insert(0, os.environ['MDP_TEST_RUNTIME'])

import_error = ''
try:
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter
    from mdp_perception import live_perception_node as live
except ImportError as exc:
    live = None
    import_error = str(exc)


@unittest.skipIf(live is None, 'Requires the Pi ROS environment: ' + import_error)
class TestLiveWorker(unittest.TestCase):
    def test_ticks_do_not_queue_and_obstacle_change_discards_inflight_result(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        published = []

        class FakeDetector:
            def __init__(self, model_path, **kwargs):
                self.model_path = model_path

            def predict(self, frame):
                calls.append(1)
                entered.set()
                if not release.wait(5):
                    raise TimeoutError('Test did not release inference')
                return [('25', 25, 0.99, (0, 0, 2, 2))]

            def draw_detections(self, frame, detections):
                return frame

        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            model = root / 'model.onnx'
            model.write_bytes(b'worker scheduling fixture')
            rclpy.init(args=['--ros-args', '-p', f'model_path:={model}',
                             '-p', f'output_dir:={root / "output"}',
                             '-p', 'confirmation_frames:=1', '-p', 'inference_fps:=1000.0'])
            node = None
            try:
                with patch.object(live, 'TargetDetector', FakeDetector):
                    node = live.LivePerceptionNode()
                node.target_pub.publish = lambda message: published.append(message.data)
                frame = node.bridge.cv2_to_imgmsg(np.zeros((4, 4, 3), dtype=np.uint8), encoding='bgr8')
                node.on_image(frame)
                node.detect_latest()
                self.assertTrue(entered.wait(5))
                original = node.inference_future
                for _ in range(100):
                    node.detect_latest()
                self.assertIs(node.inference_future, original)
                result = node.on_parameters([Parameter('obstacle_id', Parameter.Type.INTEGER, 2)])
                self.assertTrue(result.successful)
                release.set()
                original.result(timeout=5)
                self.assertEqual(calls, [1])
                self.assertEqual(published, [])
                time.sleep(0.002)
                node.on_image(frame)
                node.detect_latest()
                node.inference_future.result(timeout=5)
                self.assertEqual(calls, [1, 1])
                self.assertEqual(published, ['2,25'])
            finally:
                release.set()
                if node is not None:
                    node.close_inference()
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
