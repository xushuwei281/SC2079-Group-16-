"""Isolated live recognition: latest frames, bounded rate, confirmed publications."""

import hashlib
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from mdp_perception.detector import TargetDetector
from mdp_perception.live_confirmation import LiveConfirmation


class LivePerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_live_candidate')
        defaults = {
            'model_path': '', 'output_dir': 'runs/live-candidate',
            'camera_topic': '/camera/image_raw',
            'target_topic': '/test/android/target',
            'annotated_topic': '/test/perception/image_annotated',
            'conf_threshold': 0.50, 'inference_fps': 4.0,
            'confirmation_frames': 3, 'absence_reset_seconds': 2.0,
            'max_confirmation_gap_seconds': 3.0,
            'max_frame_age_seconds': 1.0, 'max_result_age_seconds': 2.5,
            'invert_colors': True, 'onnx_threads': 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.declare_parameter('obstacle_id', 1)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        c = self.config
        for name in ['inference_fps', 'absence_reset_seconds', 'max_confirmation_gap_seconds',
                     'max_frame_age_seconds', 'max_result_age_seconds']:
            if c[name] <= 0: raise ValueError(f'{name} must be positive')
        if not 0 < c['conf_threshold'] <= 1: raise ValueError('Invalid confidence threshold')
        self.obstacle_id = self.get_parameter('obstacle_id').value
        if self.obstacle_id < 1: raise ValueError('obstacle_id must be positive')
        self.output = Path(c['output_dir']).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.detector = TargetDetector(c['model_path'], conf_threshold=c['conf_threshold'],
                                       onnx_threads=c['onnx_threads'])
        self.model_hash = hashlib.sha256(Path(self.detector.model_path).read_bytes()).hexdigest()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.last_processed = 0
        self.generation = 0
        self.last_start = 0.0
        self.last_heartbeat = 0.0
        self.started = time.monotonic()
        self.processed = 0
        self.gate = LiveConfirmation(c['confirmation_frames'], c['absence_reset_seconds'],
                                     c['max_confirmation_gap_seconds'])
        self.target_pub = self.create_publisher(String, c['target_topic'], 10)
        self.annotated_pub = self.create_publisher(CompressedImage, c['annotated_topic'],
                                                  qos_profile_sensor_data)
        self.camera_sub = self.create_subscription(
            Image, c['camera_topic'], self.on_image, qos_profile_sensor_data,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.inference_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yolo')
        self.inference_future = None
        self.timer = self.create_timer(1.0 / c['inference_fps'], self.detect_latest,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        self.add_on_set_parameters_callback(self.on_parameters)
        (self.output / 'run-config.json').write_text(json.dumps({
            **c, 'initial_obstacle_id': self.obstacle_id, 'model_sha256': self.model_hash,
            'note': 'Stationary test candidate; no mission-ready claim. Frame age uses receipt and ROS publication timestamps.'
        }, indent=2) + '\n')
        self.get_logger().info(
            f"LIVE ready: up to {c['inference_fps']} FPS, {c['confirmation_frames']} confirmations, "
            f"invert={c['invert_colors']}, obstacle={self.obstacle_id}, output={c['target_topic']}")

    def on_parameters(self, parameters):
        for p in parameters:
            if p.name == 'obstacle_id' and (p.type_ != Parameter.Type.INTEGER or p.value < 1):
                return SetParametersResult(successful=False, reason='obstacle_id must be a positive integer')
            if p.name in self.config:
                return SetParametersResult(successful=False, reason=f'{p.name} requires a restart')
        with self.lock:
            for p in parameters:
                if p.name == 'obstacle_id' and p.value != self.obstacle_id:
                    self.obstacle_id = p.value
                    self.generation += 1
                    self.gate.reset()
                    # Require a frame received after this obstacle change.
                    self.last_processed = self.sequence
                    self.latest = None
                    self.get_logger().info(f'Obstacle changed to {p.value}; confirmation reset')
        return SetParametersResult(successful=True)

    def on_image(self, msg):
        received = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Camera decode failed: {exc}')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.lock:
            self.sequence += 1
            self.latest = (self.sequence, received, stamp, msg.header, frame)

    def heartbeat(self, now, text):
        if now - self.last_heartbeat >= 5.0:
            self.last_heartbeat = now
            self.get_logger().info(text)

    def detect_latest(self):
        # Keep ROS callbacks short. Exactly one worker processes the latest
        # frame; timer ticks during inference do not queue additional work.
        if self.inference_future is not None:
            if not self.inference_future.done():
                return
            error = self.inference_future.exception()
            if error is not None:
                self.get_logger().error(f'LIVE worker failed: {error}')
        self.inference_future = self.inference_worker.submit(self._detect_latest_frame)

    def close_inference(self):
        self.timer.cancel()
        self.inference_worker.shutdown(wait=True, cancel_futures=True)

    def _detect_latest_frame(self):
        now = time.monotonic()
        if now - self.last_start < 1.0 / self.config['inference_fps']:
            return
        with self.lock:
            item = self.latest
            if item is None or item[0] == self.last_processed:
                if item is None or now - item[1] > self.config['max_frame_age_seconds']:
                    self.gate.break_sequence()
                    self.heartbeat(now, 'LIVE waiting for fresh camera frames')
                return
            seq, received, stamp, header, raw = item
            self.last_processed = seq
            source_age = ((self.get_clock().now().nanoseconds - stamp) / 1e9) if stamp else 0.0
            if (now - received > self.config['max_frame_age_seconds'] or
                    source_age > self.config['max_frame_age_seconds'] or source_age < -1.0):
                self.gate.break_sequence()
                self.heartbeat(now, 'LIVE discarding stale camera frames')
                return
            generation, obstacle = self.generation, self.obstacle_id
        self.last_start = now
        inference_frame = 255 - raw if self.config['invert_colors'] else raw
        try:
            detections = self.detector.predict(inference_frame)
        except Exception as exc:
            with self.lock: self.gate.break_sequence()
            self.get_logger().error(f'LIVE inference failed; no publication: {exc}')
            return
        finished = time.monotonic()
        inference_ms = (finished - now) * 1000
        top = max(detections, key=lambda d: d[2]) if detections else None
        symbol_id = top[1] if top is not None else None
        published = None
        with self.lock:
            if generation != self.generation:
                self.get_logger().info('LIVE discarded result from previous obstacle')
                return
            if finished - received > self.config['max_result_age_seconds']:
                self.gate.break_sequence()
                self.heartbeat(finished, 'LIVE inference result too old; discarded')
                return
            accepted = self.gate.observe(symbol_id, received)
            count = self.gate.count
            if accepted is not None:
                published = f'{obstacle},{accepted}'
                self.target_pub.publish(String(data=published))
        self.processed += 1
        record = {
            'wall_time': time.time(), 'frame_sequence': seq, 'source_stamp_ns': stamp,
            'obstacle_id': obstacle, 'symbol_id': symbol_id,
            'confidence': float(top[2]) if top else None,
            'confirmation_count': count, 'published': published,
            'inference_ms': inference_ms, 'frame_receipt_to_result_ms': (finished-received)*1000,
        }
        with (self.output / 'observations.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        if published:
            stem = f'{time.time_ns()}-obs{obstacle}-id{symbol_id}'
            evidence = {**record, 'model_sha256': self.model_hash,
                        'invert_colors': self.config['invert_colors'], 'raw_frame': stem + '-raw.png'}
            if not cv2.imwrite(str(self.output / (stem + '-raw.png')), raw):
                self.get_logger().error('Failed to save raw evidence frame')
            annotated = self.detector.draw_detections(raw, detections)
            cv2.imwrite(str(self.output / (stem + '-annotated.jpg')), annotated)
            (self.output / (stem + '.json')).write_text(json.dumps(evidence, indent=2)+'\n')
            self.get_logger().info(
                f'ACCEPTED {published} confidence={top[2]:.3f} confirmations={count} inference={inference_ms:.0f}ms')
        else:
            fps = self.processed / max(finished-self.started, 0.001)
            state = 'no target' if top is None else f'ID {symbol_id} confidence={top[2]:.3f} confirmations={count}'
            self.heartbeat(finished, f'LIVE {state}; inference={inference_ms:.0f}ms; average={fps:.2f} FPS')
        if self.annotated_pub.get_subscription_count():
            annotated = self.detector.draw_detections(raw, detections)
            message = self.bridge.cv2_to_compressed_imgmsg(annotated, dst_format='jpg')
            message.header = header
            self.annotated_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LivePerceptionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=5.0)
        node.close_inference()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
