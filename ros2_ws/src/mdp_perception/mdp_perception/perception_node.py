#!/usr/bin/env python3
"""ROS 2 Perception Node running on Laptop/PC GPU.

Subscribes to the camera stream from the Raspberry Pi (/camera/image_raw),
performs real-time YOLOv8 symbol detection, publishes recognized target IDs
to /android/target, and builds the stitched verification image grid.
"""

from __future__ import annotations

from collections import deque
import os
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mdp_interfaces.srv import SampleTarget
from mdp_perception.detector import TargetDetector
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class PerceptionNode(Node):
    """ROS 2 YOLO Perception & Verification Grid Node."""

    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("model_path", "models/best.onnx")
        self.declare_parameter("conf_threshold", 0.50)
        self.declare_parameter("output_dir", "runs")
        self.declare_parameter("continuous_inference", False)  # Set False for high efficiency on Pi
        self.declare_parameter("preview_fps", 1.0)  # Throttled preview FPS for Foxglove

        model_path = self.get_parameter("model_path").value
        conf_th = self.get_parameter("conf_threshold").value
        self._output_dir = self.get_parameter("output_dir").value
        self._continuous_inference = bool(self.get_parameter("continuous_inference").value)
        self._preview_fps = float(self.get_parameter("preview_fps").value)
        os.makedirs(self._output_dir, exist_ok=True)

        self._detector = TargetDetector(model_path=model_path, conf_threshold=conf_th)
        self._bridge = CvBridge()

        self._current_obs_id = 1
        self._last_detect_time = 0.0
        self._last_preview_time = 0.0
        self._detect_cooldown = 1.0  # Cooldown between reporting the same obstacle

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_header = None

        # Live rolling detection buffer for zero-wait consensus sampling
        # Stores tuples: (timestamp, detections, raw_frame_copy)
        self._buffer_lock = threading.Lock()
        self._shutdown_event = threading.Event()
        self._detection_history: deque = deque(maxlen=10)

        # Callback groups
        image_callback_group = MutuallyExclusiveCallbackGroup()
        service_callback_group = ReentrantCallbackGroup()
        status_callback_group = ReentrantCallbackGroup()

        # Publishers
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        # BEST_EFFORT + CompressedImage: this is a high-rate visualization
        # stream (Foxglove only -- nothing else in the graph subscribes to
        # it), not control data, so dropping a stale frame under load is
        # correct and cheap frames matter more than perfect delivery.
        # Publishing raw BGR8 (~921KB/frame at 640x480) blows straight
        # through foxglove_bridge's default 10MB send_buffer_limit after
        # ~10 frames' worth of backlog -- it then drops the client
        # connection outright, which looks exactly like "frames stopped
        # dead after some fixed count" rather than gradual lag. The
        # camera's own /camera/image_raw/compressed (JPEG, ~30-80KB/frame)
        # never hits this because it's an order of magnitude smaller;
        # matching that pattern here fixes it the same way.
        self._annotated_pub = self.create_publisher(
            CompressedImage, "/perception/image_annotated", qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(String, "/android/status", 10)

        # Subscribers
        self._image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, 1, callback_group=image_callback_group
        )
        self._status_sub = self.create_subscription(
            String, "/android/status", self._on_status, 10, callback_group=status_callback_group
        )

        # Service Server for fast consensus sampling
        self._sample_srv = self.create_service(
            SampleTarget, "/perception/sample_target", self._on_sample_target, callback_group=service_callback_group
        )

        self.get_logger().info("Perception node running. Continuous YOLO inference with instant consensus sampling active.")

    def _on_status(self, msg: String) -> None:
        """Parse status to track which obstacle the robot is currently visiting."""
        txt = msg.data.lower()
        if "at obs" in txt or "navigating to obs" in txt:
            # Extract obstacle ID (e.g. "Navigating to Obs 3")
            words = txt.split()
            for i, w in enumerate(words):
                if w == "obs" and i + 1 < len(words):
                    try:
                        self._current_obs_id = int(words[i + 1].strip(":,"))
                    except ValueError:
                        pass

    def _on_image(self, msg: Image) -> None:
        """Process incoming frame from the Raspberry Pi camera with duty-cycling."""
        if self._shutdown_event.is_set() or not rclpy.ok():
            return
        try:
            # Convert ROS Image to OpenCV BGR
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode image message: {exc}")
            return

        with self._buffer_lock:
            self._latest_frame = frame
            self._latest_header = msg.header

        now = time.time()
        should_infer = False
        if self._continuous_inference:
            should_infer = True
        elif (
            self._preview_fps > 0.0
            and self._annotated_pub.get_subscription_count() > 0
            and (now - self._last_preview_time) >= (1.0 / self._preview_fps)
        ):
            should_infer = True

        if should_infer:
            self._last_preview_time = now
            detections = self._detector.predict(frame)
            with self._buffer_lock:
                self._detection_history.append((now, detections, frame.copy()))
            annotated = self._detector.draw_detections(frame, detections)
        elif self._annotated_pub.get_subscription_count() > 0:
            # Low-overhead preview passthrough without running heavy YOLO
            annotated = frame
        else:
            annotated = None

        if annotated is not None and self._annotated_pub.get_subscription_count() > 0:
            try:
                annotated_msg = self._bridge.cv2_to_compressed_imgmsg(annotated, dst_format="jpg")
                annotated_msg.header = msg.header
                self._annotated_pub.publish(annotated_msg)
            except Exception:
                pass

    def _evaluate_consensus(
        self,
        min_frames: int = 2,
        conf_threshold: float = 0.50,
        max_age_sec: float = 1.0,
    ) -> Optional[Tuple[str, int, float, Tuple[int, int, int, int], np.ndarray, bool]]:
        """Evaluate consensus across recent frames in the live buffer.

        Returns:
            (symbol_name, symbol_id, conf, box, frame, is_marker) or None
        """
        now = time.time()
        with self._buffer_lock:
            valid_entries = [
                (t, dets, f) for (t, dets, f) in self._detection_history if (now - t) <= max_age_sec
            ]

        if not valid_entries:
            return None

        # Aggregate counts and best detection per symbol
        symbol_counts: dict[int, int] = {}
        best_per_symbol: dict[int, Tuple[str, int, float, Tuple[int, int, int, int], np.ndarray]] = {}

        for _, dets, f in valid_entries:
            for name, sid, conf, box in dets:
                if conf >= conf_threshold:
                    symbol_counts[sid] = symbol_counts.get(sid, 0) + 1
                    if sid not in best_per_symbol or conf > best_per_symbol[sid][2]:
                        best_per_symbol[sid] = (name, sid, conf, box, f)

        # 1. Real target symbols (IDs 11 to 40: digits, letters, arrows, Stop)
        target_candidates = [
            (sid, count) for sid, count in symbol_counts.items() if 11 <= sid <= 40 and count >= min_frames
        ]
        if target_candidates:
            target_candidates.sort(key=lambda item: (item[1], best_per_symbol[item[0]][2]), reverse=True)
            best_sid = target_candidates[0][0]
            name, sid, conf, box, f = best_per_symbol[best_sid]
            return (name, sid, conf, box, f, False)

        # 2. Single high-confidence detection (conf >= 0.70)
        single_confident = [
            (sid, data) for sid, data in best_per_symbol.items() if 11 <= sid <= 40 and data[2] >= 0.70
        ]
        if single_confident:
            single_confident.sort(key=lambda item: item[1][2], reverse=True)
            name, sid, conf, box, f = single_confident[0][1]
            return (name, sid, conf, box, f, False)

        # 3. Bull's Eye marker -- the "target" class only. "circle" (Stop) is
        # a real target handled above and must NOT be treated as a marker;
        # they are distinct classes with distinct IDs (see detector.py's
        # _LABEL_TO_SYMBOL_ID), so this checks the name, not a shared ID.
        marker_candidates = [
            (sid, count)
            for sid, count in symbol_counts.items()
            if best_per_symbol[sid][0] == "target" and count >= min_frames
        ]
        if marker_candidates:
            best_sid = marker_candidates[0][0]
            name, sid, conf, box, f = best_per_symbol[best_sid]
            return (name, sid, conf, box, f, True)

        return None

    def _on_sample_target(
        self, request: SampleTarget.Request, response: SampleTarget.Response
    ) -> SampleTarget.Response:
        """Handle sample request from planner_node with fast on-demand or consensus sampling."""
        if self._shutdown_event.is_set() or not rclpy.ok():
            response.success = False
            response.symbol_id = 0
            response.symbol_name = "SHUTDOWN"
            response.confidence = 0.0
            response.is_marker = False
            return response
        t_start = time.time()
        obs_id = request.obstacle_id
        deadline = t_start + 0.35  # Up to 350ms settle window if arriving concurrently

        consensus = None
        if self._continuous_inference:
            while time.time() < deadline:
                if self._shutdown_event.is_set() or not rclpy.ok():
                    break
                consensus = self._evaluate_consensus(
                    min_frames=2, conf_threshold=self._detector.conf_threshold
                )
                if consensus is not None:
                    break
                time.sleep(0.02)

        # In on-demand mode (or if continuous had no immediate hit), infer directly on latest camera frame
        if consensus is None:
            with self._buffer_lock:
                target_frame = self._latest_frame.copy() if self._latest_frame is not None else None

            if target_frame is not None:
                self.get_logger().info(f"⚡ Running on-demand inference for Obs {obs_id} on latest camera frame...")
                detections = self._detector.predict(target_frame)
                with self._buffer_lock:
                    self._detection_history.append((time.time(), detections, target_frame))
                consensus = self._evaluate_consensus(
                    min_frames=1, conf_threshold=self._detector.conf_threshold, max_age_sec=10.0
                )

        elapsed_ms = (time.time() - t_start) * 1000.0

        if consensus is not None:
            name, symbol_id, conf, box, frame, is_marker = consensus
            response.success = True
            response.symbol_id = int(symbol_id)
            response.symbol_name = str(name)
            response.confidence = float(conf)
            response.is_marker = bool(is_marker)

            # Register crop and update stitched verification grid
            self._detector.register_target_crop(
                obstacle_id=obs_id,
                frame=frame,
                symbol_id=symbol_id,
                conf=conf,
                box=box,
            )
            grid = self._detector.build_verification_grid()
            stitch_path = os.path.join(self._output_dir, "stitched_verification.jpg")
            cv2.imwrite(stitch_path, grid)

            # Publish confirmed TARGET to Android tablet (only real target symbols 11-39, not markers)
            if not is_marker:
                self._target_pub.publish(String(data=f"{obs_id},{symbol_id}"))

            self.get_logger().info(
                f"⚡ Live Consensus Sampled for Obs {obs_id}: Symbol {symbol_id} ({name}) "
                f"[Conf: {conf:.2f}, Marker: {is_marker}] in {elapsed_ms:.1f}ms"
            )
        else:
            response.success = False
            response.symbol_id = 0
            response.symbol_name = "UNCERTAIN"
            response.confidence = 0.0
            response.is_marker = False
            self.get_logger().warn(
                f"No consensus target found for Obs {obs_id} within {elapsed_ms:.1f}ms settle window."
            )

        return response

    def request_shutdown(self) -> None:
        """Stop accepting new image/service work before executor teardown."""
        self._shutdown_event.set()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_shutdown()
        try:
            executor.shutdown(timeout_sec=2.0)
        except (Exception, KeyboardInterrupt):
            pass
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
