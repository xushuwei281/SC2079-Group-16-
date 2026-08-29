#!/usr/bin/env python3
"""ROS 2 Perception Node running on Laptop/PC GPU.

Subscribes to the camera stream from the Raspberry Pi (/camera/image_raw),
performs real-time YOLOv8 symbol detection, publishes recognized target IDs
to /android/target, and builds the stitched verification image grid.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mdp_perception.detector import TargetDetector
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class PerceptionNode(Node):
    """ROS 2 YOLO Perception & Verification Grid Node."""

    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("conf_threshold", 0.50)
        self.declare_parameter("output_dir", "runs")

        model_path = self.get_parameter("model_path").value
        conf_th = self.get_parameter("conf_threshold").value
        self._output_dir = self.get_parameter("output_dir").value
        os.makedirs(self._output_dir, exist_ok=True)

        self._detector = TargetDetector(model_path=model_path, conf_threshold=conf_th)
        self._bridge = CvBridge()

        self._current_obs_id = 1
        self._last_detect_time = 0.0
        self._detect_cooldown = 1.0  # Cooldown between reporting the same obstacle

        callback_group = ReentrantCallbackGroup()

        # Publishers
        self._target_pub = self.create_publisher(String, "/android/target", 10)
        self._annotated_pub = self.create_publisher(Image, "/perception/image_annotated", 10)
        self._status_pub = self.create_publisher(String, "/android/status", 10)

        # Subscribers
        self._image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, 1, callback_group=callback_group
        )
        self._status_sub = self.create_subscription(
            String, "/android/status", self._on_status, 10, callback_group=callback_group
        )

        self.get_logger().info("Perception node running on PC GPU. Listening on /camera/image_raw...")

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
        """Process incoming frame from the Raspberry Pi camera."""
        try:
            # Convert ROS Image to OpenCV BGR
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode image message: {exc}")
            return

        # Run YOLO detection
        detections = self._detector.predict(frame)

        if detections:
            # Sort by confidence descending
            detections.sort(key=lambda d: d[2], reverse=True)
            top_name, top_symbol_id, top_conf, top_box = detections[0]

            now = time.time()
            if now - self._last_detect_time > self._detect_cooldown:
                self._last_detect_time = now
                obs_id = self._current_obs_id

                self.get_logger().info(
                    f"Recognized Target: Obstacle {obs_id} -> Symbol {top_symbol_id} ({top_name}) "
                    f"[Conf: {top_conf:.2f}]"
                )

                # 1. Publish TARGET to Android tablet
                self._target_pub.publish(String(data=f"{obs_id},{top_symbol_id}"))

                # 2. Register crop and update stitched grid
                self._detector.register_target_crop(
                    obstacle_id=obs_id,
                    frame=frame,
                    symbol_id=top_symbol_id,
                    conf=top_conf,
                    box=top_box
                )

                # Save updated verification stitch
                grid = self._detector.build_verification_grid()
                stitch_path = os.path.join(self._output_dir, "stitched_verification.jpg")
                cv2.imwrite(stitch_path, grid)

        # Publish annotated stream for visualization
        annotated = self._detector.draw_detections(frame, detections)
        try:
            annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self._annotated_pub.publish(annotated_msg)
        except Exception:
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
