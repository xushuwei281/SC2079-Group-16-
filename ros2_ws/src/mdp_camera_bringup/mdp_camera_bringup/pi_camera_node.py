#!/usr/bin/env python3
"""Hardware-ISP Camera Node for ROS 2.

Spawns the system Picamera2 daemon to leverage the Broadcom VideoCore ISP,
reads frames via zero-copy shared memory (/dev/shm/mdp_camera.raw), and publishes
crystal-clear, demosaiced frames to ROS 2 topics:
- /camera/image_raw (sensor_msgs/msg/Image, rgb8)
- /camera/image_raw/compressed (sensor_msgs/msg/CompressedImage, jpeg)
- /camera/camera_info (sensor_msgs/msg/CameraInfo)
"""

from __future__ import annotations

import mmap
import os
import subprocess
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

SHM_PATH = "/dev/shm/mdp_camera.raw"
FRAME_W = 640
FRAME_H = 480
FRAME_SIZE = FRAME_W * FRAME_H * 3


class PiCameraNode(Node):
    """ROS 2 Node bridging system Picamera2 shared memory to ROS topics."""

    def __init__(self) -> None:
        super().__init__("camera")

        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera_link")

        self._fps = float(self.get_parameter("fps").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        # Publishers
        self._raw_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._compressed_pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        # Launch the system Picamera2 daemon
        candidate_paths = [
            os.path.join(os.path.dirname(__file__), "system_camera_streamer.py"),
            "/home/mdp/dev/SC2079-Group-16/ros2_ws/src/mdp_camera_bringup/mdp_camera_bringup/system_camera_streamer.py",
        ]
        daemon_script = candidate_paths[0]
        for cp in candidate_paths:
            if os.path.exists(cp):
                daemon_script = cp
                break

        clean_env = os.environ.copy()
        clean_env.pop("PYTHONPATH", None)
        clean_env.pop("PYTHONHOME", None)

        self.get_logger().info(f"Starting hardware Picamera2 daemon ({daemon_script})...")
        self._daemon_proc = subprocess.Popen(
            ["/usr/bin/python3", daemon_script],
            env=clean_env,
        )

        # Wait for shared memory file to appear
        t0 = time.time()
        while not os.path.exists(SHM_PATH) or os.path.getsize(SHM_PATH) < FRAME_SIZE:
            time.sleep(0.1)
            if time.time() - t0 > 5.0:
                self.get_logger().error("Timeout waiting for camera shared memory buffer!")
                break

        # Open shared memory
        self._shm_fd = os.open(SHM_PATH, os.O_RDONLY)
        self._shm = mmap.mmap(self._shm_fd, FRAME_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)

        # Publishing loop
        timer_period = 1.0 / max(1.0, self._fps)
        self._timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info("Hardware ISP camera node ready! Publishing 640x480 RGB8 frames.")

    def _on_timer(self) -> None:
        if not hasattr(self, "_shm"):
            return

        self._shm.seek(0)
        raw_bytes = self._shm.read(FRAME_SIZE)
        if len(raw_bytes) < FRAME_SIZE or raw_bytes[0:10] == b"\x00" * 10:
            return  # Empty or uninitialized buffer

        now = self.get_clock().now().to_msg()

        # 1. Publish Raw RGB8 Image
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self._frame_id
        img_msg.height = FRAME_H
        img_msg.width = FRAME_W
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = 0
        img_msg.step = FRAME_W * 3
        img_msg.data = raw_bytes
        self._raw_pub.publish(img_msg)

        # 2. Publish Compressed JPEG Image
        if self._compressed_pub.get_subscription_count() > 0:
            frame_rgb = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((FRAME_H, FRAME_W, 3))
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            success, encoded_jpg = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if success:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = now
                comp_msg.header.frame_id = self._frame_id
                comp_msg.format = "jpeg"
                comp_msg.data = encoded_jpg.tobytes()
                self._compressed_pub.publish(comp_msg)

        # 3. Publish Camera Info
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = self._frame_id
        info_msg.height = FRAME_H
        info_msg.width = FRAME_W
        self._info_pub.publish(info_msg)

    def destroy_node(self) -> None:
        try:
            if hasattr(self, "_shm"):
                self._shm.close()
            if hasattr(self, "_shm_fd"):
                os.close(self._shm_fd)
            if hasattr(self, "_daemon_proc"):
                self._daemon_proc.terminate()
                self._daemon_proc.wait(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
