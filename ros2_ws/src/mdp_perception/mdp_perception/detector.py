#!/usr/bin/env python3
"""YOLOv8 Target Detection & Verification Grid Stitching Engine.

Maps raw YOLO class labels to official SC2079 MDP target IDs (11-40)
and maintains a live 3x3 verification stitch grid of photographed obstacles.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Official SC2079 Symbol IDs (Briefing & Protocol Spec)
# 11-19: Digits 1-9
# 20-35: Letters A-Z
# 36: Up Arrow
# 37: Down Arrow
# 38: Right Arrow
# 39: Left Arrow
# 40: Stop / Circle / Target Bullseye
_LABEL_TO_SYMBOL_ID: Dict[str, int] = {
    # Digits
    "1": 11, "2": 12, "3": 13, "4": 14, "5": 15,
    "6": 16, "7": 17, "8": 18, "9": 19,
    # Letters (A-Z)
    "a": 20, "b": 21, "c": 22, "d": 23, "e": 24,
    "f": 25, "g": 26, "h": 27, "s": 28, "t": 29,
    "u": 30, "v": 31, "w": 32, "x": 33, "y": 34, "z": 35,
    # Directional Arrows
    "up": 36,
    "down": 37,
    "right": 38,
    "left": 39,
    # Target / Circle / Stop
    "circle": 40,
    "target": 40,
}


class TargetDetector:
    """YOLO Target Detection and Verification Grid Builder."""

    def __init__(
        self,
        model_path: str = "models/best.pt",
        conf_threshold: float = 0.50,
        iou_threshold: float = 0.45
    ) -> None:
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self._model = None

        # Recognized Targets Stored for Verification Grid: {obstacle_id: (crop_img, symbol_id, conf)}
        self.recognized_crops: Dict[int, Tuple[np.ndarray, int, float]] = {}

        self._load_model()

    def _load_model(self) -> None:
        """Load YOLO model weights (supports PyTorch .pt, ONNX, TensorRT)."""
        if not os.path.exists(self.model_path) and not os.path.isabs(self.model_path):
            # pixi tasks run with cwd=ros2_ws/, but models/ lives at the repo
            # root (one level up) -- not under ros2_ws/. __file__-relative
            # search doesn't help either: when running the *installed*
            # package (the normal `ros2 run` path), __file__ points into
            # ros2_ws/install/mdp_perception/lib/.../site-packages/, which
            # has no fixed relationship to the repo root.
            candidates = [
                os.path.join(os.getcwd(), "..", self.model_path),  # cwd=ros2_ws/ -> repo root
                os.path.join(os.getcwd(), self.model_path),  # cwd already at repo root
            ]
            for candidate in candidates:
                if os.path.exists(candidate):
                    self.model_path = os.path.abspath(candidate)
                    break

        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
            print(f"[TargetDetector] Successfully loaded YOLO weights from {self.model_path}")
        except Exception as exc:
            print(f"[TargetDetector] Warning: Could not load YOLO model ({exc}). Running in mock mode.")
            self._model = None

    def predict(
        self,
        frame: np.ndarray
    ) -> List[Tuple[str, int, float, Tuple[int, int, int, int]]]:
        """Run YOLO inference on a single image frame (BGR format).
        
        Returns:
            List of (class_name, symbol_id, confidence, (x1, y1, x2, y2))
        """
        if self._model is None:
            return []

        results = self._model.predict(
            source=frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False
        )

        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                raw_name = str(self._model.names.get(cls_id, str(cls_id))).lower()

                # Map class to official MDP symbol ID
                symbol_id = _LABEL_TO_SYMBOL_ID.get(raw_name, 10 + cls_id)

                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]

                detections.append((raw_name, symbol_id, conf, (x1, y1, x2, y2)))

        return detections

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[Tuple[str, int, float, Tuple[int, int, int, int]]]
    ) -> np.ndarray:
        """Annotate frame with bounding boxes and labels."""
        annotated = frame.copy()
        for raw_name, symbol_id, conf, (x1, y1, x2, y2) in detections:
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 120), 2)
            # Label banner
            label = f"ID:{symbol_id} ({raw_name}) {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x1, max(0, y1 - th - 6)), (x1 + tw + 6, y1), (0, 255, 120), -1)
            cv2.putText(annotated, label, (x1 + 3, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        return annotated

    def register_target_crop(
        self,
        obstacle_id: int,
        frame: np.ndarray,
        symbol_id: int,
        conf: float,
        box: Tuple[int, int, int, int]
    ) -> None:
        """Store the best cropped snapshot of an obstacle for the verification grid."""
        x1, y1, x2, y2 = box
        # Add 10% padding around box
        h, w = frame.shape[:2]
        pad_x = int((x2 - x1) * 0.1)
        pad_y = int((y2 - y1) * 0.1)
        cx1 = max(0, x1 - pad_x)
        cy1 = max(0, y1 - pad_y)
        cx2 = min(w, x2 + pad_x)
        cy2 = min(h, y2 + pad_y)

        crop = frame[cy1:cy2, cx1:cx2].copy()
        crop_resized = cv2.resize(crop, (200, 200))

        # Annotate with obstacle and symbol ID
        banner = np.zeros((40, 200, 3), dtype=np.uint8)
        cv2.putText(banner, f"Obs {obstacle_id}: ID {symbol_id}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        crop_with_banner = np.vstack((banner, crop_resized))

        self.recognized_crops[obstacle_id] = (crop_with_banner, symbol_id, conf)

    def build_verification_grid(self) -> np.ndarray:
        """Create a 3x3 stitched verification grid of all photographed obstacles."""
        cell_w, cell_h = 200, 240
        grid = np.zeros((cell_h * 3, cell_w * 3, 3), dtype=np.uint8)

        # Place cells 1 to 9 (or up to 8 obstacles)
        for i in range(1, 10):
            row = (i - 1) // 3
            col = (i - 1) % 3
            y = row * cell_h
            x = col * cell_w

            if i in self.recognized_crops:
                cell_img = self.recognized_crops[i][0]
                grid[y:y + cell_h, x:x + cell_w] = cell_img
            else:
                # Blank placeholder
                cv2.rectangle(grid, (x + 2, y + 2), (x + cell_w - 2, y + cell_h - 2), (40, 40, 40), -1)
                cv2.putText(grid, f"Obs {i}: --", (x + 40, y + 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)

        return grid
