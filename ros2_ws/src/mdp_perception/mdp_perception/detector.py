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
# 40: Stop (the "circle" training label)
#
# "target" is a distinct dataset class from "circle": it's the bullseye
# orbit-recovery fixture, not a numbered competition symbol, so it does NOT
# share ID 40 with Stop -- it gets sentinel 0 (outside 11-40) and is matched
# by name, not by ID, wherever is_marker is decided (see perception_node's
# _evaluate_consensus and planner_node's sample-result handling).
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
    # Stop sign
    "circle": 40,
    # Bullseye orbit-recovery marker -- not a competition symbol, sentinel
    # value only (see module docstring above).
    "target": 0,
}


class TargetDetector:
    """YOLO Target Detection and Verification Grid Builder (Supports ONNXRuntime and PyTorch)."""

    def __init__(
        self,
        model_path: str = "models/best.onnx",
        conf_threshold: float = 0.50,
        iou_threshold: float = 0.45,
    ) -> None:
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self._onnx_session = None
        self._input_name = None
        self._pt_model = None
        self._class_names = [
            "1", "2", "3", "4", "5", "6", "7", "8", "9",
            "a", "b", "c", "circle", "d", "down", "e", "f", "g", "h",
            "left", "right", "s", "t", "target", "u", "up", "v", "w", "x", "y", "z"
        ]

        # Recognized Targets Stored for Verification Grid: {obstacle_id: (crop_img, symbol_id, conf)}
        self.recognized_crops: Dict[int, Tuple[np.ndarray, int, float]] = {}

        self._load_model()

    def _load_model(self) -> None:
        """Load YOLO model weights (supports ONNX via onnxruntime and PyTorch .pt via ultralytics)."""
        candidates = []
        if os.path.isabs(self.model_path):
            candidates.append(self.model_path)
        else:
            base_name = os.path.splitext(self.model_path)[0]
            # Try ONNX first (lightweight), then PyTorch .pt
            for ext in [".onnx", ".pt"]:
                p = base_name + ext
                candidates.extend([
                    os.path.abspath(os.path.join(os.getcwd(), p)),
                    os.path.abspath(os.path.join(os.getcwd(), "..", p)),
                    os.path.abspath(os.path.join("/home/mdp/dev/SC2079-Group-16", p)),
                ])

        found_path = None
        for c in candidates:
            if os.path.exists(c):
                found_path = c
                break

        if found_path is None:
            print(f"[TargetDetector] Warning: Model file not found in candidates: {candidates[:3]}. Running in mock mode.")
            return

        self.model_path = found_path

        # 1. Try ONNX Runtime (fast, lean edge inference on Pi)
        if found_path.endswith(".onnx"):
            try:
                import onnxruntime as ort
                opts = ort.SessionOptions()
                # 2 threads is faster on Cortex-A72 than 4 (avoids L2 cache contention and thermal throttling)
                # and leaves 2 cores free for ROS2 middleware and hardware bridges.
                opts.intra_op_num_threads = 2
                opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                opts.enable_cpu_mem_arena = True
                opts.enable_mem_pattern = True
                self._onnx_session = ort.InferenceSession(found_path, sess_options=opts, providers=["CPUExecutionProvider"])
                self._input_name = self._onnx_session.get_inputs()[0].name

                # Parse class names dynamically from ONNX metadata if available
                self._meta_names = None
                meta = self._onnx_session.get_modelmeta()
                if meta and "names" in meta.custom_metadata_map:
                    import ast
                    try:
                        self._meta_names = ast.literal_eval(meta.custom_metadata_map["names"])
                    except Exception:
                        pass

                print(f"[TargetDetector] Successfully loaded ONNX model via ONNXRuntime from {found_path}")
                return
            except Exception as exc:
                print(f"[TargetDetector] Failed to load ONNX with onnxruntime: {exc}")

        # 2. Try Ultralytics PyTorch fallback (if available, e.g. on laptop/PC)
        try:
            from ultralytics import YOLO
            self._pt_model = YOLO(found_path)
            print(f"[TargetDetector] Successfully loaded PyTorch model via Ultralytics from {found_path}")
        except Exception as exc:
            print(f"[TargetDetector] Warning: Could not load model ({exc}). Running in mock mode.")

    def _infer_onnx(self, frame: np.ndarray) -> List[Tuple[str, int, float, Tuple[int, int, int, int]]]:
        """Pure-NumPy YOLOv8 ONNX inference pipeline."""
        orig_h, orig_w = frame.shape[:2]
        inp_shape = self._onnx_session.get_inputs()[0].shape
        img_size = inp_shape[2] if len(inp_shape) > 2 and isinstance(inp_shape[2], int) else 640

        # Letterbox resize maintaining aspect ratio
        scale = min(img_size / orig_h, img_size / orig_w)
        nw, nh = int(round(orig_w * scale)), int(round(orig_h * scale))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

        top_pad = (img_size - nh) // 2
        bottom_pad = img_size - nh - top_pad
        left_pad = (img_size - nw) // 2
        right_pad = img_size - nw - left_pad

        padded = cv2.copyMakeBorder(resized, top_pad, bottom_pad, left_pad, right_pad, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # BGR -> RGB, HWC -> CHW, normalize [0, 1]
        blob = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        blob = np.expand_dims(blob, axis=0)

        # Run ONNX session
        outputs = self._onnx_session.run(None, {self._input_name: blob})[0]  # Shape: (1, 4+num_classes, 8400)
        predictions = outputs[0].T  # Shape: (8400, 4+num_classes)

        boxes = []
        confidences = []
        class_ids = []

        scores_matrix = predictions[:, 4:]
        max_scores = np.max(scores_matrix, axis=1)
        valid_mask = max_scores >= self.conf_threshold

        valid_preds = predictions[valid_mask]
        if len(valid_preds) == 0:
            return []

        valid_scores = scores_matrix[valid_mask]
        valid_cls = np.argmax(valid_scores, axis=1)
        valid_conf = np.max(valid_scores, axis=1)

        for i in range(len(valid_preds)):
            cx, cy, w, h = valid_preds[i, 0], valid_preds[i, 1], valid_preds[i, 2], valid_preds[i, 3]
            # Convert cx, cy, w, h in padded image to unpadded original coordinates
            x1 = int(round((cx - w / 2.0 - left_pad) / scale))
            y1 = int(round((cy - h / 2.0 - top_pad) / scale))
            bw = int(round(w / scale))
            bh = int(round(h / scale))

            # Clamp
            x1 = max(0, min(orig_w - 1, x1))
            y1 = max(0, min(orig_h - 1, y1))
            bw = max(1, min(orig_w - x1, bw))
            bh = max(1, min(orig_h - y1, bh))

            boxes.append([x1, y1, bw, bh])
            confidences.append(float(valid_conf[i]))
            class_ids.append(int(valid_cls[i]))

        # Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.iou_threshold)

        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                cid = class_ids[idx]
                conf = confidences[idx]
                bx, by, bw, bh = boxes[idx]
                x1, y1, x2, y2 = bx, by, bx + bw, by + bh

                if self._meta_names and cid in self._meta_names:
                    raw_name = str(self._meta_names[cid])
                elif cid < len(self._class_names):
                    raw_name = self._class_names[cid]
                else:
                    raw_name = str(cid)

                if raw_name.isdigit() and 11 <= int(raw_name) <= 40:
                    symbol_id = int(raw_name)
                else:
                    symbol_id = _LABEL_TO_SYMBOL_ID.get(raw_name, 10 + cid)

                detections.append((raw_name, symbol_id, conf, (x1, y1, x2, y2)))

        return detections

    def predict(
        self,
        frame: np.ndarray
    ) -> List[Tuple[str, int, float, Tuple[int, int, int, int]]]:
        """Run YOLO inference on a single image frame (BGR format).
        
        Returns:
            List of (class_name, symbol_id, confidence, (x1, y1, x2, y2))
        """
        if self._onnx_session is not None:
            return self._infer_onnx(frame)

        if self._pt_model is not None:
            results = self._pt_model.predict(
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
                    raw_name = str(self._pt_model.names.get(cls_id, str(cls_id))).lower()

                    if raw_name.isdigit() and 11 <= int(raw_name) <= 40:
                        symbol_id = int(raw_name)
                    else:
                        symbol_id = _LABEL_TO_SYMBOL_ID.get(raw_name, 10 + cls_id)

                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]

                    detections.append((raw_name, symbol_id, conf, (x1, y1, x2, y2)))

            return detections

        return []

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
