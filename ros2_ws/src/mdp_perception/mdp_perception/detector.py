#!/usr/bin/env python3
"""YOLOv8 Target Detection & Verification Grid Stitching Engine.

Maps raw YOLO class labels to official SC2079 MDP target IDs (11-40)
and maintains a live 3x3 verification stitch grid of photographed obstacles.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from mdp_perception.class_contract import (
    _resolve_symbol_id,
    _validate_model_names,
    _validate_onnx_contract,
)


class TargetDetector:
    """YOLO Target Detection and Verification Grid Builder (Supports ONNXRuntime and PyTorch)."""

    def __init__(
        self,
        model_path: str = "models/best.onnx",
        conf_threshold: float = 0.50,
        iou_threshold: float = 0.45,
        onnx_threads: int = 4,
    ) -> None:
        if type(onnx_threads) is not int or onnx_threads < 1:
            raise ValueError("onnx_threads must be a positive integer")
        self.onnx_threads = onnx_threads
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self._onnx_session = None
        self._input_name = None
        self._pt_model = None
        self._class_names: List[str] = []

        # Recognized Targets Stored for Verification Grid: {obstacle_id: (crop_img, symbol_id, conf)}
        self.recognized_crops: Dict[int, Tuple[np.ndarray, int, float]] = {}

        self._load_model()
        for index, name in enumerate(self._class_names):
            print(
                f"[TargetDetector] index {index} -> name {name!r} -> "
                f"assessment ID {_resolve_symbol_id(name)}"
            )

    def _load_model(self) -> None:
        """Load and validate the explicitly requested model; never use mock mode."""
        if os.path.isabs(self.model_path):
            candidates = [self.model_path]
        else:
            # Permit repository-root and ros2_ws working directories, preserving
            # the exact requested extension. Use an absolute path for deployment.
            candidates = [
                os.path.abspath(self.model_path),
                os.path.abspath(os.path.join(os.getcwd(), "..", self.model_path)),
            ]
        found_path = next((path for path in candidates if os.path.isfile(path)), None)
        if found_path is None:
            raise FileNotFoundError(f"Requested model was not found: {candidates}")
        self.model_path = found_path
        extension = os.path.splitext(found_path)[1].lower()

        if extension == ".onnx":
            import onnxruntime as ort

            opts = ort.SessionOptions()
            opts.intra_op_num_threads = self.onnx_threads
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            session = ort.InferenceSession(
                found_path,
                sess_options=opts,
                providers=["CPUExecutionProvider"],
            )
            self._class_names = _validate_onnx_contract(session)
            self._input_name = session.get_inputs()[0].name
            self._onnx_session = session
            print(f"[TargetDetector] Loaded validated ONNX model: {found_path}")
            return

        if extension == ".pt":
            from ultralytics import YOLO

            model = YOLO(found_path)
            if model.task != "detect":
                raise ValueError("PyTorch model must be an object-detection model")
            self._class_names = _validate_model_names(model.names)
            self._pt_model = model
            print(f"[TargetDetector] Loaded validated PyTorch model: {found_path}")
            return

        raise ValueError(f"Unsupported model extension {extension!r}; use .onnx or .pt")

    def _infer_onnx(self, frame: np.ndarray) -> List[Tuple[str, int, float, Tuple[int, int, int, int]]]:
        """Pure-NumPy YOLOv8 ONNX inference pipeline."""
        orig_h, orig_w = frame.shape[:2]
        img_size = 640

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

                if not 0 <= cid < len(self._class_names):
                    raise RuntimeError(f"ONNX returned out-of-range class index {cid}")
                raw_name = self._class_names[cid]
                symbol_id = _resolve_symbol_id(raw_name)
                if symbol_id is None:
                    continue

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
                    if not 0 <= cls_id < len(self._class_names):
                        raise RuntimeError(f"PyTorch returned out-of-range class index {cls_id}")
                    raw_name = self._class_names[cls_id]
                    symbol_id = _resolve_symbol_id(raw_name)
                    if symbol_id is None:
                        continue

                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]

                    detections.append((raw_name, symbol_id, conf, (x1, y1, x2, y2)))

            return detections

        raise RuntimeError("No validated inference backend is loaded")

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
