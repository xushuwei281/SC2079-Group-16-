"""Class and tensor contract for the canonical SC2079 detection model."""

from __future__ import annotations

import ast
import json
from typing import Dict, List, Optional


# The model output index is not the Android assessment ID.
# 11-19: digits 1-9; 20-35: A-H and S-Z; 36-39: arrows; 40: filled circle.
_LABEL_TO_SYMBOL_ID: Dict[str, int] = {
    "1": 11, "2": 12, "3": 13, "4": 14, "5": 15,
    "6": 16, "7": 17, "8": 18, "9": 19,
    "a": 20, "b": 21, "c": 22, "d": 23, "e": 24,
    "f": 25, "g": 26, "h": 27, "s": 28, "t": 29,
    "u": 30, "v": 31, "w": 32, "x": 33, "y": 34, "z": 35,
    "up": 36, "down": 37, "right": 38, "left": 39,
    "circle": 40, "stop": 40,
}

# Historical "target" labels are excluded, never assigned to filled-circle 40.
# Confirm historical dataset images before migrating any legacy annotations.
_MARKER_LABELS = frozenset({"marker", "bullseye", "bulls_eye", "bulls-eye", "target"})
_CANONICAL_MODEL_NAMES = tuple([str(value) for value in range(11, 41)] + ["marker"])


def _resolve_symbol_id(label: str) -> Optional[int]:
    """Resolve a label without ever inventing an assessment ID."""
    name = str(label).strip().lower()
    if name in _MARKER_LABELS:
        return None
    if name.isascii() and name.isdigit() and 11 <= int(name) <= 40:
        return int(name)
    return _LABEL_TO_SYMBOL_ID.get(name)


def _normalize_class_names(names: object) -> List[str]:
    """Accept a names list or a dictionary with contiguous integer indices."""
    if isinstance(names, dict):
        indexed = {}
        for key, value in names.items():
            if type(key) is int:
                index = key
            elif isinstance(key, str) and key.isascii() and key.isdigit():
                index = int(key)
            else:
                raise ValueError(f"Invalid class index in model names: {key!r}")
            if index in indexed:
                raise ValueError(f"Duplicate class index in model names: {index}")
            indexed[index] = value
        if set(indexed) != set(range(len(indexed))):
            raise ValueError("Model class indices must be contiguous from zero")
        result = [indexed[index] for index in range(len(indexed))]
    elif isinstance(names, list):
        result = list(names)
    else:
        raise ValueError("Model names must be a list or indexed dictionary")
    if not result or any(not isinstance(name, str) for name in result):
        raise ValueError("Model class names must be nonempty strings")
    if any(not name for name in result):
        raise ValueError("Model class names must not be empty")
    return result


def _validate_model_names(names: object) -> List[str]:
    """Enforce the deployed schema: names 11 through 40 followed by marker."""
    normalized = _normalize_class_names(names)
    if tuple(normalized) != _CANONICAL_MODEL_NAMES:
        raise ValueError(
            "Canonical class contract mismatch: expected 31 ordered names "
            "'11' through '40', then 'marker'; got " + repr(normalized)
        )
    return normalized


def _parse_onnx_class_names(raw_names: Optional[str]) -> List[str]:
    """Parse exported ONNX names metadata safely, then validate its order."""
    if not isinstance(raw_names, str) or not raw_names.strip():
        raise ValueError("ONNX model is missing its required 'names' metadata")
    try:
        parsed = json.loads(raw_names)
    except json.JSONDecodeError:
        try:
            parsed = ast.literal_eval(raw_names)
        except (ValueError, SyntaxError) as exc:
            raise ValueError("ONNX 'names' metadata is malformed") from exc
    return _validate_model_names(parsed)


def _validate_onnx_contract(session: object) -> List[str]:
    """Check the exact static FP32 raw-detection layout this runtime parses."""
    metadata = session.get_modelmeta().custom_metadata_map
    names = _parse_onnx_class_names(metadata.get("names"))
    if metadata.get("task", "detect") != "detect":
        raise ValueError("ONNX model must be an object-detection model")
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError("Expected exactly one ONNX input and one raw detection output")
    if inputs[0].type != "tensor(float)" or list(inputs[0].shape) != [1, 3, 640, 640]:
        raise ValueError(
            "Expected FP32 ONNX input [1, 3, 640, 640]; re-export with "
            "imgsz=640, batch=1, dynamic=False, half=False"
        )
    if outputs[0].type != "tensor(float)" or list(outputs[0].shape) != [1, 35, 8400]:
        raise ValueError(
            "Expected FP32 raw YOLOv8 detection output [1, 35, 8400]; "
            "use the canonical detection model and export with nms=False"
        )
    return names
