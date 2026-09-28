#!/usr/bin/env python3
"""Export a canonical checkpoint in a NEW candidate directory and verify its graph."""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import shutil

from train_baseline import EXPECTED_NAMES, digest, ordered_names


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    checkpoint = args.checkpoint.resolve()
    if not checkpoint.is_file():
        parser.error("Checkpoint must be an existing file")
    output = args.output_dir.resolve()
    if output.exists():
        parser.error("Candidate directory already exists; use a new directory")

    import onnx
    from ultralytics import YOLO

    model = YOLO(str(checkpoint), task="detect")
    if model.task != "detect" or ordered_names(model.names) != EXPECTED_NAMES:
        raise ValueError("Checkpoint must be a detector with canonical names 11..40 + marker")
    output.mkdir(parents=True)
    try:
        # Ultralytics writes alongside its checkpoint, so work on a candidate copy.
        staged = output / "best.pt"
        shutil.copy2(checkpoint, staged)
        selected = YOLO(str(staged), task="detect")
        exported = Path(
            selected.export(
                format="onnx",
                imgsz=640,
                batch=1,
                opset=18,
                dynamic=False,
                half=False,
                simplify=True,
                nms=False,
                device="cpu",
            )
        )
        graph = onnx.load(str(exported))
        onnx.checker.check_model(graph)
        inputs, outputs = graph.graph.input, graph.graph.output
        shape = lambda value: [d.dim_value for d in value.type.tensor_type.shape.dim]
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Expected one input and one output")
        if shape(inputs[0]) != [1, 3, 640, 640] or shape(outputs[0]) != [1, 35, 8400]:
            raise ValueError("Unexpected ONNX tensor layout")
        if inputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
            raise ValueError("Expected FP32 input")
        if outputs[0].type.tensor_type.elem_type != onnx.TensorProto.FLOAT:
            raise ValueError("Expected FP32 output")
        metadata = {item.key: item.value for item in graph.metadata_props}
        if ordered_names(ast.literal_eval(metadata["names"])) != EXPECTED_NAMES:
            raise ValueError("Exported metadata differs from checkpoint class contract")
        manifest = {
            "source_checkpoint_sha256": digest(checkpoint),
            "onnx_sha256": digest(exported),
            "onnx_bytes": exported.stat().st_size,
            "input_shape": shape(inputs[0]),
            "output_shape": shape(outputs[0]),
            "dtype": "float32",
            "opset": 18,
            "nms_embedded": False,
            "names": EXPECTED_NAMES,
            "validation": "Graph and metadata only; backend/image and Pi tests remain required",
        }
        (output / "model_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps(manifest, indent=2))
    except Exception:
        (output / "EXPORT_FAILED.txt").write_text("Incomplete candidate; do not deploy.\n")
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
