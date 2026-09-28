#!/usr/bin/env python3
"""Compare PT and ONNX on frozen images, optionally including an explicit ROS detector.

This writes a new report and returns nonzero for disagreement or contract failure.
No camera, ROS node, robot process or network job is started.
"""
from __future__ import annotations
import argparse
import json
import sys
from typing import Any
from pathlib import Path


def iou(a: list[float], b: list[float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-12)


def differences(
    left: list[dict], right: list[dict], box_iou: float, score_tolerance: float
) -> list[dict]:
    """Greedy geometry matching for small controlled scenes; not a mAP evaluator."""
    remaining = list(range(len(right)))
    problems = []
    for item in sorted(left, key=lambda d: -d["confidence"]):
        if not remaining:
            problems.append({"missing_match": item})
            continue
        j = max(remaining, key=lambda k: iou(item["box"], right[k]["box"]))
        overlap = iou(item["box"], right[j]["box"])
        if overlap < box_iou:
            problems.append({"no_box_match": item, "best_iou": overlap})
            continue
        remaining.remove(j)
        other = right[j]
        if item["name"] != other["name"]:
            problems.append({"class_disagreement": [item, other]})
        if abs(item["confidence"] - other["confidence"]) > score_tolerance:
            problems.append({"confidence_disagreement": [item, other]})
    problems.extend({"extra_detection": right[j]} for j in remaining)
    return problems


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pt", required=True, type=Path)
    p.add_argument("--onnx", required=True, type=Path)
    p.add_argument("--images", required=True, type=Path)
    p.add_argument("--report", required=True, type=Path)
    p.add_argument("--conf", type=float, default=0.50)
    p.add_argument("--nms-iou", type=float, default=0.45)
    p.add_argument("--box-iou", type=float, default=0.90)
    p.add_argument("--score-tolerance", type=float, default=0.02)
    p.add_argument(
        "--check-folders",
        action="store_true",
        help=(
            "Expect one official target in folders 11..40 and "
            "no official output in marker/background folders."
        ),
    )
    p.add_argument(
        "--detector-source",
        type=Path,
        help="Directory containing the candidate mdp_perception package",
    )
    args = p.parse_args(argv)
    for value in (args.conf, args.nms_iou, args.box_iou, args.score_tolerance):
        if not 0 <= value <= 1:
            p.error("Thresholds and tolerances must be between zero and one")
    if args.report.exists():
        p.error("Report exists; choose a new path.")
    for path in (args.pt, args.onnx):
        if not path.is_file():
            p.error(f"Missing model: {path}")

    import cv2
    from ultralytics import YOLO

    expected_names = [str(i) for i in range(11, 41)] + ["marker"]
    pt = YOLO(str(args.pt.resolve()), task="detect")
    onnx = YOLO(str(args.onnx.resolve()), task="detect")
    custom = None
    if args.detector_source:
        sys.path.insert(0, str(args.detector_source.resolve()))
        from mdp_perception.detector import TargetDetector

        custom = TargetDetector(str(args.onnx.resolve()), args.conf, args.nms_iou)
        if getattr(custom, "_onnx_session", None) is None:
            raise ValueError("Candidate detector did not load ONNX; fallback/mock is not parity")
        if Path(custom.model_path).resolve() != args.onnx.resolve():
            raise ValueError("Candidate detector selected a different model path")
    if [str(pt.names[i]) for i in range(len(pt.names))] != expected_names:
        raise ValueError("PT metadata is not canonical.")
    files = sorted(
        x
        for x in args.images.rglob("*")
        if x.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    )
    if not files:
        raise ValueError("No frozen images found.")

    def predict(model: Any, frame: Any) -> list[dict]:
        results = model.predict(
            source=frame,
            imgsz=640,
            rect=False,
            conf=args.conf,
            iou=args.nms_iou,
            agnostic_nms=True,
            device="cpu",
            verbose=False,
        )
        result = results[0]
        names = [str(result.names[i]) for i in range(len(result.names))]
        if names != expected_names:
            raise ValueError("Backend metadata is not canonical.")
        return [
            {
                "name": names[int(b.cls.item())],
                "confidence": float(b.conf.item()),
                "box": b.xyxy[0].cpu().tolist(),
            }
            for b in result.boxes
        ]

    rows = []
    for file in files:
        frame = cv2.imread(str(file))
        if frame is None:
            raise ValueError(f"Cannot decode {file}")
        a, b = predict(pt, frame), predict(onnx, frame)
        official_reference = [d for d in b if d["name"] != "marker"]
        c = (
            None
            if custom is None
            else [
                {
                    "name": str(symbol_id),
                    "confidence": float(score),
                    "box": [int(v) for v in box],
                    "raw_name": name,
                }
                for name, symbol_id, score, box in custom.predict(frame)
                if 11 <= symbol_id <= 40
            ]
        )
        issues = {
            "pt_vs_onnx": differences(a, b, args.box_iou, args.score_tolerance),
            "onnx_vs_custom": (
                []
                if c is None
                else differences(official_reference, c, args.box_iou, args.score_tolerance)
            ),
        }
        expected = None
        if args.check_folders:
            label = file.relative_to(args.images).parts[0]
            if label not in expected_names + ["background"]:
                raise ValueError(f"Unrecognized expected-label folder: {label}")
            expected = label if label not in {"marker", "background"} else None
            observed = [
                d["name"]
                for d in sorted(
                    official_reference if c is None else c, key=lambda d: -d["confidence"]
                )
            ]
            if expected is None and observed:
                issues["expected_target"] = {"expected": [], "found": observed}
            elif expected is not None and (len(observed) != 1 or observed[0] != expected):
                issues["expected_target"] = {"expected": [expected], "found": observed}
        row = {
            "image": str(file.relative_to(args.images)),
            "expected": expected,
            "pt": a,
            "onnx": b,
            "custom": c,
            "issues": issues,
            "pass": not any(issues.values()),
        }
        rows.append(row)
        print(("PASS " if row["pass"] else "REVIEW ") + row["image"])
    report = {
        "settings": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "custom_backend_tested": custom is not None,
        "images": len(rows),
        "passed": sum(r["pass"] for r in rows),
        "rows": rows,
        "limitations": (
            "Controlled small scenes; greedy box matching, no latency or mAP measurement. "
            "Tolerances are review triggers, not universal numerical guarantees."
        ),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Report: {args.report}; {report['passed']}/{len(rows)} pass")
    return 0 if all(row["pass"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
