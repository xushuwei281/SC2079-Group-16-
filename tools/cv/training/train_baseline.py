#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11,<3.13"
# dependencies = [
#   "ultralytics-opencv-headless==8.4.128", "PyYAML==6.0.3",
#   "torch==2.14.0", "torchvision==0.29.0",
# ]
# ///
"""Explicit local/HF-worker training entrypoint; never submits jobs or replaces outputs."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import platform
import shutil
import tempfile
from typing import Any

import yaml

EXPECTED_NAMES = [str(value) for value in range(11, 41)] + ["marker"]
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def ordered_names(value: list | dict) -> list[str]:
    if isinstance(value, list):
        return [str(item) for item in value]
    if not isinstance(value, dict):
        raise ValueError("Class names must be a list or mapping")
    indices = {int(key): str(item) for key, item in value.items()}
    if len(indices) != len(value) or set(indices) != set(range(len(indices))):
        raise ValueError("Class indices must be contiguous and unique from zero")
    return [indices[index] for index in range(len(indices))]


def validate_initialization(model: str, initialization: str) -> bool:
    """Separate official pretrained seeds from canonical custom checkpoints explicitly."""
    if initialization == "pretrained":
        if model not in {"yolov8n.pt", "yolov8m.pt"}:
            raise ValueError("Pretrained initialization requires yolov8n.pt or yolov8m.pt")
        return True
    if initialization != "custom":
        raise ValueError("Initialization must be pretrained or custom")
    return False


def safe_child(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    if not candidate.is_relative_to(root.resolve()):
        raise ValueError(f"Manifest path escapes dataset: {relative}")
    return candidate


def verify_snapshot(root: Path, manifest: dict[str, Any], allow_issues: bool) -> None:
    """Verify audited bytes, exact image inventory, and permitted historical issues."""
    config = yaml.safe_load((root / "data.yaml").read_text())
    if ordered_names(config["names"]) != EXPECTED_NAMES or int(config.get("nc", 31)) != 31:
        raise ValueError("Dataset must have exactly the canonical names 11..40 + marker")
    if digest(root / "data.yaml") != manifest["data_yaml_sha256"]:
        raise ValueError("data.yaml changed after audit")
    errors = manifest.get("errors", [])
    structural = [error for error in errors if not error.startswith("Conflicting annotations")]
    if structural:
        raise ValueError(f"Dataset has structural errors: {structural[:3]}")
    duplicate_groups = manifest.get("duplicate_groups", {})
    cross_split = any(
        group["cross_split"] for groups in duplicate_groups.values() for group in groups
    )
    empty_labels = any(record["label"]["empty"] for record in manifest["images"])
    if (errors or cross_split or empty_labels) and not allow_issues:
        raise ValueError(
            "Annotation conflicts, cross-split duplicates or unverified empty labels remain; "
            "repair the dataset or explicitly document --allow-known-data-issues"
        )
    expected = set()
    for record in manifest["images"]:
        expected.add(record["path"])
        for relative, expected_hash in (
            (record["path"], record["file_sha256"]),
            (record["label_path"], record["label"]["sha256"]),
        ):
            if digest(safe_child(root, relative)) != expected_hash:
                raise ValueError(f"File changed after audit: {relative}")
    actual = set()
    for split in ("train", "valid", "test"):
        image_root = root / split / "images"
        if not image_root.is_dir() or not (root / split / "labels").is_dir():
            raise ValueError(f"Missing split directories: {split}")
        images = [p for p in image_root.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES]
        if not images:
            raise ValueError(f"Empty split: {split}")
        actual.update(p.relative_to(root).as_posix() for p in images)
    if expected != actual:
        raise ValueError("Image inventory changed after audit")


def training_settings(args: argparse.Namespace, output: Path, data: Path) -> dict[str, Any]:
    return {
        "data": str(data),
        "imgsz": 640,
        "epochs": args.epochs,
        "patience": 30,
        "batch": args.batch,
        "device": args.device,
        "workers": args.workers,
        "amp": True,
        "cache": False,
        "seed": 16,
        "deterministic": True,
        "fliplr": 0.0,
        "flipud": 0.0,
        "degrees": 0.0,
        "hsv_s": 0.3,
        "hsv_v": 0.2,
        "project": str(output / "runs" / "detect"),
        "name": args.name,
        "exist_ok": False,
        "save": True,
        "save_period": 10,
        "plots": True,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--audit-manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--initialization", choices=("pretrained", "custom"), default="pretrained")
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument("--name", required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--dataset-ref", required=True)
    parser.add_argument("--allow-known-data-issues", action="store_true")
    parser.add_argument("--dataset-note", default="")
    parser.add_argument("--require-output-marker", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if Path(args.name).name != args.name or args.name in {"", ".", ".."}:
        parser.error("--name must be one directory name")
    if args.epochs < 1 or args.batch < 1 or args.workers < 0:
        parser.error("epochs/batch must be positive; workers must be nonnegative")
    if args.allow_known_data_issues and not args.dataset_note.strip():
        parser.error("--allow-known-data-issues requires a meaningful --dataset-note")
    official_seed = validate_initialization(args.model, args.initialization)
    root, output = args.data_root.resolve(), args.output_root.resolve()
    if root == output or root in output.parents or output in root.parents:
        parser.error("Dataset and output must be separate, non-nested directories")
    manifest_path = args.audit_manifest.resolve()
    manifest = json.loads(manifest_path.read_text())
    verify_snapshot(root, manifest, args.allow_known_data_issues)
    if args.require_output_marker and not (output / ".mdp-output-ready.json").is_file():
        raise ValueError("Persistent output marker is missing; verify the mounted output bucket")
    record = output / "records" / args.name
    config_path = output / "configs" / f"{args.name}.yaml"
    for path in (record, config_path, output / "runs" / "detect" / args.name):
        if path.exists():
            raise FileExistsError(f"Existing experiment; choose a new --name: {path}")
    if args.dry_run:
        print(json.dumps(training_settings(args, output, root / "data.yaml"), indent=2))
        print("Verified audited snapshot. No training, model download, or output writes performed.")
        return 0

    import torch
    from ultralytics import YOLO

    if args.device not in {"cpu", "mps"} and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; inspect the selected worker before training")
    model_arg = args.model
    if Path(model_arg).is_file():
        model_arg = str(Path(model_arg).resolve())
    previous_cwd = Path.cwd()
    # Stage a writable copy so Ultralytics cache files never modify the audited input.
    with tempfile.TemporaryDirectory(prefix="mdp-training-") as temporary:
        work = Path(temporary)
        local_data = work / "dataset"
        shutil.copytree(root, local_data, ignore=shutil.ignore_patterns("*.cache"))
        verify_snapshot(local_data, manifest, args.allow_known_data_issues)
        record.mkdir(parents=True)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(
            yaml.safe_dump(
                {
                    "path": str(local_data),
                    "train": "train/images",
                    "val": "valid/images",
                    "test": "test/images",
                    "nc": 31,
                    "names": dict(enumerate(EXPECTED_NAMES)),
                },
                sort_keys=False,
            )
        )
        shutil.copy2(manifest_path, record / "dataset-manifest.json")
        shutil.copy2(root / "data.yaml", record / "source-data.yaml")
        shutil.copy2(Path(__file__).resolve(), record / "training-script.py")
        try:
            os.chdir(work)
            model = YOLO(model_arg, task="detect")
            if model.task != "detect":
                raise ValueError("Checkpoint is not an object detector")
            if not official_seed and ordered_names(model.names) != EXPECTED_NAMES:
                raise ValueError("Custom checkpoints must already have canonical class names")
            provenance = {
                "arguments": {
                    key: str(value) if isinstance(value, Path) else value
                    for key, value in vars(args).items()
                },
                "python": platform.python_version(),
                "torch_cuda": torch.version.cuda,
                "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
                "packages": {
                    dist.metadata["Name"]: dist.version
                    for dist in metadata.distributions()
                    if dist.metadata.get("Name")
                },
                "starting_checkpoint_sha256": digest(Path(model.ckpt_path)),
                "dataset_manifest_sha256": digest(manifest_path),
                "expected_names": EXPECTED_NAMES,
                "split_independence": "Not established by hash verification",
            }
            (record / "environment.json").write_text(json.dumps(provenance, indent=2) + "\n")
            model.train(**training_settings(args, output, config_path))
            best = Path(model.trainer.best)
            selected = YOLO(str(best), task="detect")
            if ordered_names(selected.names) != EXPECTED_NAMES:
                raise ValueError("Trained checkpoint has an incompatible class contract")
            (record / "completed.json").write_text(
                json.dumps(
                    {
                        "best_checkpoint": str(best),
                        "best_sha256": digest(best),
                        "names": EXPECTED_NAMES,
                        "run_directory": str(model.trainer.save_dir),
                        "limitations": args.dataset_note,
                        "export_status": "Export and parity are separate required steps",
                    },
                    indent=2,
                )
                + "\n"
            )
        finally:
            os.chdir(previous_cwd)
    print(f"Training complete: {record / 'completed.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
