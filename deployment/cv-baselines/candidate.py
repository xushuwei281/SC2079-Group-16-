#!/usr/bin/env python3
"""Verify, package, inspect and manually run historical stationary CV candidates."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import statistics
import sys
from typing import Any


ROOT = Path(__file__).resolve().parent


def sha256(path: Path) -> str:
    """Hash a file in bounded memory, including large ONNX weights."""
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def load_profile(name: str) -> dict[str, Any]:
    """Return the recorded immutable model identity and stationary parameters."""
    return json.loads((ROOT / "profiles.json").read_text())[name]


def verify_model(path: Path, profile: dict[str, Any]) -> dict[str, Any]:
    """Reject missing models, LFS pointers and any bytes other than the baseline."""
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"Model is missing: {path}; fetch Git LFS objects first")
    size = path.stat().st_size
    if size != profile["model_bytes"]:
        raise ValueError(f"Model size mismatch: {size}; expected {profile['model_bytes']}")
    digest = sha256(path)
    if digest != profile["model_sha256"]:
        raise ValueError(f"Model SHA-256 mismatch: {digest}")
    return {"model": str(path), "bytes": size, "sha256": digest, "verified": True}


def verify_source(root: Path = ROOT) -> dict[str, Any]:
    """Check that every archived runtime file still matches its historical copy."""
    manifest = json.loads((root / "SOURCE_MANIFEST.json").read_text())
    for record in manifest["files"]:
        path = root / record["path"]
        if not path.is_file() or sha256(path) != record["sha256"]:
            raise ValueError(f"Archived runtime differs from source manifest: {record['path']}")
    return {"verified_runtime_files": len(manifest["files"])}


def preflight(name: str, model: Path, check_dependencies: bool = False) -> dict[str, Any]:
    """Inspect files/dependencies without creating a ROS node or opening hardware."""
    result = {"profile": name, **verify_model(model, load_profile(name)), **verify_source()}
    if check_dependencies:
        modules = ["rclpy", "cv2", "cv_bridge", "numpy", "onnxruntime",
                   "sensor_msgs", "std_msgs", "rcl_interfaces"]
        missing = [name for name in modules if importlib.util.find_spec(name) is None]
        if missing:
            raise ValueError("Missing dependencies in this Python environment: " + ", ".join(missing))
        result["dependencies_discoverable"] = modules
    result["limitations"] = "No camera, middleware, throughput or physical-scene check performed"
    return result


def summarize(path: Path) -> dict[str, Any]:
    """Summarize processed observations; never infer accuracy without ground truth."""
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
            for key in ("wall_time", "inference_ms"):
                if isinstance(row[key], bool) or not math.isfinite(float(row[key])):
                    raise ValueError(f"{key} must be finite")
            if float(row["inference_ms"]) < 0:
                raise ValueError("inference_ms must be nonnegative")
            if rows and float(row["wall_time"]) < float(rows[-1]["wall_time"]):
                raise ValueError("wall_time moved backwards; split sessions or inspect the clock")
        except (ValueError, KeyError, TypeError) as error:
            raise ValueError(f"Invalid observation at line {number}: {error}") from error
        rows.append(row)
    if not rows:
        raise ValueError("Observation log is empty")
    first = float(rows[0]["wall_time"])
    last = float(rows[-1]["wall_time"])
    duration = last - first
    times = [float(row["inference_ms"]) for row in rows]
    accepted = [row for row in rows if row.get("published")]
    return {
        "source": str(path.resolve()),
        "processed_observations": len(rows),
        "first_processed_utc": datetime.fromtimestamp(first, timezone.utc).isoformat(),
        "last_processed_utc": datetime.fromtimestamp(last, timezone.utc).isoformat(),
        "observed_span_seconds": duration,
        "observed_fps": (len(rows) - 1) / duration if duration > 0 else None,
        "inference_ms": {"median": statistics.median(times), "min": min(times),
                         "max": max(times)},
        "accepted_publications": len(accepted),
        "events": [{"seconds_since_first_processed": float(row["wall_time"]) - first,
                    "published": row["published"], "confidence": row.get("confidence"),
                    "frame_sequence": row.get("frame_sequence")} for row in accepted],
        "interpretation": (
            "FPS is (processed rows - 1) / first-to-last processed time. Rows omitted by "
            "stale-result gates are not counted. Event offsets are not time from card "
            "presentation. No ground truth or physical accuracy is inferred."
        ),
    }


def build_bundle(output: Path) -> dict[str, Any]:
    """Copy a model-free portable bundle to a new directory and write its hashes."""
    output = output.resolve()
    if output.exists() or output == ROOT or ROOT in output.parents:
        raise ValueError("Bundle destination must be a new directory outside this source bundle")
    verify_source()
    shutil.copytree(ROOT, output, ignore=shutil.ignore_patterns("__pycache__", "*.pyc",
                                                              "*.lock", "SHA256SUMS"))
    files = sorted(path for path in output.rglob("*") if path.is_file())
    (output / "SHA256SUMS").write_text("".join(
        f"{sha256(path)}  {path.relative_to(output)}\n" for path in files))
    return {"bundle": str(output), "files": len(files), "models_included": False,
            "next_step": "Copy a separately verified ONNX file; start manually via Pixi"}


def ros_arguments(profile: dict[str, Any], model: Path, output: Path) -> list[str]:
    """Build exact recorded ROS parameters without shell string interpolation."""
    arguments = ["--ros-args", "-r", f"__node:={profile['node_name']}"]
    parameters = {**profile["parameters"], "model_path": str(model.resolve()),
                  "output_dir": str(output.resolve())}
    for key, value in parameters.items():
        serialized = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.extend(["-p", f"{key}:={serialized}"])
    return arguments


def start(name: str, model: Path, output: Path) -> None:
    """Manually start only the historical camera subscriber/detector, in foreground."""
    import fcntl

    preflight(name, model, check_dependencies=True)
    profile = load_profile(name)
    # One candidate from this bundle at a time, even with different output directories.
    # This is not a global camera/robot process manager.
    with (ROOT / "candidate.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("A candidate from this bundle is already running") from error
        output = output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        (output / "launch-record.json").write_text(json.dumps({
            "profile": name, "model": str(model.resolve()), "profile_data": profile,
            "runtime_source": str(ROOT / "runtime" / name),
            "started_utc": datetime.now(timezone.utc).isoformat(),
        }, indent=2) + "\n")
        sys.path.insert(0, str(ROOT / "runtime" / name))
        # Import only after explicit 'start', successful validation and the process lock.
        from mdp_perception.live_perception_node import main as live_main

        sys.argv = [sys.argv[0], *ros_arguments(profile, model, output)]
        live_main()


def main() -> int:
    """CLI entry point; all commands except start are hardware-free."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("verify-model", "preflight", "start"):
        command = commands.add_parser(name)
        command.add_argument("--profile", choices=["nano", "medium"], required=True)
        command.add_argument("--model", type=Path, required=True)
        if name == "preflight":
            command.add_argument("--check-dependencies", action="store_true")
        if name == "start":
            command.add_argument("--output", type=Path, required=True,
                                 help="New session directory; existing paths are rejected")
    commands.add_parser("verify-source")
    command = commands.add_parser("build-bundle")
    command.add_argument("--output", type=Path, required=True)
    command = commands.add_parser("summarize")
    command.add_argument("observations", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "verify-model":
            result = verify_model(args.model, load_profile(args.profile))
        elif args.command == "verify-source":
            result = verify_source()
        elif args.command == "preflight":
            result = preflight(args.profile, args.model, args.check_dependencies)
        elif args.command == "build-bundle":
            result = build_bundle(args.output)
        elif args.command == "summarize":
            result = summarize(args.observations)
        else:
            start(args.profile, args.model, args.output)
            return 0
    except (ValueError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
