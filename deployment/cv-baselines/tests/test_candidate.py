"""Offline regression checks for reproducibility tools, without ROS or hardware."""

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SPEC = importlib.util.spec_from_file_location("candidate", Path(__file__).parents[1] / "candidate.py")
candidate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(candidate)


class TestCandidateTools(unittest.TestCase):
    def test_model_identity_rejects_same_size_wrong_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / "best.onnx"
            model.write_bytes(b"good")
            profile = {"model_bytes": 4, "model_sha256": hashlib.sha256(b"good").hexdigest()}
            self.assertTrue(candidate.verify_model(model, profile)["verified"])
            model.write_bytes(b"evil")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                candidate.verify_model(model, profile)

    def test_pointer_and_missing_model_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / "best.onnx"
            profile = {"model_bytes": 103694827, "model_sha256": "unused"}
            with self.assertRaisesRegex(ValueError, "missing"):
                candidate.verify_model(model, profile)
            model.write_text("version https://git-lfs.github.com/spec/v1\n")
            with self.assertRaisesRegex(ValueError, "size mismatch"):
                candidate.verify_model(model, profile)

    def test_summary_distinguishes_processing_span_and_event_offset(self) -> None:
        rows = [
            {"wall_time": 100, "inference_ms": 900, "published": None},
            {"wall_time": 101, "inference_ms": 800, "published": None},
            {"wall_time": 103, "inference_ms": 1000, "published": "1,25"},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in rows))
            result = candidate.summarize(path)
        self.assertEqual(result["processed_observations"], 3)
        self.assertAlmostEqual(result["observed_fps"], 2 / 3)
        self.assertEqual(result["inference_ms"]["median"], 900)
        self.assertEqual(result["accepted_publications"], 1)
        self.assertEqual(result["events"][0]["seconds_since_first_processed"], 3)

    def test_summary_rejects_bad_clock_and_nonfinite_timing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            path.write_text('{"wall_time":100,"inference_ms":900}\n'
                            '{"wall_time":99,"inference_ms":900}\n')
            with self.assertRaisesRegex(ValueError, "line 2"):
                candidate.summarize(path)
            path.write_text('{"wall_time":100,"inference_ms":NaN}\n')
            with self.assertRaisesRegex(ValueError, "finite"):
                candidate.summarize(path)
            path.write_text("")
            with self.assertRaisesRegex(ValueError, "empty"):
                candidate.summarize(path)

    def test_single_row_has_no_derived_fps(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            path.write_text('{"wall_time":100,"inference_ms":900}\n')
            self.assertIsNone(candidate.summarize(path)["observed_fps"])

    def test_ros_arguments_keep_paths_as_single_values(self) -> None:
        profile = candidate.load_profile("medium")
        arguments = candidate.ros_arguments(profile, Path("/tmp/my model.onnx"), Path("/tmp/log"))
        self.assertIn(f"model_path:={Path('/tmp/my model.onnx').resolve()}", arguments)
        self.assertIn("invert_colors:=true", arguments)
        self.assertIn("max_result_age_seconds:=8.0", arguments)
        self.assertIn("__node:=perception_medium_candidate", arguments)
        self.assertIn("target_topic:=/test/medium/android/target", arguments)

    def test_source_verification_detects_modified_archive(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "runtime.py"
            source.write_text("original")
            (root / "SOURCE_MANIFEST.json").write_text(json.dumps({"files": [
                {"path": "runtime.py", "sha256": candidate.sha256(source)}]}))
            self.assertEqual(candidate.verify_source(root)["verified_runtime_files"], 1)
            source.write_text("changed")
            with self.assertRaisesRegex(ValueError, "differs"):
                candidate.verify_source(root)

    def test_bundle_refuses_overwrite_and_has_verifiable_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "bundle"
            result = candidate.build_bundle(output)
            self.assertFalse(result["models_included"])
            self.assertFalse(list(output.rglob("*.onnx")))
            for line in (output / "SHA256SUMS").read_text().splitlines():
                expected, relative = line.split("  ", 1)
                self.assertEqual(candidate.sha256(output / relative), expected)
            with self.assertRaisesRegex(ValueError, "new directory"):
                candidate.build_bundle(output)


if __name__ == "__main__":
    unittest.main()
