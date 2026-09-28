"""Offline provenance/preflight regressions; never train or connect to the robot."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image
import yaml

from audit_dataset import audit
from compare_backends import differences
from prepare_smoke import main as prepare_smoke
from train_baseline import (
    EXPECTED_NAMES,
    main,
    ordered_names,
    safe_child,
    verify_snapshot,
    validate_initialization,
)


class TrainingWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.data = self.root / "data"
        self.data.mkdir()
        (self.data / "data.yaml").write_text(yaml.safe_dump({"names": EXPECTED_NAMES, "nc": 31}))
        for index, split in enumerate(("train", "valid", "test")):
            (self.data / split / "images").mkdir(parents=True)
            (self.data / split / "labels").mkdir()
            Image.new("RGB", (8, 8), (index * 60, 1, 3)).save(
                self.data / split / "images" / "card.png"
            )
            (self.data / split / "labels" / "card.txt").write_text("0 .5 .5 .2 .2\n")
        self.manifest = self.root / "manifest.json"
        self.refresh_manifest()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def refresh_manifest(self) -> None:
        self.manifest.write_text(json.dumps(audit(self.data)))

    def args(self) -> list[str]:
        return [
            "--data-root",
            str(self.data),
            "--audit-manifest",
            str(self.manifest),
            "--output-root",
            str(self.root / "outputs"),
            "--name",
            "new-run",
            "--source-commit",
            "test-only",
            "--dataset-ref",
            "synthetic",
            "--dry-run",
        ]

    def test_dry_run_verifies_without_outputs_or_training(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(self.args()), 0)
        self.assertFalse((self.root / "outputs").exists())
        self.assertFalse(list(self.data.rglob("*.cache")))

    def test_modified_label_is_rejected(self) -> None:
        (self.data / "train/labels/card.txt").write_text("1 .5 .5 .2 .2\n")
        with self.assertRaisesRegex(ValueError, "changed after audit"):
            main(self.args())

    def test_added_image_is_rejected(self) -> None:
        Image.new("RGB", (8, 8), "red").save(self.data / "train/images/extra.png")
        with self.assertRaisesRegex(ValueError, "inventory changed"):
            main(self.args())

    def test_reused_run_name_is_rejected(self) -> None:
        (self.root / "outputs/records/new-run").mkdir(parents=True)
        with self.assertRaises(FileExistsError):
            main(self.args())

    def test_output_marker_required(self) -> None:
        with self.assertRaisesRegex(ValueError, "marker is missing"):
            main(self.args() + ["--require-output-marker"])

    def test_known_issues_need_explicit_note(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main(self.args() + ["--allow-known-data-issues"])

    def test_manifest_traversal_rejected(self) -> None:
        with self.assertRaises(ValueError):
            safe_child(self.data, "../outside")

    def test_empty_label_rejected_by_default_and_quarantined(self) -> None:
        Image.new("RGB", (8, 8), "green").save(self.data / "train/images/empty.png")
        (self.data / "train/labels/empty.txt").write_text("")
        self.refresh_manifest()
        with self.assertRaisesRegex(ValueError, "empty labels remain"):
            main(self.args())
        output, report = self.root / "smoke", self.root / "smoke-report"
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(
                prepare_smoke(
                    [
                        "--source",
                        str(self.data),
                        "--manifest",
                        str(self.manifest),
                        "--output-dir",
                        str(output),
                        "--report-dir",
                        str(report),
                    ]
                ),
                0,
            )
        self.assertTrue((self.data / "train/images/empty.png").is_file())
        self.assertFalse((output / "train/images/empty.png").exists())
        verify_snapshot(output, json.loads((report / "manifest.json").read_text()), False)

    def test_pretrained_and_custom_initialization_are_explicit(self) -> None:
        self.assertTrue(validate_initialization("yolov8n.pt", "pretrained"))
        self.assertTrue(validate_initialization("yolov8m.pt", "pretrained"))
        self.assertFalse(validate_initialization("yolov8n.pt", "custom"))
        self.assertFalse(validate_initialization("/tmp/candidate.pt", "custom"))
        with self.assertRaises(ValueError):
            validate_initialization("/tmp/candidate.pt", "pretrained")

    def test_class_count_and_order_guard(self) -> None:
        self.assertEqual(ordered_names(dict(enumerate(EXPECTED_NAMES))), EXPECTED_NAMES)
        with self.assertRaises(ValueError):
            ordered_names({1: "11", 2: "12"})

    def test_parity_flags_class_and_geometry_disagreement(self) -> None:
        reference = [{"name": "11", "confidence": 0.9, "box": [0, 0, 20, 20]}]
        self.assertEqual(differences(reference, reference, 0.9, 0.02), [])
        wrong = [{"name": "25", "confidence": 0.9, "box": [0, 0, 20, 20]}]
        self.assertIn("class_disagreement", differences(reference, wrong, 0.9, 0.02)[0])
        distant = [{"name": "11", "confidence": 0.9, "box": [40, 40, 60, 60]}]
        self.assertIn("no_box_match", differences(reference, distant, 0.9, 0.02)[0])


if __name__ == "__main__":
    unittest.main()
