"""Synthetic regression checks; no repository data or network is used."""

import csv
import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import yaml
from PIL import Image

from audit_dataset import EXPECTED_NAMES, annotation, audit, main


class DatasetAuditTests(unittest.TestCase):
    def test_mixed_rows_in_one_file_rejected(self) -> None:
        path = self.source / "mixed.txt"
        path.write_text("0 .5 .5 .4 .4\n1 .1 .1 .9 .1 .9 .9 .1 .9\n")
        with self.assertRaisesRegex(ValueError, "mixed box and polygon"):
            annotation(path)

    def test_empty_split_is_not_a_usable_dataset(self) -> None:
        self.image("train", "a.png", "red")
        self.image("valid", "b.png", "green")
        manifest = audit(self.source)
        self.assertTrue(any("test: no supported images" in error for error in manifest["errors"]))

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.source.mkdir()
        (self.source / "data.yaml").write_text(yaml.safe_dump({"names": EXPECTED_NAMES, "nc": 31}))
        for split in ("train", "valid", "test"):
            for kind in ("images", "labels"):
                (self.source / split / kind).mkdir(parents=True)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def image(
        self,
        split: str,
        name: str,
        color: str | tuple[int, int, int],
        label: str = "0 .5 .5 .4 .4\n",
    ) -> Path:
        image_path = self.source / split / "images" / name
        image_path.parent.mkdir(parents=True, exist_ok=True)
        Image.new("RGB", (24, 24), color).save(image_path)
        label_path = self.source / split / "labels" / Path(name).with_suffix(".txt")
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text(label)
        return image_path

    def seed(self) -> None:
        self.image("train", "unique-train.png", (31, 12, 10))
        self.image("valid", "unique-valid.png", (11, 67, 5))
        self.image("test", "unique-test.png", (1, 2, 93))

    def duplicate(
        self, image: Path, target_split: str, name: str, label: str | None = None
    ) -> None:
        target = self.source / target_split / "images" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(image, target)
        source_label = image.parent.parent / "labels" / image.with_suffix(".txt").name
        target_label = self.source / target_split / "labels" / Path(name).with_suffix(".txt")
        target_label.parent.mkdir(parents=True, exist_ok=True)
        target_label.write_text(source_label.read_text() if label is None else label)

    def test_polygon_box_background_and_all_class_indices(self) -> None:
        rows = "\n".join(f"{index} .1 .1 .9 .1 .9 .9 .1 .9" for index in range(31))
        self.image("train", "polygons.png", "red", rows)
        self.image("valid", "box.png", "green")
        self.image("test", "background.png", "blue", "\n")
        manifest = audit(self.source)
        self.assertEqual([], manifest["errors"])
        self.assertEqual(31, manifest["summary"]["train"]["objects"])
        self.assertEqual(1, manifest["summary"]["train"]["objects_per_class"]["marker"])
        self.assertEqual(1, manifest["summary"]["test"]["empty_labels"])
        self.assertEqual(64, len(manifest["images"][0]["decoded_rgb_sha256"]))

    def test_exact_duplicate_priority_and_source_unchanged(self) -> None:
        self.seed()
        a = self.image("train", "shared.png", "red")
        self.duplicate(a, "train", "copy.png")
        self.duplicate(a, "valid", "shared-valid.png")
        self.duplicate(a, "test", "shared-test.png")
        before = {
            str(p.relative_to(self.source)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in self.source.rglob("*")
            if p.is_file()
        }
        output, clean = self.root / "audit", self.root / "clean"
        self.assertEqual(
            0, main([str(self.source), "--output-dir", str(output), "--clean-to", str(clean)])
        )
        self.assertTrue((clean / "test/images/shared-test.png").exists())
        self.assertFalse((clean / "train/images/shared.png").exists())
        self.assertFalse((clean / "valid/images/shared-valid.png").exists())
        manifest = json.loads((output / "manifest-clean.json").read_text())
        self.assertEqual([], manifest["duplicate_groups"]["file_sha256"])
        self.assertEqual(4, sum(item["images"] for item in manifest["summary"].values()))
        after = {
            str(p.relative_to(self.source)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in self.source.rglob("*")
            if p.is_file()
        }
        self.assertEqual(before, after)

    def test_conflicting_annotations_abort_before_clean_creation(self) -> None:
        self.seed()
        a = self.image("train", "shared.png", "red")
        self.duplicate(a, "test", "shared.png", "1 .5 .5 .4 .4\n")
        output, clean = self.root / "audit", self.root / "clean"
        self.assertEqual(
            1, main([str(self.source), "--output-dir", str(output), "--clean-to", str(clean)])
        )
        self.assertFalse(clean.exists())
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertTrue(any("Conflicting annotations" in error for error in manifest["errors"]))

    def test_recursive_pairing_collision_missing_orphan_and_corrupt(self) -> None:
        self.seed()
        self.image("train", "a/same.png", "red")
        self.image("train", "b/same.png", "blue")
        manifest = audit(self.source)
        self.assertEqual([], manifest["errors"])
        self.image("train", "a/same.jpg", "red")
        (self.source / "valid/labels/unique-valid.txt").unlink()
        (self.source / "test/labels/orphan.txt").write_text("")
        (self.source / "test/images/unique-test.png").write_bytes(b"not an image")
        errors = "\n".join(audit(self.source)["errors"])
        for expected in (
            "same-relative-stem collision",
            "missing label",
            "orphan label",
            "unreadable/corrupt",
        ):
            self.assertIn(expected, errors)

    def test_bad_labels_are_rejected(self) -> None:
        label_path = self.root / "bad.txt"
        cases = [
            "31 .5 .5 .4 .4",
            "0.5 .5 .5 .4 .4",
            "0 nan .5 .4 .4",
            "0 .5 .5 0 .4",
            "0 .1 .1 .8 .8",
            "0 .1 .1 .9 .9 .5",
            "0 .1 .1 .5 .5 .9 .9",
            "0 -0.1 .1 .9 .1 .9 .9",
        ]
        for row in cases:
            with self.subTest(row=row):
                label_path.write_text(row)
                with self.assertRaises(ValueError):
                    annotation(label_path)

    def test_annotation_digest_ignores_order_and_numeric_format(self) -> None:
        a, b = self.root / "a.txt", self.root / "b.txt"
        a.write_text("0 .5 .5 .4 .4\n1 .4 .4 .2 .2\n")
        b.write_text("1 0.40 0.40 0.20 0.20\n0 0.50 0.50 0.40 0.40\n")
        self.assertEqual(annotation(a)["annotation_digest"], annotation(b)["annotation_digest"])
        self.assertNotEqual(annotation(a)["sha256"], annotation(b)["sha256"])

    def test_provenance_groups_move_related_images_to_held_out_split(self) -> None:
        self.seed()
        self.image("train", "session-original.png", "red")
        self.image("valid", "session-augmented.png", "pink")
        self.image("test", "session-frame.png", "orange")
        groups = self.root / "groups.csv"
        with groups.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["image_path", "group_id"])
            for record in audit(self.source)["images"]:
                writer.writerow(
                    [
                        record["path"],
                        "same-session" if "session-" in record["path"] else record["path"],
                    ]
                )
        output, clean = self.root / "audit", self.root / "clean"
        self.assertEqual(
            0,
            main(
                [
                    str(self.source),
                    "--output-dir",
                    str(output),
                    "--clean-to",
                    str(clean),
                    "--groups",
                    str(groups),
                ]
            ),
        )
        self.assertTrue((clean / "test/images/from_train/session-original.png").exists())
        self.assertTrue((clean / "test/images/from_valid/session-augmented.png").exists())
        self.assertTrue((clean / "test/images/session-frame.png").exists())
        self.assertFalse((clean / "train/images/session-original.png").exists())
        self.assertIn("independent review", (output / "report-clean.md").read_text())

    def test_wrong_class_order_and_incomplete_groups_fail(self) -> None:
        self.seed()
        (self.source / "data.yaml").write_text(yaml.safe_dump({"names": EXPECTED_NAMES[::-1]}))
        self.assertTrue(
            any("class order mismatch" in error for error in audit(self.source)["errors"])
        )
        (self.source / "data.yaml").write_text(yaml.safe_dump({"names": EXPECTED_NAMES}))
        groups = self.root / "groups.csv"
        groups.write_text("image_path,group_id\ntrain/images/unique-train.png,session-a\n")
        self.assertEqual(
            1,
            main(
                [
                    str(self.source),
                    "--output-dir",
                    str(self.root / "audit"),
                    "--groups",
                    str(groups),
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
