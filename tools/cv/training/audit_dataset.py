#!/usr/bin/env python3
"""Audit a local Roboflow YOLO snapshot; optionally create a separate clean copy.

Dependencies: Pillow and PyYAML. No network access or source writes occur.
Exact deduplication does NOT prove separation of augmentations or video sessions.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import yaml
from PIL import Image

EXPECTED_NAMES = [str(value) for value in range(11, 41)] + ["marker"]
SPLITS = ("train", "valid", "test")
PRIORITY = {"test": 0, "valid": 1, "train": 2}
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
LIMITATIONS = [
    (
        "Exact-file and decoded-pixel hashes cannot identify general augmentations, "
        "near duplicates, or adjacent video frames."
    ),
    (
        "Without complete, independently verified original/session groups, "
        "split independence remains unverified."
    ),
    (
        "A groups CSV enforces only the provenance supplied by its author; "
        "this tool cannot verify those group assignments."
    ),
    (
        "Empty labels are accepted as background negatives; inspect their images "
        "to confirm there are no unlabeled targets."
    ),
    (
        "Polygon checks cover syntax, finite normalized coordinates, nonzero bounds and area; "
        "they do not prove annotation semantics or absence of self-intersection."
    ),
    (
        "Decode and hash auditing does not establish that classes, target direction, "
        "or annotations are visually correct."
    ),
    (
        "Freeze and hash the resulting snapshot before training. "
        "Audit a stable local copy, not a source being changed concurrently."
    ),
]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_within(path: Path, ancestor: Path) -> bool:
    return path == ancestor or ancestor in path.parents


def ordered_names(value: list | dict) -> list[str]:
    if isinstance(value, list):
        return [str(item) for item in value]
    if isinstance(value, dict):
        normalized = {}
        for key, item in value.items():
            index = int(key)
            if str(index) != str(key) or index in normalized:
                raise ValueError("names keys must be distinct contiguous integers")
            normalized[index] = str(item)
        if set(normalized) != set(range(len(normalized))):
            raise ValueError("names keys must be contiguous from zero")
        return [normalized[index] for index in range(len(normalized))]
    raise ValueError("names must be a list or mapping")


def annotation(path: Path) -> dict:
    """Return a digest insensitive to row ordering and numeric whitespace."""
    content = path.read_text(encoding="utf-8-sig")
    canonical_rows, rows = [], []
    counts, formats = Counter(), Counter()
    for line_number, line in enumerate(content.splitlines(), start=1):
        tokens = line.split()
        if not tokens:
            continue
        if len(tokens) != 5 and not (len(tokens) >= 7 and len(tokens) % 2 == 1):
            raise ValueError(
                f"line {line_number}: expected 5 box fields or odd number >=7 polygon fields"
            )
        try:
            values = [float(token) for token in tokens]
        except ValueError as exc:
            raise ValueError(f"line {line_number}: nonnumeric field") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"line {line_number}: nonfinite value")
        class_value, coordinates = values[0], values[1:]
        if not class_value.is_integer() or not 0 <= class_value < len(EXPECTED_NAMES):
            raise ValueError(f"line {line_number}: class index must be an integer in 0..30")
        if not all(0 <= coordinate <= 1 for coordinate in coordinates):
            raise ValueError(f"line {line_number}: coordinates must be normalized into [0,1]")
        if len(tokens) == 5:
            kind = "box"
            x, y, width, height = coordinates
            bounds = (x - width / 2, y - height / 2, x + width / 2, y + height / 2)
            if width <= 0 or height <= 0:
                raise ValueError(f"line {line_number}: zero-size bounding box")
            if min(bounds) < -1e-6 or max(bounds) > 1 + 1e-6:
                raise ValueError(
                    f"line {line_number}: bounding box extends outside image; "
                    "review before clipping"
                )
        else:
            kind = "polygon"
            points = list(zip(coordinates[::2], coordinates[1::2]))
            xs, ys = coordinates[::2], coordinates[1::2]
            if max(xs) <= min(xs) or max(ys) <= min(ys):
                raise ValueError(f"line {line_number}: zero-size polygon-derived box")
            area_twice = sum(
                x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
            )
            if abs(area_twice) < 1e-15:
                raise ValueError(f"line {line_number}: zero-area polygon")
        class_id = int(class_value)
        canonical_rows.append([class_id] + [0.0 if value == 0 else value for value in coordinates])
        rows.append({"line": line_number, "class_id": class_id, "format": kind})
        counts[str(class_id)] += 1
        formats[kind] += 1
    if len(formats) > 1:
        raise ValueError(
            "mixed box and polygon rows in one file are unsupported by the pinned training loader"
        )
    # Different polygon vertex ordering may produce a conservative conflict.
    # It requires human review instead of silently treating it as equivalent.
    encoded = json.dumps(sorted(canonical_rows), separators=(",", ":")).encode()
    return {
        "sha256": sha256_file(path),
        "annotation_digest": hashlib.sha256(encoded).hexdigest(),
        "empty": not rows,
        "object_count": len(rows),
        "class_counts": dict(counts),
        "formats": dict(formats),
    }


def normalized_yaml(root: Path) -> dict:
    return {
        "path": str(root),
        "train": "train/images",
        "val": "valid/images",
        "test": "test/images",
        "nc": len(EXPECTED_NAMES),
        "names": dict(enumerate(EXPECTED_NAMES)),
    }


def audit(source: Path) -> dict:
    errors, warnings, images = [], [], []
    source_yaml = source / "data.yaml"
    config_hash = None
    try:
        if source_yaml.is_symlink():
            raise ValueError("source data.yaml is a symbolic link")
        config_hash = sha256_file(source_yaml)
        config = yaml.safe_load(source_yaml.read_text())
        names = ordered_names(config["names"])
        if names != EXPECTED_NAMES:
            errors.append(
                f"Canonical class order mismatch: found {names!r}; no label remapping was performed"
            )
        if "nc" in config and config["nc"] != len(EXPECTED_NAMES):
            errors.append(f"data.yaml nc must equal {len(EXPECTED_NAMES)}")
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        errors.append(f"data.yaml: {exc}")

    for split in SPLITS:
        image_root, label_root = source / split / "images", source / split / "labels"
        for directory in (image_root, label_root):
            if not directory.is_dir():
                errors.append(f"Missing directory: {directory.relative_to(source)}")
        if not image_root.is_dir() or not label_root.is_dir():
            continue
        image_map, label_map = defaultdict(list), defaultdict(list)
        for directory, mapping, kind in (
            (image_root, image_map, "image"),
            (label_root, label_map, "label"),
        ):
            if directory.is_symlink() or (source / split).is_symlink():
                errors.append(
                    f"Symbolic-link directory is unsupported: {directory.relative_to(source)}"
                )
                continue
            for path in sorted(directory.rglob("*")):
                if path.is_symlink():
                    errors.append(f"Symbolic link is unsupported: {path.relative_to(source)}")
                    continue
                if not path.is_file():
                    continue
                allowed = (
                    path.suffix.lower() in IMAGE_SUFFIXES
                    if kind == "image"
                    else path.suffix == ".txt"
                )
                if not allowed:
                    warnings.append(f"Skipped unsupported {kind} file: {path.relative_to(source)}")
                    continue
                relative_stem = path.relative_to(directory).with_suffix("").as_posix()
                mapping[relative_stem].append(path)
        for kind, mapping in (("image", image_map), ("label", label_map)):
            for stem, paths in mapping.items():
                if len(paths) > 1:
                    errors.append(
                        f"{split}: {kind} same-relative-stem collision {stem!r}: "
                        f"{[p.name for p in paths]}"
                    )
        if not image_map:
            errors.append(f"{split}: no supported images found")
        for stem in sorted(set(label_map) - set(image_map)):
            errors.append(f"{split}: orphan label {stem}.txt")
        for stem in sorted(image_map):
            label_paths = label_map.get(stem, [])
            if not label_paths:
                errors.append(
                    f"{split}: missing label {stem}.txt (empty labels are legal but must exist)"
                )
            for image_path in image_map[stem]:
                relative_path = image_path.relative_to(source).as_posix()
                record = {"path": relative_path, "split": split, "relative_stem": stem}
                try:
                    record["size_bytes"] = image_path.stat().st_size
                    record["file_sha256"] = sha256_file(image_path)
                    with Image.open(image_path) as opened:
                        opened.verify()
                    with Image.open(image_path) as opened:
                        opened.load()
                        record["width"], record["height"] = opened.size
                        record["mode"] = opened.mode
                        rgb = opened.convert("RGB")
                        pixel_hash = hashlib.sha256(f"RGB:{rgb.width}x{rgb.height}:".encode())
                        pixel_hash.update(rgb.tobytes())
                        record["decoded_rgb_sha256"] = pixel_hash.hexdigest()
                        if opened.getexif().get(274, 1) != 1:
                            warnings.append(
                                f"EXIF orientation present; review loader geometry: {relative_path}"
                            )
                except Exception as exc:
                    errors.append(f"Image unreadable/corrupt {relative_path}: {exc}")
                if len(label_paths) == 1:
                    label = label_paths[0]
                    record["label_path"] = label.relative_to(source).as_posix()
                    try:
                        record["label"] = annotation(label)
                    except (OSError, UnicodeError, ValueError) as exc:
                        errors.append(f"Invalid label {record['label_path']}: {exc}")
                images.append(record)

    duplicate_sets = []
    for field in ("file_sha256", "decoded_rgb_sha256"):
        index = defaultdict(list)
        for record in images:
            if field in record:
                index[record[field]].append(record)
        groups = []
        for digest, group in sorted(index.items()):
            if len(group) < 2:
                continue
            label_digests = {item.get("label", {}).get("annotation_digest") for item in group}
            conflict = len(label_digests) > 1
            descriptor = {
                "hash": digest,
                "paths": [item["path"] for item in group],
                "splits": sorted({item["split"] for item in group}),
                "cross_split": len({item["split"] for item in group}) > 1,
                "annotation_conflict": conflict,
            }
            groups.append(descriptor)
            if conflict:
                errors.append(
                    f"Conflicting annotations for {field} duplicate group: {descriptor['paths']}"
                )
        duplicate_sets.append((field, groups))

    summary = {}
    total_formats = Counter()
    for split in SPLITS:
        records = [record for record in images if record["split"] == split]
        objects, image_counts, formats = Counter(), Counter(), Counter()
        for record in records:
            label = record.get("label", {})
            objects.update(label.get("class_counts", {}))
            image_counts.update(label.get("class_counts", {}).keys())
            formats.update(label.get("formats", {}))
        total_formats.update(formats)
        summary[split] = {
            "images": len(records),
            "paired_labels": sum("label_path" in record for record in records),
            "valid_labels": sum("label" in record for record in records),
            "empty_labels": sum(record.get("label", {}).get("empty", False) for record in records),
            "objects": sum(objects.values()),
            "objects_per_class": {
                name: objects[str(index)] for index, name in enumerate(EXPECTED_NAMES)
            },
            "images_per_class": {
                name: image_counts[str(index)] for index, name in enumerate(EXPECTED_NAMES)
            },
            "row_formats": dict(formats),
        }
    if len(total_formats) > 1:
        warnings.append(
            "Dataset mixes detection boxes and polygons: "
            "inspect the pinned Ultralytics loader behavior before training."
        )
    return {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": str(source),
        "data_yaml_sha256": config_hash,
        "expected_names": EXPECTED_NAMES,
        "summary": summary,
        "images": images,
        "duplicate_groups": dict(duplicate_sets),
        "errors": errors,
        "warnings": warnings,
        "limitations": LIMITATIONS,
        "provenance_grouping": "not supplied; augmentation/session separation UNVERIFIED",
    }


def load_groups(csv_path: Path, records: list[dict]) -> dict[str, str]:
    mapping = {}
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or not {"image_path", "group_id"}.issubset(reader.fieldnames):
            raise ValueError("groups CSV needs image_path,group_id columns")
        for line, row in enumerate(reader, start=2):
            path, group_id = row["image_path"].strip(), row["group_id"].strip()
            if not path or not group_id or path in mapping:
                raise ValueError(f"groups CSV line {line}: empty value or duplicate image_path")
            mapping[path] = group_id
    expected = {record["path"] for record in records}
    if set(mapping) != expected:
        raise ValueError(
            "groups CSV must cover every image exactly once; "
            f"missing={sorted(expected - set(mapping))[:10]}, "
            f"unknown={sorted(set(mapping) - expected)[:10]}"
        )
    return mapping


def prepare_plan(manifest: dict, groups: dict[str, str] | None) -> list[dict]:
    """Union exact-image identity with any supplied original/session identity."""
    records = manifest["images"]
    parent = {record["path"]: record["path"] for record in records}

    def find(path: str) -> str:
        while parent[path] != path:
            parent[path] = parent[parent[path]]
            path = parent[path]
        return path

    def union(a: str, b: str) -> None:
        parent[find(a)] = find(b)

    first_hash, first_group = {}, {}
    for record in records:
        path, digest = record["path"], record["file_sha256"]
        if digest in first_hash:
            union(path, first_hash[digest])
        first_hash[digest] = path
        if groups:
            group_id = groups[path]
            if group_id in first_group:
                union(path, first_group[group_id])
            first_group[group_id] = path
    components = defaultdict(list)
    for record in records:
        components[find(record["path"])].append(record)
    plan = []
    used_destinations = set()
    for component in components.values():
        target_split = min((record["split"] for record in component), key=PRIORITY.get)
        by_hash = defaultdict(list)
        for record in component:
            by_hash[record["file_sha256"]].append(record)
        for identical in by_hash.values():
            keep = min(identical, key=lambda record: (PRIORITY[record["split"]], record["path"]))
            source_path = Path(keep["path"])
            # Retain original target-split filenames; prefix files moved from other splits.
            suffix_path = Path(*source_path.parts[2:])
            if keep["split"] != target_split:
                suffix_path = Path("from_" + keep["split"]) / suffix_path
            image_destination = Path(target_split) / "images" / suffix_path
            label_destination = (Path(target_split) / "labels" / suffix_path).with_suffix(".txt")
            key = label_destination.as_posix()
            if key in used_destinations:
                # Refuse a rare original-name collision during group reassignment.
                raise ValueError(
                    f"Destination same-stem collision: {key}; "
                    "rename conflicting input files in a separate reviewed copy"
                )
            used_destinations.add(key)
            plan.append(
                {
                    "kept_source": keep["path"],
                    "source_label": keep["label_path"],
                    "destination_image": image_destination.as_posix(),
                    "destination_label": label_destination.as_posix(),
                    "discarded_exact_copies": sorted(
                        record["path"] for record in identical if record is not keep
                    ),
                    "file_sha256": keep["file_sha256"],
                    "label_sha256": keep["label"]["sha256"],
                    "group_id": groups[keep["path"]] if groups else None,
                }
            )
    return sorted(plan, key=lambda item: item["destination_image"])


def write_report(manifest: dict, output: Path, suffix: str = "") -> None:
    (output / f"manifest{suffix}.json").write_text(json.dumps(manifest, indent=2) + "\n")
    lines = [
        "# MDP dataset audit",
        "",
        f"Source: `{manifest['source']}`",
        "",
        f"Result: **{'FAIL' if manifest['errors'] else 'STRUCTURAL CHECKS PASS'}**",
        "",
        manifest["provenance_grouping"],
        "",
        "| Split | Images | Valid labels | Empty labels | Objects |",
        "|---|---:|---:|---:|---:|",
    ]
    for split, count in manifest["summary"].items():
        lines.append(
            f"| {split} | {count['images']} | {count['valid_labels']} | "
            f"{count['empty_labels']} | {count['objects']} |"
        )
    for field, groups in manifest["duplicate_groups"].items():
        lines += [
            "",
            f"## {field} duplicates",
            "",
            f"Groups: {len(groups)}; "
            f"cross-split groups: {sum(group['cross_split'] for group in groups)}; "
            f"extra copies: {sum(len(group['paths']) - 1 for group in groups)}.",
        ]
    for title, key in (
        ("Errors", "errors"),
        ("Warnings", "warnings"),
        ("Limits and manual review", "limitations"),
    ):
        lines += ["", "## " + title, ""]
        lines += ["- " + message for message in manifest[key]] or ["None."]
    lines += [
        "",
        (
            "Per-image hashes, annotation digests, duplicate membership, "
            "and per-class counts are in the JSON manifest."
        ),
    ]
    (output / f"report{suffix}.md").write_text("\n".join(lines) + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "source", type=Path, help="Local snapshot containing data.yaml and train/valid/test"
    )
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="New audit output directory outside source"
    )
    parser.add_argument(
        "--clean-to", type=Path, help="New separate directory; source files are never modified"
    )
    parser.add_argument(
        "--groups", type=Path, help="Complete CSV of image_path,group_id (paths relative to source)"
    )
    args = parser.parse_args(argv)
    source = args.source.resolve()
    output = args.output_dir.resolve()
    clean = args.clean_to.resolve() if args.clean_to else None
    if not source.is_dir():
        parser.error("source must exist and be a directory")
    paths = [source, output] + ([clean] if clean else [])
    for index, path in enumerate(paths):
        for other in paths[index + 1 :]:
            if is_within(path, other) or is_within(other, path):
                parser.error(
                    "source, output-dir, and clean-to must be separate non-nested directories"
                )
    if output.exists() or (clean and clean.exists()):
        parser.error("output-dir and clean-to must not already exist; choose new paths")
    output.mkdir(parents=True)
    manifest = audit(source)
    groups = None
    if args.groups:
        try:
            groups = load_groups(args.groups, manifest["images"])
            manifest["provenance_grouping"] = (
                "Complete user-supplied grouping; group semantics still require independent review"
            )
            manifest["groups_csv_sha256"] = sha256_file(args.groups)
            split_sets = defaultdict(set)
            for record in manifest["images"]:
                record["group_id"] = groups[record["path"]]
                split_sets[groups[record["path"]]].add(record["split"])
            manifest["cross_split_provenance_groups"] = {
                group: sorted(splits) for group, splits in split_sets.items() if len(splits) > 1
            }
        except (OSError, ValueError, KeyError) as exc:
            manifest["errors"].append(f"groups CSV: {exc}")
    (output / "data-normalized.yaml").write_text(
        yaml.safe_dump(normalized_yaml(source), sort_keys=False)
    )
    write_report(manifest, output)
    if manifest["errors"]:
        print(
            f"FAIL: {len(manifest['errors'])} errors. Review {output / 'report.md'}",
            file=sys.stderr,
        )
        return 1
    if clean:
        try:
            plan = prepare_plan(manifest, groups)
            (output / "cleanup-plan.json").write_text(json.dumps(plan, indent=2) + "\n")
            clean.mkdir(parents=True)
            for split in SPLITS:
                for kind in ("images", "labels"):
                    (clean / split / kind).mkdir(parents=True)
            for entry in plan:
                for source_key, destination_key, hash_key in (
                    ("kept_source", "destination_image", "file_sha256"),
                    ("source_label", "destination_label", "label_sha256"),
                ):
                    original, destination = (
                        source / entry[source_key],
                        clean / entry[destination_key],
                    )
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(original, destination)
                    if sha256_file(destination) != entry[hash_key]:
                        raise ValueError(
                            f"Source changed during copy: {entry[source_key]}; "
                            "discard this incomplete output"
                        )
            (clean / "data.yaml").write_text(
                yaml.safe_dump(normalized_yaml(clean), sort_keys=False)
            )
            cleaned = audit(clean)
            cleaned["parent_source_manifest_sha256"] = sha256_file(output / "manifest.json")
            cleaned["provenance_grouping"] = manifest["provenance_grouping"]
            if groups:
                with (output / "clean-groups.csv").open("w", newline="") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(["image_path", "group_id"])
                    writer.writerows(
                        (entry["destination_image"], entry["group_id"]) for entry in plan
                    )
            write_report(cleaned, output, "-clean")
            if cleaned["errors"]:
                raise ValueError(
                    "Cleaned output did not pass its post-copy audit; inspect report-clean.md"
                )
        except (OSError, ValueError, KeyError) as exc:
            (output / "CLEANUP_FAILED.txt").write_text(
                str(exc) + "\nDo not use any partial clean output for training.\n"
            )
            print(f"CLEANUP FAILED: {exc}", file=sys.stderr)
            return 1
    print(
        f"Structural checks passed. Review {output / 'report.md'} "
        "and the manual-review limitations."
    )
    if clean:
        print(f"Clean copy: {clean}. Post-copy manifest: {output / 'manifest-clean.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
