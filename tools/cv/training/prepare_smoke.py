#!/usr/bin/env python3
"""Create a NEW smoke-only copy excluding all conflicting duplicates and empty labels."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import yaml

from audit_dataset import audit, is_within, normalized_yaml, sha256_file, write_report
from train_baseline import safe_child, verify_snapshot


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    source, output, report = [
        p.resolve()
        for p in (
            args.source,
            args.output_dir,
            args.report_dir,
        )
    ]
    paths = [source, output, report]
    for index, path in enumerate(paths):
        for other in paths[index + 1 :]:
            if is_within(path, other) or is_within(other, path):
                parser.error("Source, output and report must be separate non-nested directories")
    if output.exists() or report.exists():
        parser.error("Output and report directories must be new")
    manifest = json.loads(args.manifest.read_text())
    verify_snapshot(source, manifest, allow_issues=True)
    excluded = {record["path"] for record in manifest["images"] if record["label"]["empty"]}
    # Quarantine the complete duplicate group; never choose between conflicting annotations.
    records = {record["path"]: record for record in manifest["images"]}
    for groups in manifest["duplicate_groups"].values():
        for group in groups:
            annotations = {records[path]["label"]["annotation_digest"] for path in group["paths"]}
            if len(annotations) > 1:
                excluded.update(group["paths"])
    report.mkdir(parents=True)
    output.mkdir(parents=True)
    try:
        for split in ("train", "valid", "test"):
            for kind in ("images", "labels"):
                (output / split / kind).mkdir(parents=True)
        for record in manifest["images"]:
            if record["path"] in excluded:
                continue
            for relative, expected_hash in (
                (record["path"], record["file_sha256"]),
                (record["label_path"], record["label"]["sha256"]),
            ):
                destination = safe_child(output, relative)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(safe_child(source, relative), destination)
                if sha256_file(destination) != expected_hash:
                    raise ValueError(f"Source changed during copy: {relative}")
        (output / "data.yaml").write_text(yaml.safe_dump(normalized_yaml(output), sort_keys=False))
        result = audit(output)
        write_report(result, report)
        (report / "quarantine.json").write_text(
            json.dumps(
                {
                    "source_manifest_sha256": sha256_file(args.manifest),
                    "excluded_images": sorted(excluded),
                    "excluded_count": len(excluded),
                    "purpose": (
                        "Loader/pipeline smoke only; " "not a repaired independent evaluation set"
                    ),
                },
                indent=2,
            )
            + "\n"
        )
        if result["errors"]:
            raise ValueError("Smoke copy failed structural checks; inspect report.md")
    except Exception:
        (report / "PREPARATION_FAILED.txt").write_text("Incomplete smoke copy; do not train.\n")
        raise
    print(f"Smoke copy: {output}; audit: {report}; excluded {len(excluded)} images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
