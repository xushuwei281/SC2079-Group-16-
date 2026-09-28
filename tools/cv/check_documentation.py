#!/usr/bin/env python3
"""Check local file links in the added CV documentation without network access."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit


def missing_links(document: Path, repository: Path) -> list[str]:
    """Return missing local Markdown link targets for one document."""
    text = document.read_text(encoding="utf-8")
    text = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
    missing = []
    for target in re.findall(r"\]\(([^\n)]+)\)", text):
        target = target.strip().strip("<>")
        parsed = urlsplit(target)
        if parsed.scheme or parsed.netloc or not parsed.path:
            continue
        path = unquote(parsed.path)
        resolved = repository / path.lstrip("/") if path.startswith("/") else document.parent / path
        if not resolved.exists():
            missing.append(f"{document.relative_to(repository)}: {target}")
    return missing


def main() -> int:
    """Check the portable CV documentation and return nonzero on missing links."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    repository = args.repository.resolve()
    paths = [
        repository / "docs/cv",
        repository / "tools/cv",
        repository / "deployment/cv-baselines",
        repository / "models/cv-baselines",
        repository / "experiments/bullseye-navigation-v0.2",
    ]
    documents = sorted({p for root in paths for p in root.rglob("*.md")})
    errors = [error for document in documents for error in missing_links(document, repository)]
    for error in errors:
        print(error)
    print(f"Checked {len(documents)} Markdown files; {len(errors)} missing local links.")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
