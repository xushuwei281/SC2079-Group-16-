# Selected historical CV evidence

This directory contains a compact, repository-portable selection of the original
September 2026 training, camera-test and replay records. The selection supports
[history](../history.md), [results](../results.md) and
[bullseye status](../bullseye-status.md). It excludes model binaries, datasets,
snapshot archives, credentials, SSH configuration and authentication files.

## Provenance and redaction

[manifest.json](manifest.json) records each original source's logical root,
relative filename, original bytes/SHA-256, published bytes/SHA-256 and any
transformations. `cv-workspace` denotes the original CV working area;
`guide-workspace` denotes its accompanying guides/experiment area. The original
report's checksum is included without publishing workstation-specific paths.

Personal workstation roots and the Pi home root are replaced by `${CV_WORKSPACE}`,
`${GUIDE_WORKSPACE}` and `${PI_HOME}` in selected text records. Private Tailscale
addresses and hostnames, if present, are redacted. Placeholders are documentary
labels, not guaranteed environment variables or paths to execute. Metrics, event
times, model hashes and decisions are preserved. Photos are copied without edits.
An original hash and a redacted-copy hash can legitimately differ; use
`published_sha256` to verify the file in this repository.

The dated folders group the stage of work. They do not assert that every timestamp
inside a combined completion record occurred on that folder's date.

| Directory | Contents and evidentiary scope |
|---|---|
| [2026-09-08](2026-09-08) | Original/smoke audit, annotation conflicts, smoke checks and synthetic localhost ROS contract |
| [2026-09-09](2026-09-09) | Nano training/export record, physical stationary failures/successes, physical automatic F log/frame and separate saved-image replay result |
| [2026-09-11](2026-09-11) | Medium training plan, cloud export manifest and output replacement record |
| [2026-09-14](2026-09-14) | Combined medium completion, local contract check, Pi benchmark and final ROS saved-image replay |
| [2026-09-18](2026-09-18) | Historical standalone bullseye package readiness, explicitly not a verified Pi installation |

## Reading historical status fields

- Nano's export manifest says transfer/testing was pending because it was written
  before the later physical test records.
- Medium's cloud manifest precedes its Pi transfer and replay. The later combined
  completion confirms those steps while retaining fresh physical testing as pending.
- A validation summary saying a node was running is a historical observation, not
  a current process check.
- Replay input files were earlier camera photographs. Replaying them on a Pi uses
  real CPU inference, but it creates no new physical camera observations.
- A test event naming a diagnostic frame does not guarantee that frame is included
  in this compact selection. The manifest is the inventory. The two included
  photos are the successful digit 1 service-adjacent frame and the exact physical
  automatic F acceptance frame.

## Verify the published evidence files

From the repository root, the following read-only check verifies the selected
files against the manifest. It does not run inference or interact with hardware.

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib
import json

root = Path("docs/cv/evidence")
manifest = json.loads((root / "manifest.json").read_text())
for item in manifest["entries"]:
    data = (root / item["path"]).read_bytes()
    assert len(data) == item["published_bytes"], item["path"]
    assert hashlib.sha256(data).hexdigest() == item["published_sha256"], item["path"]
print(f"Verified {len(manifest['entries'])} published evidence files")
PY
```

No overall release-readiness claim is encoded by this checksum check. It only
establishes that the selected files match the documentary snapshot.
