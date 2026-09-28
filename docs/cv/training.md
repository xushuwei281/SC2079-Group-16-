# Dataset, training, export and backend verification

This guide explains the completed nano/medium experiments and the reusable scripts added with
this documentation. The scripts are a maintained, configurable version of the recorded workflow;
adding them does **not** mean a new GPU training run or model promotion has occurred. They do not
change the active ROS detector, submit Jobs automatically, overwrite earlier runs, or replace
cloud output prefixes.

Use the [CV documentation index](README.md) for field testing, deployment, evidence and navigation.
The model binaries are [nano](../../models/cv-baselines/nano/best.onnx) and
[medium](../../models/cv-baselines/medium/best.onnx). Fetch Git LFS content before using those files;
a small text pointer is not an ONNX model.

## 1. Script inventory and responsibilities

All paths below are relative to the repository root. Run commands from that root.

| File | Purpose | Inputs | Outputs / failure behavior |
|---|---|---|---|
| `tools/cv/training/audit_dataset.py` | Inspect all images/labels, classes, hashes and duplicate annotations; optionally make a separate deduplicated copy | Local dataset; optional complete provenance CSV | New report directory with `manifest.json`, `report.md`, normalized YAML; exit 1 on structural/annotation conflict errors; existing or nested destinations rejected |
| `tools/cv/training/prepare_smoke.py` | Quarantine every empty-label image and every member of a conflicting duplicate group | Original snapshot and its audit manifest | New smoke dataset, new audit and `quarantine.json`; source unchanged; incomplete output marked `PREPARATION_FAILED.txt` |
| `tools/cv/training/train_baseline.py` | Verify audited bytes, stage a writable worker copy, train selected nano/medium detector and record provenance | Dataset, audit, output directory, unique run name, source commit and dataset reference | Checkpoints, plots, CSV and environment/completion records; existing run refused; `--dry-run` checks without training/download/output writes |
| `tools/cv/training/export_onnx.py` | Export a checkpoint into a new candidate and check its graph/metadata | Canonical `best.pt` and new output directory | Copied `best.pt`, `best.onnx`, `model_manifest.json`; failures mark `EXPORT_FAILED.txt` |
| `tools/cv/training/compare_backends.py` | Compare real predictions from PyTorch and ONNX; optionally compare an explicitly chosen candidate `TargetDetector` | Matching PT/ONNX, frozen images, optional candidate package source | New JSON parity report; exit 1 if any comparison/expected-target check fails; no latency or mAP claim |
| `tools/cv/training/test_audit_dataset.py` | Synthetic audit/cleanup regressions | Temporary generated fixtures | 10 offline tests; no project dataset changes |
| `tools/cv/training/test_workflow.py` | Provenance, dry-run, overwrite, quarantine and comparison regressions | Temporary generated fixtures | Offline tests; no cloud, camera, ROS process or training |
| `tools/cv/training/requirements-audit.txt` | Lightweight dependency pins | Dedicated local environment | PyYAML and Pillow |
| `tools/cv/training/requirements-training.txt` | Retained training/export package versions | Dedicated GPU worker/development environment | Version pins for Ultralytics, Torch, ONNX and supporting packages |

The audit and comparison tools originate from the project guide's canonical companion scripts.
They are now adapted for repository use, typed interfaces and explicit candidate selection. The
historical medium runner also included hardcoded cloud replacement steps; those are deliberately
absent from the reusable runner. Archiving and promotion are separate, explicit operations.

## 2. Environment and access

Use Python 3.11 or 3.12 in a dedicated development environment. The historical GPU worker used
Python 3.12.12. Its medium runner pinned `ultralytics-opencv-headless==8.4.128`, Torch 2.14.0,
Torchvision 0.29.0, ONNX 1.22.0, ONNX Runtime 1.29.0 and ONNX Slim 0.1.96. PyYAML 6.0.3 and
Pillow 12.3.0 are the locally verified audit dependencies. These are recorded versions, not a
claim that arbitrary newer versions produce identical artifacts. This is not a complete
transitive lock or container digest; retain the generated environment record for each run.

On the Pi, continue using the repository's Pixi environments. Do not install these heavy training
packages into Pi system Python. The Pi is an inference/test target, while GPU training occurs on
a separate worker. For a developer environment outside the Pi:

```bash
python3.12 -m venv /absolute/path/to/cv-training-env
/absolute/path/to/cv-training-env/bin/python -m pip install \
  -r tools/cv/training/requirements-training.txt
```

For audit-only work, use `requirements-audit.txt` instead. Below, `python` means the interpreter
of that dedicated environment. Install the authenticated Hugging Face CLI separately, then use
`hf auth login` and `hf auth whoami`; enter credentials only in its login flow. Never put a token
in a committed script or log. The command shapes below were checked against installed
`hf` version **1.30.0**; inspect your installed `--help` when upgrading.

## 3. Extract the dataset into a stable local snapshot

The source is a Hugging Face **Storage Bucket**, `xxshuwei/mdp-symbols-bucket`. It is not a dataset
repository that can be fetched by assuming `load_dataset("xxshuwei/mdp-symbols-bucket")` works.
Download to a new directory. Do not use `--delete` for this workflow.

```bash
hf buckets sync hf://buckets/xxshuwei/mdp-symbols-bucket \
  /absolute/path/to/data-original-YYYYMMDD --dry-run
hf buckets sync hf://buckets/xxshuwei/mdp-symbols-bucket \
  /absolute/path/to/data-original-YYYYMMDD
```

A bucket is mutable: its name alone does not identify the dataset used by a historical run. Retain
its download date, exact file inventory, audit hashes and any upload/snapshot receipts. Stop
writing to that copy before auditing or training. Expected layout:

```text
data-original-YYYYMMDD/
  data.yaml
  train/images/  train/labels/
  valid/images/  valid/labels/
  test/images/   test/labels/
```

Each image must have exactly one same-relative-stem `.txt` label. Nested directories are supported.
An intentionally empty annotation can represent background, but only visual inspection can verify
that there is no unlabelled target. The classes must be **31 names in this order**:

```text
11, 12, 13, 14, 15, 16, 17, 18, 19,
20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, marker
```

Dataset class indices are zero-based: index 0 means official target 11; index 29 means official
target 40; index 30 means the separate marker. Numeric names represent official IDs, not digit
characters to remap again. F is ID 25, digit 1 is ID 11, and circle/stop is ID 40. Marker is not ID 40.

## 4. Audit, review and prepare

```bash
python tools/cv/training/audit_dataset.py /absolute/path/to/data-original-YYYYMMDD \
  --output-dir /absolute/path/to/audit-original-YYYYMMDD
```

Read `report.md` and `manifest.json`. The audit checks decode failures, pairing collisions, missing
or orphan labels, class IDs, finite/normalized coordinates, nonzero boxes/polygons, canonical name
order and duplicate geometry conflicts. It hashes encoded files and decoded RGB pixels. One image
file cannot mix box and polygon rows; mixed formats between files are reported for loader review.
Polygon syntax validation does not prove polygon semantics or absence of self-intersection.

A structural pass does not prove accurate annotations, independent splits or field accuracy. The
original v61 audit found 8,082 pairs (6,948 train, 790 valid, 344 test), 56 exact duplicate groups,
25 cross-split groups and 121 empty labels. The same 56 groups appeared under both file and decoded
pixel hashing, so 112 conflict entries did not mean 112 independent groups. All 56 groups had
consistent class IDs but conflicting annotation geometry. Seven empty-label test images visibly
contained arrows. No reliable original/session provenance was available.

### Smoke-only quarantine

```bash
python tools/cv/training/prepare_smoke.py \
  --source /absolute/path/to/data-original-YYYYMMDD \
  --manifest /absolute/path/to/audit-original-YYYYMMDD/manifest.json \
  --output-dir /absolute/path/to/data-smoke-YYYYMMDD \
  --report-dir /absolute/path/to/audit-smoke-YYYYMMDD
```

This copies retained pairs, verifies copied bytes and audits the result. It removes **all members**
of conflicting duplicate groups rather than selecting an annotation arbitrarily. It also removes
all unverified empty-label pairs. Applied to the recorded v61 snapshot, that policy excluded 115
conflicting images plus 121 empty labels, leaving **7,846 pairs**: 6,765/757/324 train/valid/test.
The new script implements that policy; no new training result is implied by its inclusion here.
It does not remove every possible same-label duplicate or infer session ancestry. Review its
resulting duplicate report; the training preflight still refuses cross-split duplicates by default.
Use this copy for the short pipeline smoke, not as a claim that full data quality was repaired.

### Optional reviewed cleanup with provenance

For a corrected source without conflicting annotations:

```bash
python tools/cv/training/audit_dataset.py /absolute/path/to/corrected-source \
  --output-dir /absolute/path/to/cleanup-report \
  --clean-to /absolute/path/to/clean-dataset \
  --groups /absolute/path/to/source-groups.csv
```

`--groups` is optional and must cover every source image exactly once when supplied:

```csv
image_path,group_id
train/images/frame001.jpg,recording_session_01
valid/images/frame001_crop.jpg,recording_session_01
test/images/frame900.jpg,recording_session_09
```

Use real provenance; the CSV above illustrates the schema only. Cleanup keeps one exact-file copy,
preferring the most protected existing split (test, then valid, then train), and puts connected
provenance groups into that split. It does not silently pick conflicting annotations. It records
`cleanup-plan.json`, `manifest-clean.json`, `report-clean.md` and, when applicable, `clean-groups.csv`.
Inspect changed split/class balance. Differently encoded but pixel-identical files are reported
but not automatically deduplicated. `CLEANUP_FAILED.txt` means a partial copy is unusable.

## 5. Training preflight and two-epoch smoke

```bash
python tools/cv/training/train_baseline.py \
  --data-root /absolute/path/to/data-smoke-YYYYMMDD \
  --audit-manifest /absolute/path/to/audit-smoke-YYYYMMDD/manifest.json \
  --output-root /absolute/path/to/training-outputs \
  --name smoke-yolov8n-YYYYMMDD --model yolov8n.pt --epochs 2 \
  --source-commit COMMIT_SHA --dataset-ref SNAPSHOT_REFERENCE --dry-run
```

Replace `COMMIT_SHA` with the reviewed repository commit and `SNAPSHOT_REFERENCE` with the retained
snapshot identity. The dry-run checks YAML and every audited image/label SHA, image inventory,
nonempty splits, source/output separation and unused run paths. It does not import Torch, download
a model or create outputs. Remove **only** `--dry-run` to execute in a prepared environment with
the selected CUDA device available; use `--device cpu` only for an intentionally slow local run.

The runner stages a verified writable copy on worker storage so Ultralytics cache creation cannot
modify the frozen source. Budget enough disk for the source, this working copy and saved outputs.
Temporary data are removed after completion; the normalized training YAML preserves that temporary
path for provenance, so regenerate a YAML pointing to your current snapshot for later evaluation.

`--initialization pretrained` is the default and requires the official seed name `yolov8n.pt` or
`yolov8m.pt`; a previously downloaded file with that name can be reused. Only use a trusted official
seed for this mode. It may start with generic pretrained classes, and training replaces its head
for the 31-class dataset. For an explicit checkpoint path, use `--initialization custom`; custom
checkpoints must already have canonical names. Initialization intent is recorded explicitly, so
the same cached official seed is not accidentally reclassified as a custom checkpoint. The historical first nano full attempt failed
because its generic seed was incorrectly subjected to the custom canonical-name guard.

The recorded successful smoke job `6aa022b2900620b5c77e28a4` took **229 running seconds** on
`t4-small` and proved loader/training/output/export integration. Smoke accuracy is not a release gate.

## 6. Full nano and medium settings

Use the same command with a fresh name and `--epochs 180`; choose `--model yolov8n.pt` or
`--model yolov8m.pt`. The common recorded configuration was:

| Setting | Value |
|---|---|
| Task/input | Object detection, 640 pixels |
| Batch/data workers | 16 / 4 |
| Max epochs / early-stop patience | 180 / 30 |
| Seed / deterministic | 16 / true |
| Automatic mixed precision | true |
| Image RAM cache | false |
| Horizontal flip / vertical flip / rotation | 0 / 0 / 0 |
| HSV saturation / value augmentation | 0.3 / 0.2 |
| Periodic checkpoint save | Every 10 epochs |
| Other training defaults | Resolved by pinned Ultralytics; save `args.yaml` |

Direction-changing augmentation is disabled because it can change arrow meaning without changing
the label. These are detection runs using the pinned loader's polygon-to-enclosing-box behavior,
not segmentation-model training.

By default the reusable runner refuses annotation conflicts, cross-split duplicates and unverified
empty labels. Both historical full baselines were explicitly authorised on the original, still
problematic v61 split. To intentionally repeat that documented experiment, use both:

```text
--allow-known-data-issues --dataset-note "Historical raw-v61 comparison; annotation conflicts,
25 cross-split duplicate groups and unverified empty labels remain; no session provenance"
```

Put the note on one quoted shell line when executing. This override permits those known issues;
it does not allow broken image decoding, missing labels, malformed annotations or changed bytes.
The limitations are recorded in the environment/completion records. Never describe its test score
as an independent, repaired holdout result.

| Historical result | Nano | Medium |
|---|---:|---:|
| Job ID | `6aa027d5900620b5c77e29ad` | `6aa3621921047bf1b03747ec` |
| GPU flavor | `t4-small` | `l4x1` |
| Completed / selected best epoch | 129 / 99 | 118 / 88 |
| Recorded running time | 9 h 33 m 42 s | 4 h 40 m 35 s |
| Historical compute estimate | USD 3.82 | USD 3.74 |
| Validation mAP50 | 0.99317 | 0.992326 |
| Validation mAP50–95 | 0.98389 | 0.984181 |
| Medium test mAP50 / mAP50–95 | — | 0.993545 / 0.985934 |

These are recorded estimates, not present-day pricing or the billing invoice. The two GPUs and
input preparation paths differed; medium's shorter job time does not show that it needs less
compute. Medium staged the dataset to local worker storage. Nano logged slow access to mounted
images. A `yolo26n.pt` download during AMP checks was a library self-check, not a model switch.

## 7. Hugging Face Jobs: explicit submission and persistence

The repository does not contain an auto-submit script. Before paying for another run, review its
snapshot, preflight, requested GPU, time limit and **durable output mount**. `hf jobs hardware`
shows the account's available flavors and prices. Jobs continue remotely after submission; keeping
this chat or a Pi SSH session open is unnecessary.

Upload a reviewed input snapshot and its audit to new private bucket prefixes first. `hf buckets
sync` accepts local-to-bucket paths too; inspect `--dry-run` before uploading. Mount source read-only.
Create an output readiness marker in the verified output bucket and require it in the runner.
The marker is a deliberate operational check, not a technical proof that any arbitrary folder is
durable. The bucket mount is what provides persistence.

Template for a **manual** smoke submission, after replacing every `YOUR_...` placeholder and creating
the marker in the mounted output prefix:

```bash
hf jobs uv run --detach --flavor t4-small --timeout 1h --python 3.12 \
  --volume hf://buckets/YOUR_NAMESPACE/YOUR_BUCKET/input-snapshot:/input:ro \
  --volume hf://buckets/YOUR_NAMESPACE/YOUR_BUCKET/input-audit:/audit:ro \
  --volume hf://buckets/YOUR_NAMESPACE/YOUR_BUCKET/new-output-prefix:/outputs:rw \
  tools/cv/training/train_baseline.py \
  --data-root /input --audit-manifest /audit/manifest.json \
  --output-root /outputs --require-output-marker \
  --model yolov8n.pt --epochs 2 --batch 16 --workers 4 \
  --name smoke-yolov8n-YYYYMMDD \
  --source-commit COMMIT_SHA --dataset-ref SNAPSHOT_REFERENCE
```

The runner is self-contained and has a PEP 723 dependency block so `hf jobs uv run` can provision
its training dependencies. The audit manifest must be the one for the mounted snapshot. Place
Jobs options before the script path and training arguments after it. The recorded first smoke
submission failed because option parsing consumed a script argument incorrectly.
For a full repeat, set `--epochs 180`, a unique name and a reviewed timeout (historically 24 hours);
medium historically used `--flavor l4x1`. Apply any raw-data override consciously as described above.

```bash
hf jobs inspect JOB_ID
hf jobs logs JOB_ID --follow
hf buckets sync hf://buckets/YOUR_NAMESPACE/YOUR_BUCKET/new-output-prefix \
  /absolute/path/to/retrieved-run-YYYYMMDD
```

Log silence during weight downloads, AMP checks, image scanning or slow storage is not itself
proof of a crash. Inspect job state and elapsed time rather than launching a duplicate run. A local
directory mounted by the CLI is first synced remotely; it is not a live reverse-sync of outputs.
Retrieve durable outputs explicitly and verify hashes after completion.

## 8. Output inventory and export

Retain all experiment context, even though Pi inference only needs the selected model and runtime:

```text
training-outputs/
  configs/RUN_NAME.yaml
  records/RUN_NAME/
    source-data.yaml
    dataset-manifest.json
    training-script.py
    environment.json
    completed.json
  runs/detect/RUN_NAME/
    args.yaml
    results.csv
    weights/best.pt
    weights/last.pt
    weights/epoch*.pt
    ... curves, confusion matrices and validation plots ...
```

`best.pt` is the selected validation checkpoint. `last.pt` is history from the final completed epoch;
inspect its optimizer state before assuming exact training resume is supported. `completed.json`
records the selected file's SHA and canonical names; its existence does not mean export or Pi tests
passed. A failure may leave useful partial output; never silently reuse the same run name.

```bash
python tools/cv/training/export_onnx.py \
  --checkpoint /absolute/path/to/training-outputs/runs/detect/RUN_NAME/weights/best.pt \
  --output-dir /absolute/path/to/new-candidate
```

The exporter copies the checkpoint first, then exports beside that copy. It preserves the training
artifact and rejects an existing candidate directory. Export settings are static batch 1, 640×640,
FP32, opset 18, simplified graph, no embedded NMS. It checks graph validity, one input and output,
`[1,3,640,640] → [1,35,8400]`, FP32 tensors and canonical metadata before writing its manifest.
The 35 channels are four box values plus 31 class values. The runtime still needs RGB conversion,
letterboxing, normalization, confidence filtering, NMS and official-ID interpretation.

Known completed artifacts:

| Model | ONNX bytes | SHA-256 |
|---|---:|---|
| Nano | 12,288,971 | `0207a7fb6ab117ce0a62fd683185da8d8ca065c92f1a0ee1d155e65f14e2c48a` |
| Medium | 103,694,827 | `37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031` |

A re-export or retraining may produce different bytes. Do not copy an old manifest to a new model.
Use the manifest generated for the actual file, and check the runtime contract separately from its
filename. The old cloud prefix with `yolov8n` in its name was replaced with medium outputs by request;
it no longer identifies nano content. Nano's original output set was preserved under the archive
prefix ending `before-medium-20260911T013018Z`. Canonical medium prefix is
`hf://buckets/xxshuwei/jobs-artifacts/baseline-yolov8m-v61-full-20260911T013018Z`.

## 9. Backend comparison and evaluation

Build a frozen camera-scene set with folders `11` through `40`, `marker` and `background`. Each
target frame in this small check has one intended official symbol. Background/marker folders expect
no official target. Keep original files and a scene manifest rather than selecting only successes.

```bash
python tools/cv/training/compare_backends.py \
  --pt /absolute/path/to/new-candidate/best.pt \
  --onnx /absolute/path/to/new-candidate/best.onnx \
  --images /absolute/path/to/frozen-scenes \
  --report /absolute/path/to/new-parity-report.json --check-folders
```

This checks PyTorch against Ultralytics ONNX at confidence 0.50, NMS IoU 0.45, matching box IoU 0.90
and confidence difference tolerance 0.02, with fixed 640 geometry and class-agnostic NMS. Marker
predictions participate in PT/ONNX parity but are excluded from official-target expectations.
Omit `--check-folders` for backend agreement only; agreement alone does not prove correctness.

To compare a specific project runtime, add:

```text
--detector-source /absolute/path/to/candidate/ros2_ws/src/mdp_perception
```

That directory must contain the reviewed `mdp_perception` Python package. The checker requires the
candidate to open the explicitly requested ONNX file: mock mode, automatic NCNN/PT fallback and
another model path are rejected. The report says whether this optional third backend was tested.
Do not infer compatibility from merely finding a similarly named detector in another branch.

The comparison is greedy box matching for small scenes. It is not mAP evaluation, timing,
publication confirmation or a live robot test. The historical medium export also underwent raw
PT/ONNX numerical checks on six images; maximum absolute output differences were approximately
0.000427–0.000992. Neither check establishes independent field recognition accuracy.

For an explicit validation/test run with the selected checkpoint, use the pinned Ultralytics
interface in the prepared environment and the normalized YAML for your current snapshot. Save the
arguments and results in a new directory; preserve validation and test roles. The full historical
scores above came from the documented original split, whose leakage limits remain applicable.
Do not repeatedly tune thresholds on test data and then report it as an untouched holdout.

## 10. Verification performed for these added scripts

Run the offline regression suite:

```bash
python -m unittest discover -s tools/cv/training -p 'test_*.py' -v
```

The suite covers malformed labels, duplicate conflicts, source-byte preservation, protected split
priority, provenance grouping, mutated snapshots, output reuse, output readiness markers, explicit
known-issue notes, safe manifest paths, smoke quarantine and parity mismatches. Expected rejection
messages printed by negative tests are not suite failures; use the final unittest result.

These new utilities were checked offline during documentation integration. No new GPU experiment,
paid job submission, live camera session or robot launch was executed to prepare this addition.
The historical nano physical camera tests and medium Pi replay results are documented separately;
training mAP and software tests must not be substituted for those field measurements.
