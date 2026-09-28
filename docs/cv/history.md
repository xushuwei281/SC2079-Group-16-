# CV engineering history and workflow

This document records the September 2026 computer-vision work for accountability,
handover and presentation. It describes what was built, why decisions were made,
and what evidence was retained. The accompanying [results](results.md) distinguish
physical camera tests from saved-image replay. [Bullseye status](bullseye-status.md)
separates the historical navigation experiment from the newer integrated code.

The evidence was collected before this documentation change. This change did not
run another training job, connect to the robot, move the robot, or repeat a camera
test. The source report was prepared on 28 September 2026; its exact checksum and
the published evidence checksums are in [the evidence manifest](evidence/manifest.json).
Historical status fields are retained, even where subsequent records supersede them.

## 1. Objective and system responsibilities

The work began as a request to consolidate two instructions sets for extracting,
training and testing CV modules from GitHub and Hugging Face. It developed into
execution of that workflow, deployment to a Raspberry Pi, automatic recognition
of printed cards, comparison of nano and medium models, and a separate obstacle
navigation experiment.

| System | Responsibility in the recorded workflow |
|---|---|
| This GitHub project | Robot application, ROS 2 packages, training and deployment source |
| Hugging Face account `xxshuwei` | Dataset/output storage and remote GPU Jobs |
| Development workstation | Dataset audit, isolated development checkouts, model verification and evidence collection |
| Raspberry Pi | Camera acquisition, ONNX CPU inference and ROS 2 result publication |
| Android and STM32 | Existing user-interface and movement subsystems; not validated end-to-end by the stationary recognition tests |
| SSH over Tailscale | Shell access and file transfer; not a component of neural-network training |

Access setup and authentication preceded execution. Credentials are not included
in this repository evidence set. Once accepted, a Hugging Face Job runs remotely;
keeping a chat or Pi SSH terminal open is not part of its execution path.

## 2. Chronology

Dates are artifact dates; timestamps ending in `Z` are UTC.

| Action and reason | Recorded outcome |
|---|---|
| Audit original source, label mapping and dataset before expensive training | Found class interpretation errors, duplicate leakage, conflicting polygons and unverified empty labels |
| Establish canonical class contract and diagnostic tests | Corrected official IDs; separated filled circle/Stop ID 40 from the marker |
| Quarantine problematic records for a short smoke run | 7,846 smoke image/label pairs; structural audit passed, session independence still unknown |
| Run two-epoch YOLOv8n smoke job | Initial submission failed before training; corrected submission completed in 229 running seconds |
| Train full nano baseline after explicit authorization to use the original split | Initial checkpoint guard failed; retry completed 129 epochs, selecting epoch 99 |
| Verify/export nano and stage a separate Pi candidate | Static FP32 ONNX contract and model identity recorded |
| Diagnose physical digit 1 failures | Original black-on-white frames failed; saved-image inversion identified a polarity mismatch |
| Repeat stationary camera checks with inversion | Digit 1 and F accepted; one background scene rejected |
 Add automatic live recognition | Three processed-frame confirmation, duplicate suppression, exact accepted-frame evidence and isolated output topic |
| Exercise live behavior on Pi | Saved-image replay passed seven checks; physical live F accepted automatically at about 1 FPS |
| Train full YOLOv8m baseline | 118 epochs, selected epoch 88; verified original nano backup before replacing the requested old output prefix |
| Benchmark medium on Pi and fix scheduling | Single inference worker avoids callback blocking/backlog; final saved-image ROS replay passed at about 0.20 FPS |
| Preserve medium runtime and validation artifacts | Configuration/model snapshot recorded; fresh physical medium camera test remained pending |
| Package medium ONNX with Git LFS | Historical model commit `841f090` retained exact model identity |
| Build separate bullseye experiment and one-command launcher | Local image/simulation tests completed; final Pi installation and physical navigation unverified |
| Later support work | Handle changed Pi endpoint, remote terminal and Codex sign-in | Access support did not establish restoration of the prior robot workspace |
| Reconcile report and original records | Separate physical evidence, replay evidence, simulation and proposed work |

The historical report inspected older development checkouts. It does not describe
the latest integrated `main` branch as if no later changes occurred. In particular,
the current repository includes integrated bullseye tracking/control; see
[the version boundary](bullseye-status.md#2-current-integrated-repository-code).

## 3. Extraction, preservation and audit

### Source and snapshot

The source was the Hugging Face **Storage Bucket**
`xxshuwei/mdp-symbols-bucket`, not a `datasets` repository. A local snapshot named
`20260908T142129Z` was retained so audit and training decisions could refer to the
same extracted files.

The [raw audit](evidence/2026-09-08/raw-dataset-audit.md) reports 8,082 image/label
pairs and 16,168 files including metadata:

| Split | Pairs | Empty labels | Recorded objects |
|---|---:|---:|---:|
| Train | 6,948 | 102 | 6,846 |
| Validation | 790 | 12 | 778 |
| Test | 344 | 7 | 337 |
| Total | 8,082 | 121 | 7,961 |

“Valid labels” in the audit means the parser accepted the file structure. It does
not establish that a polygon is correct or that an empty file represents a true
background image.

### Findings and their effect on interpretation

There were **56 exact-duplicate groups containing 115 images**, equivalent to 59
extra copies. **25 groups crossed splits.** All 56 groups retained the same class
IDs but disagreed in polygon geometry; see the
[conflict classification](evidence/2026-09-08/duplicate-conflict-summary.json).
File-byte hashes and decoded-RGB hashes independently rediscovered these groups.
The resulting 112 conflict entries must not be described as 112 separate groups.

There were 121 empty labels. Visual review in the source work found arrows in all
seven empty-label test images, so those were not trustworthy negative examples.
No corrected annotation export or original recording-session/augmentation groups
were available. Exact-hash checks cannot detect every adjacent frame, crop or
augmented derivative, and cannot prove a leak-free holdout.

### Smoke quarantine versus full training

The smoke copy excluded every member of the 56 conflicting groups, rather than
arbitrarily choosing one polygon, and excluded every empty label: 115 + 121 = 236
pairs removed. Its [audit](evidence/2026-09-08/smoke-dataset-audit.md) reports 6,765
train, 757 validation and 324 test pairs, totaling 7,846. It passed structural checks
with no exact duplicates or empty labels. Session independence remained unverified.

The smoke archive was 390,487,469 bytes with SHA-256
`dd6da1a74b1c7f21990bfd80f8c92de108679e8119b7d0e1ad1706162411068f`.

**Both full baselines used the original split after explicit authorization.** The
smoke quarantine was a pipeline check, not a repair silently applied to full
training. The full-run metrics are consequently diagnostic baseline results, not
an independent estimate of real-world accuracy.

## 4. Class-contract correction

The model output index, class-name string and official target ID are different
things. A detector can localize the right image and still send the wrong ID if
its interpretation layer confuses them.

The trained baseline class order is:

```text
["11", "12", ..., "39", "40", "marker"]
```

| Zero-based model index | Meaning | Official output |
|---|---|---|
| 0–8 | Digits 1–9 | 11–19 |
| 9–16 | A–H | 20–27 |
| 17–24 | S–Z | 28–35 |
| 25–28 | Up, down, right, left arrows | 36–39 |
| 29 | Filled circle / Stop | 40 |
| 30 | Bullseye marker | Not an ordinary official target |

The canonical numeric names are interpreted directly. F is 25; digit 1 is 11;
circle 40 remains a valid symbol. Marker suppression is based on marker semantics,
not on treating numeric 40 as a marker.

Recorded development checks included 13/31 to 31/31 correctly interpreted cases
on the same small old-model diagnostic, 38/38 cross-backend fixture agreement,
and 19 class/loader regression passes. These are software/diagnostic checks, not
independent full-dataset accuracy scores. One colcon summary counted zero XML
test cases because that unittest runner did not emit JUnit XML; the unittest log
recorded 19 passes.

The [isolated ROS contract record](evidence/2026-09-08/isolated-ros-contract.json)
contains four synthetic publication cases, using localhost domain 116. It is not
a robot mission test. Marker suppression, IDs 11/39 with obstacle association and
blank handling were exercised.

## 5. Training and export decisions

The intended sequence was preserved data → audit → smoke job → export/contract
checks → authorized full baseline → verified ONNX candidate → Pi tests.

The first smoke submission failed because a script option was consumed by the
submission command. The successful smoke job was `6aa022b2900620b5c77e28a4`; see
[smoke verification](evidence/2026-09-08/smoke-verification.json).

The first full nano attempt failed with:

```text
ValueError: Custom fine-tuning checkpoint must have canonical class names
```

The guard needed to distinguish official generic pretrained initialization from a
custom fine-tuning checkpoint. The former receives a newly configured 31-class
head; the latter must already satisfy the expected class contract. The corrected
retry retained validation for custom checkpoints.

An AMP check downloaded `yolo26n.pt` during initialization. That was an auxiliary
AMP check, not a switch from the requested YOLOv8 architecture. The nano job also
reported slow mounted-image access. A pause in epoch log updates during such
preparation is not enough to establish failure; the completion record establishes
the eventual outcome.

Both full runs requested 180 epochs, patience 30, image size 640, batch 16, four
workers, seed 16, deterministic mode, AMP, automatic optimizer selection and
periodic checkpoints every 10 epochs. Horizontal flip, vertical flip and rotation
were disabled to preserve arrow semantics. HSV settings were 0.015/0.3/0.2.
The task was detection: polygon source labels were handled through the detection
loader's enclosing-box path, not a segmentation model.

The nano environment record identifies Python 3.12.12, PyTorch 2.14.0, CUDA 13.0
and `ultralytics-opencv-headless` 8.4.128. These are recorded historical versions,
not an instruction to silently upgrade dependencies today.

Exports use static FP32 input `[1,3,640,640]`, output `[1,35,8400]`, batch 1 and
opset 18. NMS is not embedded. The runtime must still perform color conversion,
letterboxing, normalization, confidence filtering, NMS and class interpretation.
The [nano](evidence/2026-09-09/nano-export-manifest.json) and
[medium](evidence/2026-09-11/medium-export-manifest.json) manifests retain the
contract. Exact model identities and completed metrics are in [results](results.md).

## 6. Cloud output replacement and retention

Medium retraining was requested to replace the previous output set. The workflow
first preserved the nano files, then verified medium exports before replacement.
The [replacement receipt](evidence/2026-09-11/REPLACEMENT_COMPLETE.json) records
that action; this documentation did not re-query or modify the bucket.

| Historical bucket location under `hf://buckets/xxshuwei/jobs-artifacts/` | Meaning |
|---|---|
| `archives/baseline-yolov8n-v61-full-20260908T142129Z-r2-4506d3d3-before-medium-20260911T013018Z` | Preserved nano output set |
| `baseline-yolov8m-v61-full-20260911T013018Z` | Canonical medium output |
| `baseline-yolov8n-v61-full-20260908T142129Z-r2-4506d3d3` | Legacy nano-named prefix populated with medium outputs after replacement |

The last path's name is misleading after replacement. Use model checksums and
manifests, never folder names alone. The completion record lists 73 cloud files,
61 downloaded files and 12 periodic checkpoints retained on Hugging Face.

Preserve `best.pt` for selected-checkpoint evaluation/fine-tuning, `last.pt` for run
history, `best.onnx` for inference, and arguments, environment, metrics, model
manifest and dataset identity for reproduction. A bare ONNX file does not explain
how the model was trained or what preprocessing was tested. A checkpoint named
`last.pt` does not by itself prove optimizer state is present for exact resume.

## 7. Runtime evolution and preservation

Physical testing first used request-driven `/test/perception/sample_target` calls.
After the polarity diagnosis, inversion was applied once in the inference path.
Automatic recognition then used the latest received image, three fresh processed
observations, confidence gating and duplicate suppression. It saved the exact
accepted image and event metadata, improving traceability over the earlier
service tests' nearby diagnostic frames.

Medium's latency exposed callback blocking and stale-result problems. A single
inference worker was introduced so input/parameter callbacks remained responsive;
timer ticks did not enqueue an unbounded backlog. Obstacle-generation checks
rejected results started under an earlier obstacle assignment. This scheduling
fix improves responsiveness and association, not neural-network compute speed.

Historical source milestones were `bc942ea` (class contract), `50e1283` (canonical
training), `adeb8d9` (nano live candidate), `698e38a` (medium worker/candidate) and
`841f090` (medium ONNX in Git LFS). They identify the historical work; they are not
claims that each old branch should replace the latest integrated application.

Runtime snapshots named `live-20260909T061751Z.tar.gz` and
`medium-20260914T083849Z.tar.gz` were retained outside Git. The medium archive was
89,169,983 bytes with SHA-256
`b4979bc2ebeedd5583ddfed772a4800d0da663d6f9c1542f7e1ce382e0b9e48a`, with 70 verified
files recorded. These are configuration/model snapshots, not complete OS images.
Clean-machine restoration was not established by taking a snapshot.

## 8. How to present this work accurately

The demonstrated process includes dataset auditing, controlled smoke training,
full baseline training, exact export verification, failure diagnosis with retained
examples, stationary physical recognition, measured throughput, temporal-policy
replay, latency-driven scheduling changes and preservation of tested settings.

The demonstrated outcome is narrower than complete autonomous field readiness:
physical nano examples cover digit 1, F and one background scene; medium's Pi
tests replay earlier camera images; the historical navigation experiment was
tested locally. Do not convert these into claims of all-symbol field accuracy,
recognition while moving, Android delivery, or completed physical obstacle orbit.
Use [results](results.md) and [bullseye status](bullseye-status.md) for the exact
boundaries and evidence links.
