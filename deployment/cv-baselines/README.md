# Historical CV baseline reproduction bundle

This bundle preserves the **9 September nano** and **14 September medium** stationary
recognition experiments. It is independent of the newer `ros2_ws/src/mdp_perception`
implementation. Importing a historical module here is not a recommendation to replace
the active robot package. No current launch file, planner, firmware or model default
is changed by this bundle.

Read the [Pi testing manual](../../docs/cv/pi-testing.md) for the physical test sequence,
recorded outcomes and evidence limits. The two model files are held separately under
[`models/cv-baselines`](../../models/cv-baselines/README.md) through Git LFS.

## What is supplied

| File or directory | Purpose and inputs | Outputs/dependencies |
|---|---|---|
| `profiles.json` | Fixed nano/medium identities, ROS node names and measured stationary timing settings | Model byte size, SHA-256 and ROS parameters; no machine-specific paths |
| `SOURCE_MANIFEST.json` | Original local bundle names and hashes for 17 archived Python files | Provenance of unchanged runtime copies |
| `candidate.py verify-model` | Profile and explicit ONNX path | JSON confirming exact bytes; standard library only |
| `candidate.py verify-source` | Bundled runtime files | JSON count or failure; standard library only |
| `candidate.py preflight` | Profile, model; optional dependency discovery | Model/source checks and optional module availability; opens no camera or ROS node |
| `candidate.py build-bundle` | New destination outside this source directory | Portable source copy plus `SHA256SUMS`; no weights, OS or Python environment |
| `candidate.py summarize` | One session's `observations.jsonl` | JSON counts, latency, throughput and acceptance event offsets; no accuracy inference |
| `candidate.py start` | Profile, verified model and **new** output directory | Foreground historical detector, accepted events, images and JSON records; ROS dependencies required |
| `start_candidate.sh` | Absolute ROS workspace, profile, ONNX path and new session directory | Invokes `candidate.py start` with the workspace's frozen Pixi `pi` environment |
| `runtime/nano/` | Exact 9 September source and offline tests | Historical nano behaviour, including inference in its original callback path |
| `runtime/medium/` | Exact final 14 September source and tests | Single worker prevents slow inference from blocking callback processing and accumulating work |
| `tests/test_candidate.py` | Synthetic files/logs | Eight hardware-free regression tests for the new packaging/verification tools |

The wrapper does **not** start a camera, router, Bluetooth connection, motor bridge,
planner or boot service. It subscribes to an already-running camera. It locks this
bundle against simultaneous nano/medium starts, but cannot detect every other detector
or prevent another independent bundle from running. Stop competing tests manually.

## Archived Python modules, individually

Both profiles have a separate `mdp_perception` package so later main-branch changes do
not silently change the experiment being reproduced:

| Module | Responsibility |
|---|---|
| `__init__.py` | Marks the historical Python package; not an executable command |
| `class_contract.py` | Parse model class metadata; validate static FP32 detection input/output; map canonical strings `11` through `40` to official IDs; exclude the separate `marker` class |
| `detector.py` | Select the explicit model/backend, enforce compatibility, prepare images, run ONNX, apply confidence/NMS, return detections and draw annotations |
| `live_confirmation.py` | Pure-Python temporal gate: fresh processed observations, three-frame agreement, duplicate suppression, absence reset and obstacle reset |
| `live_perception_node.py` | ROS image subscriber, freshness checks, inference scheduling, event publication, compressed annotated images and evidence recording; medium includes its historical worker fix |
| `test/test_class_contract.py` | Fourteen tests of canonical classes, invalid metadata and static ONNX contract |
| `test/test_detector_contract.py` | Five tests of explicit model/backend failure and marker/circle separation using lightweight fakes |
| `test/test_live_confirmation.py` | Ten tests of confirmation, time gaps, repeated frames, duplicate suppression, absence and obstacle changes |
| `runtime/medium/test/test_live_worker.py` | Blocked-inference regression: timer ticks must not queue work and obstacle changes invalidate an in-flight result; requires ROS and skips when unavailable |

These copies intentionally preserve historical formatting and implementation. Changes
to them invalidate `verify-source`. New robot integration should be reviewed against
the current active perception package rather than editing these archived copies.

## Dependencies and environment

The packaging tools require Python 3.10+ and the standard library. The historical
Pi record used Python 3.12.14/aarch64, ONNX Runtime 1.29.0 and OpenCV 4.13.0. Actual
inference additionally requires `numpy`, `cv2` and `onnxruntime`; ROS live operation
requires `rclpy`, `cv_bridge`, `sensor_msgs`, `std_msgs` and `rcl_interfaces` plus the
workspace's middleware configuration. Use Pixi on the Pi. Do not install packages into
system Python. A source bundle is not a complete environment backup.

Run from the repository root; substitute your actual Pi checkout if it differs:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
export PI_REPO=/home/mdp/dev/SC2079-Group-16
export PI_WS="$PI_REPO/ros2_ws"
export CV_BUNDLE="$PI_REPO/deployment/cv-baselines"
cd "$PI_REPO"
git lfs pull --include='models/cv-baselines/**'

pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" preflight --profile nano \
  --model "$PI_REPO/models/cv-baselines/nano/best.onnx" --check-dependencies
```

`--check-dependencies` checks module discovery, not successful inference, camera
operation or transport connectivity. The historical detector validates ONNX metadata
when it loads the model. Byte verification alone is intentionally sufficient to
identify these already-exported, fixed artifacts; it is not a general model validator.

## Offline commands

All commands in this section are hardware-free and create no ROS node. On a workstation
with Python installed, the same `candidate.py` subcommands can be run with that Python
interpreter instead of the Pi-specific Pixi prefix.

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" verify-source

pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" verify-model --profile medium \
  --model "$PI_REPO/models/cv-baselines/medium/best.onnx"

# Destination must not already exist; model files are deliberately separate.
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" build-bundle --output "$HOME/mdp-cv/cv-baseline-bundle"

cd "$HOME/mdp-cv/cv-baseline-bundle"
sha256sum -c SHA256SUMS
```

The source-copy builder includes both profiles and their tests. Copy a separately
verified model when moving a bundle to another computer, and supply its explicit path
when launching. It never deploys or starts anything itself. The generated checksum
manifest covers the copied files, excluding its own checksum, cache files and locks.

## Manual foreground candidate start

Follow the full scene and process preparation in the [Pi manual](../../docs/cv/pi-testing.md)
first. On a stationary robot with motors disabled and the camera/router available:

```bash
"$CV_BUNDLE/start_candidate.sh" "$PI_WS" nano \
  "$PI_REPO/models/cv-baselines/nano/best.onnx" \
  "$HOME/mdp-cv/reports/nano-$(date -u +%Y%m%dT%H%M%SZ)"
```

For medium, substitute `medium` and `models/cv-baselines/medium/best.onnx` and give it
a new session directory. All three filesystem arguments must be absolute. The command
runs in the foreground; use **Ctrl+C in that same terminal** to stop it. No broad process
killer or automatic background launcher is provided.

`candidate.py start` is the lower-level equivalent for an already activated correct
Python environment. It checks model/source/dependencies, locks this bundle, refuses an
existing session directory, writes `launch-record.json`, and imports only the chosen
historical runtime. The lock is released when the process exits; the lock file's mere
existence does not mean the process is running.

## Evidence output and summarizer

A session contains:

- `launch-record.json`: chosen profile, model path, immutable profile values, runtime
  source directory and UTC start time from the new wrapper.
- `run-config.json`: historical runtime parameters and actual model hash.
- `observations.jsonl`: one line per processed result that reaches the recorded gate.
- `*-obsN-idS.json`: accepted event, confidence, timing, model identity and image name.
- `*-raw.png` and `*-annotated.jpg`: exact original accepted frame and annotated view.

Normal node logs are written to the terminal; use your terminal's capture feature or
`tee` if a text log is needed. They are not automatically named `live.log` by this new
wrapper. Freshness-rejected work may not produce an observation row, so preserve console
logs when investigating missing publications.

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" summarize \
  "$HOME/mdp-cv/reports/YOUR_SESSION/observations.jsonl" \
  > "$HOME/mdp-cv/reports/YOUR_SESSION/summary.json"
```

This reads the entire JSONL log and prints JSON to stdout. It rejects malformed,
nonfinite or backward-clock timing instead of silently producing a false rate.
`observed_fps` is `(number of rows - 1) / (last time - first time)`; one row produces
`null`. Publication offsets begin at the first processed result, not the physical
instant a card entered view. The tool cannot calculate field accuracy, false-positive
rate or card-presentation latency without a separately labelled trial record.

For the retained 142-row physical F log, it yields whole-log median **913.4 ms**,
**1.059 processed rows/s**, and one accepted `1,25` publication. The historical recent
window summary reports **913.7 ms / 1.041 FPS**. These use different windows and do not
contradict each other.

## Offline test commands

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  -m unittest discover -s "$CV_BUNDLE/tests" -v

PYTHONPATH="$CV_BUNDLE/runtime/nano" \
  pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  -m unittest discover -s "$CV_BUNDLE/runtime/nano/test" -v

PYTHONPATH="$CV_BUNDLE/runtime/medium" \
  pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  -m unittest discover -s "$CV_BUNDLE/runtime/medium/test" -v
```

The new tool suite has eight tests. Each historical profile has 29 hardware-free tests;
medium has one additional ROS worker test. Report that worker test as **skipped** if
ROS is absent, not passed. It uses a fake detector and creates no camera or motor node.

## Failure meanings

| Failure | Meaning/action |
|---|---|
| Model size mismatch | Often an unfetched LFS pointer or a wrong candidate; run `git lfs pull`, then recheck the profile/path |
| Model SHA mismatch | Wrong or corrupted bytes; do not relabel them as this baseline |
| Archived runtime differs | Historical source changed; restore the reviewed copy or create a separately documented implementation |
| Missing dependencies | Wrong interpreter or incomplete Pixi environment; repair the declared environment instead of system Python |
| Session directory exists | Select a new timestamped path; previous evidence is preserved |
| Candidate already running | Stop that bundle's foreground process; do not delete a lock to bypass a live process |
| No publications | Follow camera, confidence, freshness, inversion and timing checks in the Pi manual; a detected frame need not be an accepted event |

This folder contains reproduction tooling and historical code. It does not establish
that the current Pi filesystem, camera environment or moving-robot integration matches
the September experiments.
