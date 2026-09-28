# Historical bullseye navigation experiment — v0.2

This directory preserves the separate camera-guided navigation experiment developed alongside the
September 2026 CV baselines. It is included for accountability and offline reproduction. It is not
the current robot's checklist A.5 or Task 1 implementation and is not added to ROS launch defaults.

**Recorded status:** 29 local core/startup tests passed; saved-image ROS preview was tested on the
Mac. An earlier Pi bundle passed 18/20 tests before a planner timing optimisation. Final Pi transfer
and retesting were not verified. Camera calibration, live approach and physical orbit were not
validated. Default configuration refuses motion until the required measured setup is provided.

See [repository-wide status](../../docs/cv/bullseye-status.md) for the distinction from current
`ros2_ws` navigation code. The historical 0.21 m turning-radius configuration must not be assumed
to match the current robot's later calibration.

## Script and module inventory

| File | Purpose and usage |
|---|---|
| `run.py` | CLI for saved-image detection, readiness, ROS preview and explicitly selected mission mode; run `python run.py --help` |
| `calibrate_camera.py` | Derives intrinsics from supplied checkerboard photographs; see `--help` and the historical guide for required measurements |
| `start_task.py` | Supervises a complete historical session; validates configuration and owns/cleans up only processes it starts |
| `start_bullseye.sh` | Activates the configured Pi environment and forwards arguments to `start_task.py` |
| `launch_preview.sh` | Historical preview wrapper; camera/router prerequisites are documented below |
| `launch_mission.sh` | Historical explicitly selected movement entry point; not run by this upload |
| `bullseye/vision.py` | Concentric-square marker detection and pose geometry |
| `bullseye/geometry.py` | Geometry, configuration-space and collision helpers |
| `bullseye/planner.py` | Bounded path search and trajectory generation |
| `bullseye/mission.py` | Approach/face-search state machine and freshness rules |
| `bullseye/symbols.py` | Strict canonical ONNX symbol reader |
| `bullseye/ros_node.py` | ROS adaptation for preview and separately enabled movement |
| `config/camera_calibration.yaml` | Required camera measurements; shipped incomplete |
| `config/mission.yaml` | Historical motion/geometry settings and required verification fields |
| `tests/test_core.py` | Offline geometry, state, collision and synthetic mission regression cases |
| `tests/test_start_task.py` | Offline startup refusal, ownership and cleanup cases |
| `tests/check_saved_cases.py` | Symbol-reader check using explicitly supplied saved-image cases |
| `tests/ros_replay.py` | Saved-image ROS preview harness; requires compatible ROS environment |
| `tests/launcher_replay.py` | Saved-image launcher harness; forbids physical camera startup |

## Source integrity and offline checks

`SOURCE_PROVENANCE.json` records SHA-256 identities of the preserved source/configuration/test
files. `SHA256SUMS` verifies those exact files. Guides have been relinked and labelled historical.

In this directory:

```bash
sha256sum -c SHA256SUMS
python -m unittest discover -s tests -p 'test_*.py' -v
python run.py --help
python calibrate_camera.py --help
```

Use a compatible existing environment with Python 3.12, NumPy, OpenCV and PyYAML for core checks.
ONNX symbol checks additionally require ONNX Runtime and a verified nano model. ROS replay requires
ROS 2 Jazzy, `cv_bridge`, the relevant messages, and a compatible Pixi workspace. Do not install
packages into the Pi system Python. The tests use synthetic/saved inputs and do not establish
physical navigation performance.

## Operational documentation and limits

- [Detailed historical guide](HISTORICAL_GUIDE.md): calibration procedure, coordinate frames,
  geometry, configuration, saved-image checks and outputs.
- [Historical one-command guide](HISTORICAL_START_HERE.md): setup checks, preview and staged
  commissioning, with exact intended Pi directories.
- [CV models](../../models/cv-baselines/README.md): the exact pinned nano file required by this
  experiment. The original guide references its historical Pi candidate path; model transfer and
  environment restoration are separate explicit steps.

Physical commands in those guides are for manual operator execution after setup verification.
Do not start this experiment alongside a current motion controller. It does not search outside
the camera's present field of view. A simulated success is not proof of real clearance, stopping
distance, camera range or odometry accuracy.
