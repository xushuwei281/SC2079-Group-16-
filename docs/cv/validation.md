# Validation of the CV documentation and tooling addition

Prepared on 28 September 2026 against repository `main` revision `a8b69b5`.
This page records validation of the repository additions. Historical training and
physical-test outcomes are separately documented in [results.md](results.md).

## Checks completed

| Check | Result and scope |
|---|---|
| Existing algorithm suite | 13 offline tests passed using the installed Pixi PC Python environment |
| Training/audit/export workflow helpers | 21 offline regression tests passed; no training or export job executed |
| Historical bullseye source | 29 offline core/startup tests passed; no camera or robot movement |
| New Pi packaging/verification utility | 8 offline regression tests passed |
| Preserved nano runtime tests | 29 passed |
| Preserved medium runtime tests | 29 passed; one ROS worker test skipped where `rclpy` was unavailable |
| Historical runtime source integrity | All 17 source/test file hashes matched the provenance manifest |
| Actual nano ONNX identity | 12,288,971 bytes; expected SHA-256 matched |
| Actual medium ONNX identity | 103,694,827 bytes; expected SHA-256 matched |
| Selected evidence integrity | All 30 published files matched their published hashes/sizes; JSON/JSONL parsed |
| Evidence privacy review | Personal workstation roots/network addresses removed where needed; original and published hashes distinguished |
| Documentation links | 18 Markdown files checked; zero missing local file targets |
| Script syntax | New Python syntax and shell syntax checks passed |

The six listed test suites account for 129 passing test executions; the nano and medium
historical profiles intentionally repeat some shared contract cases. The skipped ROS worker
test is excluded from the pass count.

The two hashed historical detector files retain an original whitespace-only blank line.
Their per-file Git whitespace attribute preserves that source identity without changing the
new-code whitespace checks.

The historical experiment's first test invocation used a Pixi manifest whose working directory
was outside the experiment and could not import `bullseye`. Re-running with the experiment
directory explicitly on `PYTHONPATH` passed all 29 tests. Follow the documented working directory
or set the module path explicitly when using a manifest from another checkout.

## Required full Pi suite: platform limitation

The repository-required command was attempted from the new checkout:

```bash
cd ros2_ws
pixi run -e pi test-all
```

Pixi rejected the command before tests ran because the `pi` environment supports Linux ARM64,
while this host is macOS ARM64. This is **not a passing full-suite result**. The original task
recursively selects `-e pi`, so running it from a PC environment would not establish Pi success.
The full Pi suite must still be run on a compatible environment before accepting an operational
robot release. No remote deployment or physical launch was performed for this addition.

The active `ros2_ws/src`, firmware, planner, launch files, Pixi manifests/locks and original model
selection are unchanged. Archived implementations and new workflow utilities are separate.

## Re-run the supplied offline checks

Use the documented dependencies for each tool family and Python 3.12. From repository root:

```bash
python tools/cv/check_documentation.py
python -m unittest discover -s tools/cv/training -p 'test_*.py' -v
python -m unittest discover -s deployment/cv-baselines/tests -v
PYTHONPATH=deployment/cv-baselines/runtime/nano \
  python -m unittest discover -s deployment/cv-baselines/runtime/nano/test -v
PYTHONPATH=deployment/cv-baselines/runtime/medium \
  python -m unittest discover -s deployment/cv-baselines/runtime/medium/test -v
PYTHONPATH=experiments/bullseye-navigation-v0.2 \
  python -m unittest discover -s experiments/bullseye-navigation-v0.2/tests -v
python deployment/cv-baselines/candidate.py verify-source
git lfs fsck
git diff --check
```

On the Pi use the frozen Pixi `pi` environment rather than system Python. The medium worker
test must be reported as skipped when its ROS dependencies are absent. Dataset/model inference
dependencies are not implied by a successful standard-library packaging test.

This change does not rerun GPU training, independent field accuracy, physical navigation,
camera calibration, live middleware transport, clean-machine installation or restored-Pi startup.
