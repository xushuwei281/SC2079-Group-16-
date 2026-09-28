# Computer vision workflow, scripts and evidence

This is the entry point for the September 2026 CV work: dataset audit, Hugging Face training,
ONNX export, Raspberry Pi inference, live camera testing and recorded results. It supports both
reproducing the experiments and showing what work was completed.

**Read the evidence scope first:** nano was tested with physical digit 1/F cards and a background
scene; medium passed saved-image inference and ROS replay on the Pi. Neither result is an all-class
field-accuracy study. The original training splits have known leakage and annotation issues.

## Choose a document

| Document | What it answers |
|---|---|
| [Work history](history.md) | What was done, why it changed, which failures were resolved and what remains |
| [Training and data workflow](training.md) | How to retrieve/audit data, run smoke/full training, export and compare backends |
| [Pi testing](pi-testing.md) | How to verify/build a candidate and manually perform stationary live tests |
| [Results and evidence](results.md) | Training metrics, physical examples, replay results, timings and limitations |
| [Bullseye status](bullseye-status.md) | Historical experiment versus the newer repository navigation code |
| [Validation of this addition](validation.md) | Checks performed while preparing these files and platform limits |

## Files included in this repository

| Path | Contents |
|---|---|
| [`tools/cv/training/`](../../tools/cv/training/) | Dataset audit, training, export, comparison tools and their tests |
| [`deployment/cv-baselines/`](../../deployment/cv-baselines/) | Isolated historical nano/medium runtime source, profiles, packaging/verification tools and tests |
| [`models/cv-baselines/`](../../models/cv-baselines/) | Both exact ONNX models through Git LFS, manifests and SHA-256 files |
| [`evidence/`](evidence/) | Selected small records and camera images supporting the reported work |
| [`experiments/bullseye-navigation-v0.2/`](../../experiments/bullseye-navigation-v0.2/) | Separate historical navigation experiment, calibration/planning scripts and offline tests |

Each script family has its own README with purpose, dependencies, inputs, outputs, usage and failure
behaviour. Use those instructions for the supplied tools rather than assuming old workstation
directory layouts.

## Get the models

From this repository's checkout, with Git LFS installed:

```bash
git lfs pull --include="models/cv-baselines/*/best.onnx" --exclude=""
git lfs fsck
```

Then follow the [model verification instructions](../../models/cv-baselines/README.md). The model
files are approximately 12.29 MB (nano) and 103.69 MB (medium). A small LFS pointer is not a model.

## Reproduction order

1. Read the dataset limitations and results before interpreting the high training scores.
2. Retrieve the intended input data or use the provided verified ONNX candidates.
3. Run offline model/contract checks and saved-image tests.
4. Build the isolated candidate bundle using its documented model/profile settings.
5. On a prepared stationary Pi, let the operator start camera/router and one detector manually.
6. Save the configuration, model identity, exact accepted frames, observations and test outcomes.
7. Record a new session separately from the historical evidence.

Training is optional when demonstrating the existing result. These scripts do not automatically
submit paid jobs, overwrite cloud outputs or replace the robot's active model. The full dataset,
virtual environments, caches, credentials, checkpoint archives and operating-system snapshots are
not copied into Git. The ONNX files and retained manifests identify the tested inference artifacts.

## Current robot code and historical candidates

This addition is based on `main` commit `a8b69b5`. That branch contains newer perception, NCNN,
tracking, checklist A.5 and bullseye/orbit code than the September 9/14 candidate experiments.
The historical runtime is deliberately packaged separately; it is not installed over
`ros2_ws/src/mdp_perception` and is not selected by the current launch files.

The historical models require the canonical 31 names `11` through `40`, followed by `marker`.
Filled circle is official ID 40; marker is a separate class. The current source's downstream
compatibility must be reviewed before substituting a model into a mission. See
[bullseye status](bullseye-status.md) for the remaining distinctions.

No new physical robot test is claimed by this documentation/upload change. Historical failures,
the successful limited camera tests, simulations and pending work are labelled separately.
