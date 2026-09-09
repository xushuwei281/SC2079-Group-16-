# Build and restore the live Pi candidate

The camera pipeline previously required a sample service call. This candidate continuously processes the latest available frame and publishes an official ID after three matching processed frames. For example, a held black-on-white F card produces `1,25` once; removing it for a sustained blank interval and returning it allows another publication.

The running deployment and manual commands are described in [LIVE_TESTING.md](../../ros2_ws/src/mdp_perception/LIVE_TESTING.md). This directory contains the exact launcher/wrapper scripts used for the isolated deployment and a builder that copies the version-controlled Python sources. It does not replace the installed mission node or its planner service.

## Build on a machine with Python 3

From the repository root:

```bash
python3 deployment/live-pi/build_bundle.py --output /tmp/mdp-live-candidate
```

The output directory must not already exist. The bundle contains the Python package, tests, launcher, process lock, stop script, documentation and checksums. Model weights, credentials, generated images, logs and virtual environments are not included.

## Model

See [model-reference.json](model-reference.json) for the training job, exact artifact location, class order, and PT/ONNX SHA-256 values. The tested full-baseline ONNX is required. An arbitrary file named `best.onnx` is not interchangeable with it.

Export settings: image size 640, batch 1, FP32, opset 18, static dimensions, no embedded NMS, simplified graph. Input is `[1,3,640,640]`; output is `[1,35,8400]`. The ordered names are `11` through `40`, followed by `marker`. The loader validates this contract and fails if it is missing or incompatible.

## Deploy to the existing Pi environment

The tested Pi workspace is `/home/mdp/dev/SC2079-Group-16/ros2_ws`. It already provides ROS, OpenCV, ONNX Runtime and Picamera2. A clean OS installation has not been validated by this change. The saved Pi configuration snapshot retains its Pixi manifest, lock, activation scripts, camera code and Python package inventory.

Copy the bundle to a new directory under `/home/mdp/mdp-cv/live-runtime/` using your authorized SSH account. Verify it on the Pi:

```bash
cd /home/mdp/mdp-cv/live-runtime/NEW_BUNDLE_DIRECTORY
sha256sum -c SHA256SUMS
```

Inspect `start-live.sh` if your model or ROS workspace is in a different location. Stop the previous candidate with its `stop-live.sh` before starting the new one; the lock prevents duplicates within the same bundle, not across different bundles. Stop any earlier color-inversion adapter too: this node inverts the original camera frame internally.

Keep the router and camera running, then run `start-live.sh`. It publishes only to `/test/android/target` by default. It creates no boot service and does not launch motors, planner, serial bridge or Android bridge.

## Save and restore runtime settings

On the Pi, with the live node running and from the ROS workspace:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
pixi run --frozen -e pi ros2 param dump /perception_live_candidate > live-parameters.yaml
```

On a later start, pass that saved file:

```bash
/path/to/bundle/start-live.sh --params-file /absolute/path/to/live-parameters.yaml
```

This restores configuration such as obstacle ID, confidence, inversion and frame limits. It does not restore old detections or confirmation history. See the Pi snapshot's `RESTORE.md` for its exact locations. Start the router and camera separately after a reboot.

## Validation

- 29 class, loader and confirmation tests passed locally and on the Pi.
- Real ONNX inference on replayed camera images passed automatic publishing, held-card duplicate suppression, background rejection, reappearance, symbol changes and obstacle-number reset.
- Live F was accepted as ID 25 at 92.4% confidence and suppressed over continued observations.
- Requested cap: 4 FPS. Measured live throughput: about 1.04 FPS, with median inference around 914 ms. Three-frame confirmation takes about three seconds on this Pi.

Recorded summaries are in [docs/validation/live-pi-20260909](../../docs/validation/live-pi-20260909). These are sample-level checks, not an accuracy benchmark or mission approval. All-card/marker physical testing, varied environments, and moving-mission association remain pending. Dataset provenance and split leakage remain unresolved. Color inversion was verified only on the tested cards/backgrounds; keep broader validation pending.

## Tests

From the repository root with Python available:

```bash
PYTHONPATH=ros2_ws/src/mdp_perception python3 -m unittest discover \
  -s ros2_ws/src/mdp_perception/test -p 'test_*contract.py' -v
PYTHONPATH=ros2_ws/src/mdp_perception python3 -m unittest discover \
  -s ros2_ws/src/mdp_perception/test -p 'test_live_confirmation.py' -v
```

The class/loader tests use small fakes, so native ROS/GPU dependencies are not required. Live execution and the recorded replay validation require the Pi environment.
