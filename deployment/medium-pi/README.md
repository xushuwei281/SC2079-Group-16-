# YOLOv8m stationary Pi candidate

The full medium baseline is exported, validated and available for stationary testing. It uses the existing live detector with a separate node, topic and timing profile. The nano profile remains available because it is substantially faster on this Pi.

## Locations

- ROS workspace: `/home/mdp/dev/SC2079-Group-16/ros2_ws`
- Bundle: `/home/mdp/mdp-cv/live-runtime/medium-v1-20260914`
- Model: `/home/mdp/mdp-cv/candidates/baseline-yolov8m-v61-full-20260911T013018Z/best.onnx`
- Live records: `/home/mdp/mdp-cv/reports/medium-live-20260914`
- Benchmark and replay records: `/home/mdp/mdp-cv/reports/medium-post-training-20260914`
- Node: `/perception_medium_candidate`
- Accepted target topic: `/test/medium/android/target`
- Annotated compressed image topic: `/test/medium/perception/image_annotated`

## Training and files

Job: https://huggingface.co/jobs/xxshuwei/6aa3621921047bf1b03747ec

The run completed 118 epochs, selected epoch 88, and stopped after 30 epochs without improvement. Total job running time was 16,835 seconds (4 hours 40 minutes 35 seconds), including preparation, evaluation and output replacement.

Hugging Face bucket `xxshuwei/jobs-artifacts` contains the canonical run at `baseline-yolov8m-v61-full-20260911T013018Z/`. The previous output prefix now contains the same completed medium artifact set. The original nano files remain under `archives/baseline-yolov8n-v61-full-20260908T142129Z-r2-4506d3d3-before-medium-20260911T013018Z/`.

The cloud job checked the ONNX graph, 31 class names, input `[1,3,640,640]`, output `[1,35,8400]`, and numerical agreement with PyTorch on six test images. It wrote `REPLACEMENT_COMPLETE.json` after copying and verifying the replacement. The medium file is 103,694,827 bytes. Its SHA-256 is `37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031`.

Use `best.onnx` on the Pi. Keep `best.pt`, `last.pt`, metrics, plots and configuration on the workstation/Hugging Face for evaluation and future training. See [model-reference.json](model-reference.json) for exact paths, checksums and metrics. This is the immutable cloud manifest; later Pi evidence is recorded separately.

## Performance and timing

Using the same saved Pi images and the actual deployed detector:

| Model / threads | Median inference | Range | Approximate compute ceiling |
|---|---:|---:|---:|
| Nano / 2 | 852 ms | 847–862 ms | 1.17 images/s |
| Medium / 2 | 4,844 ms | 4,834–5,552 ms | 0.21 images/s |
| Medium / 4 | 3,766 ms | 3,625–3,895 ms | 0.27 images/s |

These are nine inferences per configuration after warm-up on saved digit 1, F and background images, without a live camera. All expected sample decisions passed. They are not a broad accuracy benchmark. CPU load, temperature and a live camera can change the timings.

The completed ROS replay processed about **0.20 images/s**, with median inference **4,378 ms** and a measured range of **4,168–5,329 ms**. First confirmed F appeared after **16.65 seconds**. It passed F confirmation, duplicate suppression, background rejection, F reappearance, digit 1, and obstacle-number reset. Earlier attempts under load exceeded the result deadline (roughly 8–11 seconds) and correctly withheld publication. Do not assume the lower standalone timing applies under load.

The live node now uses a single inference worker. ROS image and parameter callbacks stay short; timer ticks while inference is busy do not queue more work. Changing the obstacle number still invalidates an in-flight result. A Pi test blocked inference, issued 100 timer ticks, changed the obstacle number, and verified both absence of a work backlog and rejection of the outdated result. The 29 existing class, loader and confirmation unit tests also passed.

The medium profile requests at most 1 FPS, uses four ONNX threads and still requires three successive processed frames. Actual throughput is much lower than 1 FPS. Hold a card steadily for at least 20 seconds; remove it for at least 15 seconds before repeating the same card. The nano profile's 2.5-second result deadline would reject medium inference. This stationary profile explicitly uses an 8-second result deadline and 8-second confirmation gap, while retaining the 1-second limit on input age before inference. Results can therefore be several seconds old by publication; this profile is unsuitable for associating symbols with a moving robot without further design and testing.

## Start a physical test

Disable motor power while keeping the Pi and camera powered. Do not start a second detector for the same test or duplicate camera/router processes. The commands below do not launch the robot mission.

In each terminal, enter the ROS workspace first:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
```

If the router is not already running, start it in its own terminal:

```bash
pixi run --frozen -e pi zenoh
```

If the camera is not already running, start it in a second terminal:

```bash
pixi run --frozen -e pi camera
```

Start the medium detector in a third terminal:

```bash
/home/mdp/mdp-cv/live-runtime/medium-v1-20260914/start-live.sh
```

Watch accepted events in another terminal with the workspace/environment above:

```bash
pixi run --frozen -e pi ros2 topic echo /test/medium/android/target
```

Digit 1 should publish `1,11`; F should publish `1,25`. Messages appear only after confirmation, with duplicates suppressed while the same card remains visible. New subscribers see future events. A sample service call is unnecessary. A blank scene should publish nothing.

Change the manually assigned obstacle number:

```bash
pixi run --frozen -e pi ros2 param set /perception_medium_candidate obstacle_id 2
```

Stop only this candidate:

```bash
/home/mdp/mdp-cv/live-runtime/medium-v1-20260914/stop-live.sh
```

The profile does not forward results into mission/planner/Android-control topics or install a boot service. The saved nano bundle can still be started separately using `/home/mdp/mdp-cv/live-runtime/live-v1-20260909/start-live.sh` after stopping medium.

## Rebuild and verify

From the repository root:

```bash
python3 deployment/medium-pi/build_bundle.py --output /tmp/mdp-medium-candidate
```

The destination must be new. Copy the resulting bundle to the Pi path above, then verify:

```bash
cd /home/mdp/mdp-cv/live-runtime/medium-v1-20260914
sha256sum -c SHA256SUMS
cd /home/mdp/mdp-cv/candidates/baseline-yolov8m-v61-full-20260911T013018Z
sha256sum -c PI_SHA256SUMS
```

The worker regression test requires the Pi ROS environment. From the ROS workspace:

```bash
MDP_TEST_RUNTIME=/home/mdp/mdp-cv/live-runtime/medium-v1-20260914/package \
  pixi run --frozen -e pi python \
  /home/mdp/mdp-cv/live-runtime/medium-v1-20260914/package/test/test_live_worker.py -v
```

Save active ROS parameters while the medium node runs:

```bash
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
pixi run --frozen -e pi ros2 param dump /perception_medium_candidate > /home/mdp/mdp-cv/reports/medium-post-training-20260914/saved-parameters.yaml
```

## Remaining evaluation

Whole-frame color inversion is retained for the black-on-white printed cards. The original dataset has duplicate leakage, conflicting annotations and unverified empty labels, so the validation/test scores do not establish independent generalization. Broader physical checks on all official symbols, the separate marker class, backgrounds, distances and lighting remain necessary. A saved-image replay does not substitute for a new physical camera test. Evidence and completion status are in `docs/validation/medium-pi-20260914`.

The candidate is staged and stopped after verification. A Pi reboot occurred after the successful replay; the model and report files survived and were retrieved. No fresh camera observations were claimed. Start this profile manually only for a prepared stationary test.
