# Live Raspberry Pi candidate

This bundle automatically recognizes the black-on-white target cards using the full-baseline ONNX model and color inversion. It is an isolated stationary test deployment, not a mission release.

## Installed locations

- Bundle: `/home/mdp/mdp-cv/live-runtime/live-v1-20260909`
- ROS environment: `/home/mdp/dev/SC2079-Group-16/ros2_ws`
- Model: `/home/mdp/mdp-cv/candidates/baseline-yolov8n-v61-full-20260908T142129Z-r2/best.onnx`
- Logs and evidence: `/home/mdp/mdp-cv/reports/live-v1-20260909`

## Watch results on the Pi

```bash
tail -n 30 -f /home/mdp/mdp-cv/reports/live-v1-20260909/live.log
```

For only accepted target messages, in another Pi terminal:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
pixi run --frozen -e pi ros2 topic echo /test/android/target
```

Publishing is automatic. There is no sample-target service in this node. A new subscriber will see future events, not old ones. Hold the card for roughly 3-5 seconds. Holding the same card longer does not repeatedly publish it. To repeat the same card, remove it for at least 4 seconds of clearly visible blank background, then present it again.

## Start and stop

The camera and Zenoh router must already be running. If needed, start each in a separate Pi terminal from the ROS workspace:

```bash
pixi run --frozen -e pi zenoh
pixi run --frozen -e pi camera
```

Do not start duplicates of those processes. Start the live detector in a third terminal:

```bash
/home/mdp/mdp-cv/live-runtime/live-v1-20260909/start-live.sh
```

Only one process from this bundle can run at a time. Stop its current process using:

```bash
/home/mdp/mdp-cv/live-runtime/live-v1-20260909/stop-live.sh
```

Stopping the detector does not stop the camera/router. This bundle does not install a boot service.

## Obstacle number

The default is obstacle 1, so F publishes `1,25`. To change the obstacle number while running:

```bash
pixi run --frozen -e pi ros2 param set /perception_live_candidate obstacle_id 2
```

Changing the obstacle number resets confirmation, rejects results from the previous obstacle, and requires newly received frames. This is manual stationary-test association; planner request integration and moving missions remain separate work.

## Behavior

- Input: `/camera/image_raw`; original camera topic stays unchanged.
- Color inversion is applied once inside inference; annotations/evidence use original colors.
- Requested inference limit: 4 FPS. Actual measured throughput on this Pi is about 1 FPS. ONNX inference does not overlap; camera frames are replaced with the latest received frame instead of queuing inference work.
- Confidence: 0.50; confirmation: three successive processed frames with the same official ID.
- Misses or stale frames break confirmation. More than a three-second gap between processed observations also breaks it.
- The same accepted ID is suppressed until sustained no-target observations span two seconds, or a different ID is confirmed, or the obstacle changes.
- ID 40 is a valid filled-circle target. The distinct marker class is filtered and cannot directly become an official ID.
- Uses model metadata and fails on an incompatible model instead of silently entering mock mode.
- Publishes `/test/android/target` and compressed annotations on `/test/perception/image_annotated`.
- Saves exact accepted original frames, annotated images, model hash, obstacle ID and confidence. `observations.jsonl` records per-inference decisions.

## Validation and limits

29 unit tests passed locally and on the Pi. An automatic replay with real ONNX inference on the Pi passed F acceptance, duplicate suppression, background rejection, F reappearance, digit 1 recognition, and obstacle-number reset. Physical live F also published automatically.

The original dataset uses the opposite target polarity; whole-frame inversion is a candidate adaptation that still needs broader checks on all cards, backgrounds, distances and lighting. The Pi camera timestamps frame publication, not verified sensor exposure, so timestamp checks do not establish true acquisition freshness. The current detector's existing edge-box clipping and NMS behavior remain unchanged. Original training dataset provenance/leakage limitations remain. No claim of mission readiness or four actual inference frames per second is made.
