# Raspberry Pi CV deployment, live testing and evidence protocol

This manual covers the historical nano and medium baseline candidates packaged in
[`deployment/cv-baselines`](../../deployment/cv-baselines/README.md). They reproduce
the September 2026 experiments without replacing the newer active ROS perception
package. Commands below are **manual test instructions**, not a claim that another
physical test has been performed.

## 1. What was actually demonstrated

| Experiment | Input and location | Recorded result | Limit |
|---|---|---|---|
| Initial nano digit 1 tests | Physical Pi camera, 9 September | Two `UNCERTAIN` results | Original image polarity did not produce a detection |
| Polarity diagnosis | Saved physical image, PyTorch/ONNX checks | Inversion recovered ID 11, about 0.944 confidence | Diagnosis on an existing photograph |
| Nano stationary service checks | Physical Pi camera, inversion enabled | Digit 1 → `1,11`, F → `2,25`, background → no target | Two positive symbols and one background, not all-class accuracy |
| Nano automatic live test | Physical Pi camera and F card | Three processed observations confirmed `1,25`; 142 retained observations, one publication | Stationary physical test; held-card duplicate suppression demonstrated |
| Nano behavioural replay | Saved Pi images replayed through ROS on Pi | Confirmation, background rejection, duplicate suppression, reappearance, symbol switch and obstacle reset passed | No new physical camera conditions |
| Medium standalone benchmark | Saved images on Pi CPU | 3,766 ms median with four threads | Inference-only benchmark on three scenes |
| Medium final ROS replay | Saved images, real medium ONNX on Pi | All recorded behavioural checks passed; about 0.20 processed FPS | Fresh physical medium testing remained pending |
| Autonomous bullseye approach/orbit | Separate add-on image tests and simulation | Software evidence retained elsewhere | No completed physical moving-robot demonstration established |

The physical service results were approximately:

| Case | Confidence | Service round trip | Output |
|---|---:|---:|---|
| Original digit 1 | 0 | 893 ms | None |
| Original digit 1 retry | 0 | 874 ms | None |
| Inverted-input digit 1 | 0.9520 | 1,230 ms | `1,11` |
| Inverted-input F | 0.9361 | 1,180 ms | `2,25` |
| Inverted-input background | 0 | 1,118 ms | None |

Service round-trip time includes work beyond model inference. Photographs retained
near those service requests are not guaranteed to be the exact service input frames.
The later live node records the exact accepted original frame, which improves traceability.

Sources: [stationary digit 1](evidence/2026-09-09/stationary/digit1-live-inverted-result.json),
[stationary F](evidence/2026-09-09/stationary/letter-f-live-inverted-result.json),
[background](evidence/2026-09-09/stationary/background-live-inverted-result.json),
[physical live summary](evidence/2026-09-09/live/validation-summary.json),
[nano replay](evidence/2026-09-09/replay-result.json),
[medium replay](evidence/2026-09-14/replay/replay-result.json).

![Exact original frame accepted during automatic physical F recognition](evidence/2026-09-09/live/1788933672803107938-obs1-id25-raw.png)

This accepted F event had confidence **0.9241**, official ID **25**, and three processed
confirmations. The raw image retains original colours; inversion occurred only in the
model-input path. [Event record](evidence/2026-09-09/live/1788933672803107938-obs1-id25.json).

## 2. Model and class contract

Use the model that belongs to the selected profile, after fetching its LFS object:

| Profile | Repository file | Size | SHA-256 |
|---|---|---:|---|
| Nano | `models/cv-baselines/nano/best.onnx` | 12,288,971 bytes | `0207a7fb6ab117ce0a62fd683185da8d8ca065c92f1a0ee1d155e65f14e2c48a` |
| Medium | `models/cv-baselines/medium/best.onnx` | 103,694,827 bytes | `37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031` |

Both have static FP32 `[1,3,640,640]` input, `[1,35,8400]` output, batch 1, opset 18,
no embedded NMS and 31 classes. The runtime performs letterboxing, colour conversion,
normalization, confidence filtering and NMS. Canonical names are strings `11` to `40`
followed by `marker`. Index 0 means official ID 11; index 29 means official ID 40;
index 30 is the separate marker class. **ID 40 is the filled circle, not the bullseye
marker.** The historical canonical loader filters the marker out of accepted official
symbol publications. An older runtime using a different mapping cannot safely infer
this meaning from index numbers alone.

The output message is `obstacle_id,symbol_id`. For example, `1,25` means obstacle 1,
symbol F. Obstacle number is a manually set test association, not visual localization.
These historical `/test/...` topics do not automatically establish delivery to the
tablet, mission planner or obstacle map.

## 3. How automatic live recognition works

```text
Camera publishes /camera/image_raw
  → subscriber stores newest received image and sequence number
  → scheduled inference attempt selects an unprocessed fresh image
  → invert once (for the tested black-on-white cards)
  → model inference, confidence/NMS, canonical class interpretation
  → reject stale results or work belonging to a previous obstacle number
  → require 3 successive distinct processed observations of the same symbol
  → publish once, save accepted image/event, suppress held-card duplicates
  → rearm after sustained processed absence, a confirmed different symbol,
    or an explicit obstacle-number change
```

A camera producing around 15 FPS does not mean YOLO processes 15 images each second.
The runtime chooses recent frames instead of queuing every image. Three confirmations
mean three **processed observations**, not three timer ticks or reuse of one frame.
Missing/stale images break confirmation; no incoming camera image is not proof that a
card physically disappeared. Absence reset requires actual processed no-target results.

The nano archive preserves its original multithreaded ROS executor and callback-path
inference. The medium archive adds a single inference worker and keeps subscription
and parameter callbacks short. Timer ticks while that worker is busy cannot queue a
backlog. Obstacle changes increment a generation value so an older in-flight result
cannot be relabelled with the new obstacle number. This worker behaviour was tested
with deliberately blocked inference and repeated timer ticks.

Camera timestamps here represent message publication, not independently verified
sensor exposure time. Receipt/publication freshness gates cannot rule out upstream
camera buffering. That is one reason these profiles remain stationary experiments.

## 4. Recorded profiles and performance

The launcher loads [`profiles.json`](../../deployment/cv-baselines/profiles.json):

| Setting | Nano | Medium | Meaning |
|---|---:|---:|---|
| Confidence | 0.50 | 0.50 | Minimum candidate detection confidence |
| Inference scheduling limit | 4 FPS | 1 FPS | Upper scheduling limit, not achieved throughput |
| Confirmation observations | 3 | 3 | Same symbol in distinct successive processed frames |
| ONNX CPU threads | 2 | 4 | Recorded inference thread configuration |
| Input age limit | 1.0 s | 1.0 s | Reject old inputs before inference |
| Result age limit | 2.5 s | 8.0 s | Reject results too old when inference completes |
| Confirmation gap limit | 3.0 s | 8.0 s | Break an interrupted sequence |
| Processed absence reset | 2.0 s | 2.0 s | Rearm after sustained no-target observations |
| Colour inversion | true | true | Applied once internally |
| Initial obstacle number | 1 | 1 | Manual association |

| Measurement | Result | What the number means |
|---|---:|---|
| Nano standalone, two threads | 851.9 ms median | Saved-image CPU inference |
| Nano physical live, recent window | 913.7 ms / 1.041 FPS | Historical recent observation summary |
| Nano entire retained physical log | 913.4 ms / 1.059 FPS | 142 rows over 133.16 s; rate `(142−1)/span` |
| Medium standalone, two threads | 4,843.5 ms median | Saved-image CPU inference |
| Medium standalone, four threads | 3,766.1 ms median | Saved-image CPU inference |
| Medium ROS replay, four threads | 4,377.8 ms / 0.1985 FPS | Real inference with replay/ROS overhead |
| Medium first replay F acceptance | 16.65 s | Since replay start; not a new physical camera test |

The nine-inference standalone comparisons used saved digit 1, F and background scenes
after warm-up. They do not provide a broad accuracy estimate. Medium was about 4.4 times
slower than nano in the four-thread/two-thread standalone comparison, despite good
dataset metrics. Early medium replay runs under load exceeded deadlines and withheld
publication; the final wider stationary timing profile allowed completion.

An 8-second result deadline also means a result can be several seconds old when it is
published. Do not treat it as a validated scene association while driving. Cloud mAP
near 0.99 does not establish 99% field accuracy: the training data had annotation and
split-leakage limitations, and the physical sample set was small.

Sources: [benchmark](evidence/2026-09-14/pi-benchmark.json),
[medium completion](evidence/2026-09-14/medium-completion-summary.json),
[physical observations](evidence/2026-09-09/live/observations.jsonl).

## 5. Before using the Pi again

The original historical checkout was `/home/mdp/dev/SC2079-Group-16/ros2_ws`.
Later access work found a changed Pi endpoint and missing historical workspace paths.
A working SSH connection alone does not establish that these files or dependencies
are installed. Verify your current checkout and hardware before using a historic command.

Keep motor power disabled while the Pi and camera remain powered for this stationary
protocol. Have one operator own the test. Check existing processes and terminals;
do not start duplicate camera, router or detector instances. Do not launch a full robot
mission for this CV-only test. The repository's agent instructions reserve live
launches for manual user execution.

All commands below are for an operator's Pi terminal. Adjust `PI_REPO` if the checkout
is elsewhere, and define these variables in **each terminal**:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
export PI_REPO=/home/mdp/dev/SC2079-Group-16
export PI_WS="$PI_REPO/ros2_ws"
export CV_BUNDLE="$PI_REPO/deployment/cv-baselines"
export ROS_DOMAIN_ID=16
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
test -f "$PI_WS/pixi.toml"
test -f "$CV_BUNDLE/candidate.py"
```

Domain 16 is the historical project environment. A deliberately isolated test domain
must be used consistently by router, camera, detector and observer; changing only one
terminal prevents communication. Domain separation alone does not free a camera that
another process already owns.

Fetch and inspect the files, without starting hardware:

```bash
cd "$PI_REPO"
git lfs pull --include='models/cv-baselines/**'
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" preflight --profile nano \
  --model "$PI_REPO/models/cv-baselines/nano/best.onnx" --check-dependencies
```

Use `--profile medium` and the medium file to inspect medium instead. Preflight checks
model bytes, archived source hashes and discoverable dependencies; it deliberately
does not certify the camera, actual inference speed or physical scene. If the workspace
or environment is missing, restore/build it using the repository setup instructions
before attempting a live test. Do not overwrite an existing user's checkout to recreate
the historical directory name.

## 6. Manual start and direct on-Pi display

In terminal 1, only if a router is not already running:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi zenoh
```

In terminal 2, only if a camera publisher is not already running:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi camera
```

In terminal 3, start **one** detector in the foreground:

```bash
"$CV_BUNDLE/start_candidate.sh" "$PI_WS" nano \
  "$PI_REPO/models/cv-baselines/nano/best.onnx" \
  "$HOME/mdp-cv/reports/nano-$(date -u +%Y%m%dT%H%M%SZ)"
```

For medium, stop nano with Ctrl+C first, then run:

```bash
"$CV_BUNDLE/start_candidate.sh" "$PI_WS" medium \
  "$PI_REPO/models/cv-baselines/medium/best.onnx" \
  "$HOME/mdp-cv/reports/medium-$(date -u +%Y%m%dT%H%M%SZ)"
```

In terminal 4, display accepted results directly on the Pi:

```bash
# Nano
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi \
  ros2 topic echo /test/android/target

# Use this instead for medium:
# pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi \
#   ros2 topic echo /test/medium/android/target
```

Expected messages include `data: 1,25` for F and `data: 1,11` for digit 1. A held
confirmed card does not continuously emit repeated messages; that silence is intentional.
The observer receives future events, so start it before presenting the card. A sample
service call is unnecessary for these live nodes.

For an annotated visual display, if the Pixi environment has `rqt_image_view` and the
Pi has a graphical session, start it manually:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi \
  ros2 run rqt_image_view rqt_image_view
```

The candidate publishes `sensor_msgs/msg/CompressedImage` directly at
`/test/perception/image_annotated` or `/test/medium/perception/image_annotated`.
These names lack the conventional `/compressed` suffix, so standard image-transport
viewers may require explicit remapping or a compatible subscriber. This viewer path
has not been validated by the recorded tests; the optional package is not installed
by this bundle. For the supported evidence display, use terminal events and open
the accepted `*-annotated.jpg` files afterward. Subscriber-driven annotation updates
follow inference throughput, not camera capture FPS. Viewing images adds work, so record
whether it was enabled in performance comparisons.

## 7. Repeatable stationary acceptance protocol

Use the same black-on-white printed cards when reproducing the inversion configuration.
Record measured distance, card angle, illumination, background, print dimensions and
camera mounting. Those measurements were not established by the original held-card test.

| Step | Action | Expected result |
|---|---|---|
| 1 | Start with an unobstructed no-target scene | No target publication |
| 2 | Present F steadily | After three processed confirmations, one `1,25` |
| 3 | Keep F still | No duplicate spam |
| 4 | Remove F and show the background | No accepted symbol; processed absence rearms |
| 5 | Present F again | One new `1,25` after confirmation |
| 6 | Replace F with digit 1 | One `1,11` after confirmation |
| 7 | Change obstacle number to 2 while observing digit 1 | Fresh confirmation produces `2,11`; old work must not be reused |
| 8 | Present the marker, then the filled circle as separate cases | Marker produces no official target; filled circle can be accepted as ID 40 |
| 9 | Remove all cards and stop | Retain complete logs and evidence, including wrong/no results |

Start with 3–5 seconds held and at least 4 seconds absent for nano. For medium, allow
at least 20 seconds held and 15 seconds absent. These are practical starting intervals
derived from recorded performance, not guaranteed maximum delays. Extend the observation
window if the logs show CPU load or slow inference, and record the change.

To set the manual obstacle association while the nano node is running:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi \
  ros2 param set /perception_live_candidate obstacle_id 2
```

For medium use `/perception_medium_candidate`. Other profile parameters are read-only
at runtime and require a deliberate new configuration/restart. Do not increase deadlines
just to force a passing moving-robot demonstration.

Stop the detector with **Ctrl+C in its launch terminal**. Stop the camera/router only
if you started them for this test and no other operator depends on them. This candidate
does not send motor commands or clear an E-STOP. Do not use a broad process-kill command
as part of normal evidence collection.

## 8. Preserve evidence and assess outcomes

Save the current ROS parameters while the candidate is still running, using the new
session directory shown in the launch command:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi \
  ros2 param dump /perception_live_candidate \
  > "$HOME/mdp-cv/reports/YOUR_SESSION/saved-parameters.yaml"
```

For medium substitute its node name. After stopping, summarize the session:

```bash
pixi run --frozen --manifest-path "$PI_WS/pixi.toml" -e pi python \
  "$CV_BUNDLE/candidate.py" summarize \
  "$HOME/mdp-cv/reports/YOUR_SESSION/observations.jsonl" \
  > "$HOME/mdp-cv/reports/YOUR_SESSION/summary.json"
```

Keep the model hash, repository commit, `launch-record.json`, `run-config.json`, saved
parameters, observation JSONL, all accepted event JSON files, raw/annotated images and
terminal logs together. Give every repeat a new directory; the launcher refuses existing
directories to prevent evidence mixing.

Create a separate ground-truth trial table. A suitable CSV header is:

```csv
session,trial,model_sha256,source_commit,expected_symbol,obstacle_id,face_id,distance_cm,angle_deg,lighting,background,presented_at,first_correct_at,accepted_ids,wrong_accepts,duplicate_count,timeout,evidence_files,notes
```

Measure time from the **actual card presentation**, ideally synchronized with a recording.
The JSONL summarizer only knows when processed observations were logged. Count wrong
accepted IDs separately from missing/timeout results and repeated duplicate messages.
Do not divide the number of observation rows by target publications to call it accuracy.

For broader evaluation, cover all official symbols plus the separate marker and true
negative scenes, multiple measured distances including 20–50 cm where relevant to the
checklist, view angles, lighting, glare, partial occlusion and backgrounds. Keep the same
trial protocol for nano and medium. Retain independent physical scenes not used for
training, selection or threshold tuning. Log temperature/throttling and CPU/memory load
when comparing sustained performance.

## 9. Troubleshooting by symptom

| Symptom | Check | Interpretation/action |
|---|---|---|
| `best.onnx` is a tiny text file | Run model verification and inspect Git LFS status | Fetch actual LFS content before starting |
| Model load fails | Confirm exact profile, hash and ONNX metadata | Wrong model/layout should fail, not silently fall back |
| No camera frames | Inspect `/camera/image_raw` and router/domain consistency | One camera publisher, correct environment and matching transport are required |
| High camera FPS, low detection FPS | Compare inference timing with the profile table | Camera rate and model throughput are different measurements |
| Initial black-on-white digit 1 is missed | Verify internal inversion is enabled once | Do not run a second inversion publisher at the same time |
| Detections appear but no event | Inspect confidence, confirmations and freshness | A single frame is intentionally insufficient |
| Repeated F produces no new message | Check whether a processed absence or different symbol occurred | Held-card duplicate suppression is expected |
| Medium never confirms | Inspect result age, input age and actual inference duration | Under-load results may exceed 8 s and be rejected; do not claim a pass without fresh results |
| Old obstacle appears in output | Confirm obstacle parameter change and generation handling | Preserve the log; investigate before mission integration |
| Marker reported as 40 | Check which runtime/class mapping is loaded | Historical canonical contract reserves 40 for the filled circle |
| New subscriber sees nothing | Present a new card/rearm and observe future events | This is an event stream, not a retained current-target display |
| No annotation topic updates | Verify a compatible compressed-image subscriber | Annotation publication is subscriber-driven and follows processed frames |
| Historical paths absent after SSH | Verify actual device and checkout | Restore intentionally; old successful access is not current deployment proof |

## 10. Accountability statement

The supported claim is: **nano and medium YOLOv8 baselines were trained and exported;
their deployment contracts were checked; stationary physical nano recognition was
demonstrated on the Pi; medium passed saved-image inference and ROS replay on the Pi.**
The data and tests do not establish all-symbol field accuracy, four inference FPS on the
Pi, moving-robot image association or a completed physical bullseye navigation circuit.

The current robot's wider checklist state remains documented separately in the
[checklist compliance audit](../checklist-compliance-audit.md). Source implementation,
offline tests and a physical demonstration are different evidence categories.
