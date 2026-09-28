> Historical September 2026 experiment. Not the active repository navigation stack. Read [README.md](README.md) before using these instructions. Physical navigation was not validated.

# Bullseye navigation add-on

This independent module finds the supplied concentric-square marker, estimates
its position using calibration, plans an approach and visits other obstacle
faces until it confirms a numbered symbol. It uses the existing Pi motion
service; it does not modify or import the existing project planner, perception
code or firmware.

**One-command Pi startup is now provided by `start_bullseye.sh`. Read
`HISTORICAL_START_HERE.md` first.** It starts the required components as one supervised
session. The older `launch_preview.sh` / `launch_mission.sh` commands below
start only the add-on and assume the supporting components already exist.

**The shipped configuration prevents motion.** Preview creates no motion service
client, emergency-stop publisher or assessment-result publisher. The motion
entry point is implemented but refuses to start with the shipped incomplete
calibration and physical configuration. Physical driving has not been tested.

## Locations

- Pi add-on: `/home/mdp/mdp-cv/bullseye-navigation`
- Existing ROS environment: `/home/mdp/dev/SC2079-Group-16/ros2_ws`
- New mission configuration: `config/mission.yaml`
- New camera configuration: `config/camera_calibration.yaml`
- Annotated preview: `reports/latest.jpg`
- Current state and missing configuration: `reports/status.json`
- Mission events: `reports/events.jsonl`
- Most recently calculated executable path: `reports/planned_path.json`

No full robot launch, model replacement, boot service, planner restart or motor
command is needed to inspect this add-on. There must already be a camera
publisher and a ROS router for live preview. The saved-image commands below
do not need either.

## 1. Run a saved-image check on the Pi

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
/home/mdp/dev/SC2079-Group-16/ros2_ws/.pixi/envs/pi/bin/python \
  run.py image assets/bullseye-reference.png
```

The result is written to `reports/offline/result.json` and
`reports/offline/annotated.png`. This verifies the reference picture, not a
new camera observation. The detector checks nested contours and agreement
with the original reference after perspective correction. Its quality number
is a matching score, not a probability.

## 2. Run live preview without movement

In a Raspberry Pi terminal:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
./launch_preview.sh
```

For an image window when using a terminal on the Pi desktop:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
./launch_preview.sh --display
```

If OpenCV cannot open a window, the node falls back to the annotated ROS topic
and saved JPEG. Press Ctrl+C to stop this preview. A duration limit is available:

```bash
./launch_preview.sh --seconds 30
```

The preview reads `/camera/image_raw` and publishes:

- `/bullseye/image_annotated/compressed`: annotated images.
- `/bullseye/status`: JSON status and configuration gaps.

Until calibration is supplied, the image shows the marker outline and centre;
metric distance remains unavailable. A reference image or repeated copies of
one old frame are not evidence of calibrated real-world performance.

To read status from a second Pi terminal:

```bash
cat /home/mdp/mdp-cv/bullseye-navigation/reports/status.json
```

Do not start a second camera or router if one is already running. These scripts
only start the new add-on and use the existing frozen Pi environment.

## 3. Collect and calculate camera calibration

Use a flat checkerboard with known internal corner counts and a measured square
side. Move and tilt it through different parts of the image, including near the
edges. Keep the camera resolution/crop and lens settings the same as navigation.

The preview can save up to 20 unannotated frames, one every three seconds:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
./launch_preview.sh --capture-calibration --seconds 65
```

The images go to `reports/calibration-frames`. The following is an example for
a board with **9 × 6 internal corners** and **25 mm squares**. Substitute the
measurements of your actual board:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
/home/mdp/dev/SC2079-Group-16/ros2_ws/.pixi/envs/pi/bin/python calibrate_camera.py \
  --images reports/calibration-frames \
  --columns 9 --rows 6 --square-size-m 0.025 \
  --output config/camera_measured.yaml
```

The tool requires at least 12 usable views, basic position/scale diversity, and
reprojection RMS no larger than 1.5 pixels. It refuses to overwrite an existing
output. These checks do not replace real range validation.

In the **new module's** `config/mission.yaml`, change `camera_calibration` to
`camera_measured.yaml`. Measure the actual outer black square; set its width in
metres under `marker_size_m`. Approximately 0.10 m was supplied by the user.
The white paper margin must not be included.

Compare estimated distances against a ruler at several positions and angles.
Record the camera-to-robot rigid transform and marker centre height. Fill in
`base_from_camera_rotation`, `camera_origin_in_base_m`, and
`marker_center_height_m`. Rotation is a row-major 3 × 3 proper rotation mapping
OpenCV optical coordinates (right, down, forward) to robot base coordinates
(forward, left, up). Translation is the camera origin in robot-base metres.
Do not copy the illustrative level-camera rotation unless the actual mount
has been measured to match it.

Set `range_verified` and `mount_verified` only after those checks. A low camera
reprojection error alone does not establish a correct camera mount transform.

## 4. Configure the first physical mission

Only the add-on's files need editing. Establish all of the following before
enabling motion:

1. Measure the usable rectangular test area in the **actual `/robot_pose/raw`
   odometry frame** and enter `[xmin, ymin, xmax, ymax]` in `motion.bounds_m`.
   It is not a rectangle expressed relative to whichever point the robot happens
   to occupy when started. The code requires the pose frame name `odom`.
2. Confirm the floor within that area is clear except for the target block and
   any explicit `other_obstacles`. The camera does not create a complete map of
   unseen objects. Each additional square obstacle is `{x, y, yaw, size}` in
   metres/radians in that same frame.
3. Verify the 19 × 23 cm chassis footprint, its centre offset relative to the
   robot pose, and actual turning radius. The supplied radius is 0.21 m and
   margin 0.03 m. The 0.30 m view stand-off is measured from the obstacle face
   to the robot pose reference, not automatically to its front bumper.
4. Use a low-speed motion-controller session: the add-on checks that its
   configured `velocity_speed_mps` is at most 0.10 m/s and that its turning
   radius matches this module. It reads those parameters and refuses a mismatch;
   it does not change them. The existing source defaults to 0.15 m/s.
5. Give this mission exclusive control. Existing planner/fastest-car and keyboard
   teleop nodes must not be running. The add-on checks their ordinary node names
   and aborts on Android commands or teleop input. This check cannot discover an
   arbitrary renamed controller or enforce ownership over every possible ROS
   client; the operator must establish the exclusive session.
6. Verify fresh odometry and all three front proximity streams. The add-on
   rejects invalid/stale ranges and uses conservative 0.15 m ultrasonic and
   0.12 m IR stop thresholds. It never clears an existing emergency-stop latch.
7. Set the configuration's verification fields only after the physical work.
   Keep `allow_reverse: false` until rear and side clearance are established;
   the current sensors do not prove rear clearance.

The first version acquires a marker **within the current camera field of view**.
It waits up to 30 seconds while stationary and fails if no marker is found.
It does not blindly drive or pivot to search unknown floor space. Searching
outside the current view needs a separately validated search route or a pan
mechanism; neither is claimed here.

## 5. Motion entry point and behaviour

Check the configuration first; this command cannot move the robot:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
/home/mdp/dev/SC2079-Group-16/ros2_ws/.pixi/envs/pi/bin/python run.py preflight
```

The shipped configuration intentionally returns exit code 2 and lists missing
measurements. Before the first physical run, set `mission_stage: approach` in
the new mission configuration. It will stop after the approach without starting
the orbit or reporting an invented symbol. Next use `one_face` to inspect only
one reachable adjacent face. Use `full` for the complete face search after those
stages pass. After completing the physical setup, the separate motion entry is:

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
./launch_mission.sh --enable-motion
```

**This command enables real movement only when the configuration and live
checks pass. It was not executed as a robot-driving test during implementation.**

The node confirms a stable marker, estimates its pose and sends one short
primitive at a time to `/execute_moves`. Straight primitives are 5 cm; curved
primitives turn 15 degrees at the verified steering radius. A service request
contains exactly one primitive; long batches are never queued. A failed or
overlong primitive aborts the mission. If this process is killed without normal
shutdown, an already accepted service primitive may finish; it cannot rely on
this process's shutdown handler. Keeping each primitive short limits that case,
but physical stopping distance must still be measured.

The planner checks the full chassis along the same discretised straight/curved
primitives it executes. It supports oriented square obstacles and bounded
forward/reverse hybrid A* search. Planning is bounded in time and can decline a
physically possible path if the search limit is reached. It is not guaranteed
to find the shortest route or all reachable routes.

At the initial marker face, it visits other faces in a fixed candidate order
(first adjacent, other adjacent, opposite), skipping unreachable viewpoints.
It recognises symbols only while stopped, within the projected face region,
using three distinct post-arrival frames. Old face/mission results are rejected.
Three frames containing a marker never complete the mission as symbol 40.

The default symbol model is the previously verified nano candidate. Its exact
checksum is required before loading. To compare medium while stationary, change
`recognition.model` to the verified medium candidate path, set its SHA-256 to
`37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031`, and validate
the longer face timeout. Previous medium inference was roughly 4–5 seconds per
frame on the Pi. The new geometric marker detector does not need that model
for guidance.

Results are published once to **`/bullseye/target`**, for example `1,25` for F.
This is a dedicated result topic, not the existing Android mission topic.
Mission integration can later select an output topic in the new configuration;
it must preserve exclusive control and obstacle identity. Repeating identical
bullseyes does not encode an obstacle ID; ID 1 is assigned for this one-obstacle
demonstration.

Ctrl+C requests a stop if the module has armed. Missing telemetry, invalid
range, proximity, conflicting manual commands and loss of the approach marker
also stop it. During an orbit, the marker is expected to leave view, so odometry
and the saved obstacle geometry guide that segment. Inspecting all reachable
faces without confirmation ends in failure rather than another endless loop.

## 6. Verification commands and limits

Pure tests (no ROS, serial or motor access):

```bash
cd /home/mdp/mdp-cv/bullseye-navigation
/home/mdp/dev/SC2079-Group-16/ros2_ws/.pixi/envs/pi/bin/python \
  -m unittest discover -s tests -v
```

ROS preview replay (requires the existing ROS router; publishes only a saved
reference and blank images to `/bullseye/replay/image`):

```bash
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
pixi run --frozen -e pi python \
  /home/mdp/mdp-cv/bullseye-navigation/tests/ros_replay.py
```

Do not run this replay at the same time as another instance of the add-on.
The test checks that preview has no ROS service clients or control publishers.
Unit tests exercise marker rejection, perspective/blur, calibration geometry,
full-body collisions, reverse-turn kinematics, stale/duplicate/old-face results,
marker/symbol-40 separation, failed motion and a simulated complete mission.

Saved reference tests, synthetic geometry and replay do not validate camera
range, physical steering, odometry accuracy, or clearance on the actual floor.
The next physical stages are preview/range verification, approach-only, a
single adjacent-face move, then the full search. Keep each stage's evidence
separate in `reports/`.
