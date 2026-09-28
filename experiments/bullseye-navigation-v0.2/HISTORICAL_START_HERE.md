> Historical September 2026 experiment. Not the active repository navigation stack. Read [README.md](README.md) before using these instructions. Physical navigation was not validated.

# Start the bullseye task from one Pi terminal

Run this from **any directory on the Raspberry Pi**:

```bash
/home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh
```

This is the complete-task launcher. It activates the existing Pi ROS environment,
checks the installed files and the pinned ONNX model, checks measured camera and
floor geometry, starts a router if required, starts or reuses one camera, starts
its own STM32 bridge and low-speed controller, and runs the bullseye mission.
It shows state changes and the final symbol in the same terminal. Keep that
terminal open. **Ctrl+C stops the task and closes the components it started.**
It leaves a pre-existing router or camera running.

The existing project source, model selection and launch files are unchanged.
The launcher does not start Android, the existing planner, or Task 2. It refuses
to take over an existing hardware/control session; stop that session in its own
terminal before starting this one. It never automatically clears an emergency
stop. A latched hardware stop must be resolved by the operator before a new run.

## Current readiness

The software can be installed and tested using saved images. **The full driving
task cannot run yet with the shipped configuration.** The camera intrinsics,
measured camera mounting transform, range validation, chassis verification and
clear test-area bounds have not been supplied. The default command prints the
missing items and exits **before starting the camera, serial bridge or controller**.
Do not replace missing measurements with guessed values or mark unchecked items
as verified just to make startup pass.

Read the readiness report without starting any robot component:

```bash
/home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh --check
```

Exit code 2 means setup is incomplete. It does not mean the transfer failed.

## Start a camera-only preview now

This single command starts the required camera/router as needed and the marker
preview. It creates no motor link or motion controller:

```bash
/home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh --preview
```

On the Pi desktop, add `--display` for a live image window:

```bash
/home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh --preview --display
```

The marker must be in the camera's current view. The robot does not drive around
blindly to find a marker outside that view. A successful preview outlines the
marker; distance is unavailable until calibration is complete.

## Complete setup once, then keep using the same command

1. Complete the camera checkerboard calibration and ruler checks in `HISTORICAL_GUIDE.md`,
   sections 3 and 4. Record the actual outer black square width (approximately
   0.10 m), camera mounting transform and marker centre height in the add-on's
   camera configuration.
2. Record verified robot dimensions, turning radius, floor bounds and any other
   obstacles in `/home/mdp/mdp-cv/bullseye-navigation/config/mission.yaml`. Bounds
   must use the actual `/robot_pose/raw` odometry coordinates. The existing serial
   bridge has its own initial pose; do not assume that startup means `(0, 0, 0)`.
3. Establish a clear test area and exclusive control, with an operator present
   and a physical way to stop the robot. Set the verification fields only after
   those checks. Leave reverse disabled until rear clearance is verified.
4. Run `start_bullseye.sh --check` again. File/model checks must pass, all missing
   configuration entries must be resolved and the STM32 port must be present.
5. Commission the physical movements in order. These are each still one command:

   ```bash
   /home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh --stage approach
   ```

   ```bash
   /home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh --stage one_face
   ```

   After those pass, use the ordinary complete-task command:

   ```bash
   /home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh
   ```

Stage overrides affect only that invocation. The supplied configuration's normal
stage is `full`. The launcher applies at most 0.10 m/s and the configured turning
radius to the controller and serial bridge **for this session only**; it does not
edit their existing defaults. Proximity stopping stays enabled. Live odometry,
range, service and competing-controller checks still have to pass before motion.

## Files and results

| Item | Location on the Pi |
|---|---|
| One-command launcher | `/home/mdp/mdp-cv/bullseye-navigation/start_bullseye.sh` |
| Startup supervisor | `/home/mdp/mdp-cv/bullseye-navigation/start_task.py` |
| Mission settings | `/home/mdp/mdp-cv/bullseye-navigation/config/mission.yaml` |
| Camera measurements | `/home/mdp/mdp-cv/bullseye-navigation/config/camera_calibration.yaml` |
| Latest annotated image | `/home/mdp/mdp-cv/bullseye-navigation/reports/latest.jpg` |
| Latest mission status | `/home/mdp/mdp-cv/bullseye-navigation/reports/status.json` |
| Readiness report | `/home/mdp/mdp-cv/bullseye-navigation/reports/readiness.json` |
| Latest session directory | Path written in `/home/mdp/mdp-cv/bullseye-navigation/reports/latest-session.txt` |

Each session has individual component logs and a `result.json`. Successful
recognition is also published once to `/bullseye/target`, for example `1,25` for
obstacle 1 with F. This does not automatically send a result to Android.

The existing verified nano model is referenced in place at:

```text
/home/mdp/mdp-cv/candidates/baseline-yolov8n-v61-full-20260908T142129Z-r2/best.onnx
```

SHA-256: `0207a7fb6ab117ce0a62fd683185da8d8ca065c92f1a0ee1d155e65f14e2c48a`.
The geometric bullseye detector guides navigation; the nano model reads symbols
while stopped. The medium model remains in its separate candidate directory.

If the STM32 device changes, provide its verified port for that invocation, for
example `start_bullseye.sh --serial-port /dev/ttyACM1`. Do not guess between
multiple serial devices. The launcher never sends a reset to discover a port.

`--seconds 30` bounds a preview/session. For calibration capture use
`start_bullseye.sh --preview --capture-calibration --seconds 65`; see the full
README for checkerboard requirements. Startup and shutdown take additional time.

Saved-image tests and launch supervision checks are not physical driving tests.
No physical approach/orbit performance is claimed by this package.
