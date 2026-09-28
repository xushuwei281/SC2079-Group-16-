# Bullseye navigation: implementation versus demonstrated results

The CV history contains an early standalone bullseye navigation experiment.
The current repository also contains newer integrated navigation code. These are
different implementations and evidence sets. Neither source availability nor a
passing local test proves completion of a physical approach-and-orbit trial.

## 1. Historical standalone experiment, through 18 September

The requested task was to find an obstacle with the camera, approach its bullseye
face and move around it until a valid symbol was found. The user estimated both
the obstacle footprint and the marker's outer black square as 10 cm × 10 cm.
The initial constraint was to keep the existing robot application unchanged, so
the feature was developed as a separate add-on.

The historical source is preserved under
[experiments/bullseye-navigation-v0.2](../../experiments/bullseye-navigation-v0.2/README.md).
Treat its launch scripts, parameters and tests as that experiment's reproducible
snapshot, not as the preferred entry point to the latest integrated stack.

Its design used nested-square image geometry, known marker width and calibrated
camera geometry to estimate the marker pose. It used bounded path search and
collision checks for an Ackermann vehicle, together with pose freshness,
confirmation, motion limits and range-feedback gates. Approximate card dimensions
were not a substitute for measured camera intrinsics/extrinsics and chassis
calibration.

Recorded local validation included:

- A localhost ROS preview replay of 79 frames, without movement interfaces.
- Offline classification of saved Pi scenes as digit 1, F or no target.
- A synthetic mission visiting faces 0, 1, 3 and 2 with 816 collision-checked
  trajectory samples and F accepted on the opposite face.
- 29 final local tests: 23 core tests and six startup tests.
- A v0.2 archive of 50,698 bytes, 25 verified source files, SHA-256
  `60304179bd37ffc7ffa3e5124696b22a234a5b8752af8cc7ac28a25affbe7364`.

The synthetic mission used known poses, a bounded 2 m × 2 m scene and reverse
enabled. It was not visual navigation by a real robot in an unknown room.
An earlier partial Pi bundle had 18/20 tests passing; local planning changes
followed, but the final v0.2 installation and retest were not verified. The
[package receipt](evidence/2026-09-18/bullseye-package-readiness.json) explicitly
records `pi_installation_verified: false`.

Transfers were interrupted while Pi connectivity/identity changed. A later login
or terminal support task does not establish that the previous application paths
and configuration were restored. No physical bullseye approach or orbit is
demonstrated by the historical CV evidence.

## 2. Current integrated repository code

The following sources were inspected against the starting `main` revision
`a8b69b5` for this documentation update:

| Source | Implemented role |
|---|---|
| [bullseye_orbit_node.py](../../ros2_ws/src/mdp_bringup/mdp_bringup/bullseye_orbit_node.py) | Closed-loop approach/orbit service `/bullseye/orbit_approach`; live tracking, ultrasonic feedback and sample-target confirmation |
| [bullseye_control.py](../../ros2_ws/src/mdp_bringup/mdp_bringup/bullseye_control.py) | Pure control-law calculations used by the integrated orbit node |
| [perception_node.py](../../ros2_ws/src/mdp_perception/mdp_perception/perception_node.py) | Gated continuous `/perception/tracked_target` output as well as service-based sampling |
| [detector.py](../../ros2_ws/src/mdp_perception/mdp_perception/detector.py) | Canonical numeric label support and marker sentinel 0, distinct from circle/Stop ID 40 |
| [robot.launch.py](../../ros2_ws/src/mdp_bringup/launch/robot.launch.py) | Integrated application launch composition, including the orbit node |
| [checklist_a5.py](../../ros2_ws/src/mdp_bringup/mdp_bringup/checklist_a5.py) | Separate ultrasonic approach and face-recognition routine |
| [arena.py](../../algorithm/arena.py) | Current shared vehicle/planning geometry |

The latest integrated perception and orbit source accepts real symbol IDs 11–40
and separates the marker. At the inspected revision, the older `checklist_a5.py`
still accepts only 11–39 and treats 40 as a marker. Older handbook/FSM prose also
contains that convention. This is a compatibility boundary requiring attention
when selecting a launch path; the historical baseline evidence does not silently
repair every old mission consumer.

Current `algorithm/arena.py` sets the turning radius to **25 cm**. The standalone
experiment used **21 cm** and different collision margins. Do not import its
historical geometry into the integrated controller or claim its simulated
clearance proves the current robot's clearance.

The repository's [checklist compliance audit](../checklist-compliance-audit.md)
labels A.5 partial and says a live search demonstration and missing-obstacle
recovery are not evidenced. The new source code is an implementation milestone;
the retained September camera tests establish only the narrower outcomes in
[CV results](results.md).

## 3. Requirements for a physical demonstration

Before presenting the feature as physically complete, retain evidence for:

1. Measured marker dimensions, camera calibration and correct pose/distance
   estimates on saved and live images.
2. A stationary live preview with correct marker/official-symbol separation and
   fresh tracking signals.
3. A controlled approach with measured stopping distance and verified safety gates.
4. A controlled single-face transition with chassis and obstacle clearance checked
   against the current geometry.
5. A full approach-and-search trial with confirmed target ID, obstacle/face
   association and recorded failure behavior when no valid image exists.

Use the current application's documented operational path and one hardware owner
per camera/UART/Bluetooth device. Under the repository's
[agent rules](../../AGENTS.md), long-running ROS launches are started manually by
the operator. This documentation update does not start them or enable movement.

These are remaining validation steps. They are not retrospective claims that
those physical experiments were performed.
