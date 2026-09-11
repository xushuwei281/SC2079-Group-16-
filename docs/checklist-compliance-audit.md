# SC2079 Checklist Compliance Audit

**Audit date:** 2026-09-11
**Scope:** Official SC2079 technical materials, checklist items A.1–A.5, B.1–B.3 and C.1–C.10, plus the current repository implementation.
**Method:** Source inspection and offline automated tests. Physical demonstrations, calibration measurements and live evaluator runs were not available.

This is the current engineering status. It supersedes any older report that claims full compliance without distinguishing source evidence from physical verification.

## Executive status

The project is **not yet checklist-complete**. The largest known gaps are Android Bluetooth discovery/reconnection, immediate transmission of arena edits, target-label rendering, and missing WiFi/AP evidence.

| Area | Status | Summary |
|---|---|---|
| A — hardware and robot functions | Partial | Perception and serial interfaces exist; network setup and physical accuracy are not demonstrated. |
| B — simulator and planning | Partial | Simulator and distance-based TSP exist; shortest-time optimisation is not demonstrated. |
| C — Android/checklist GUI | Partial | Core teleoperation/map code exists, but C.2 and C.6–C.9 have concrete divergences. |
| Automated tests | Passing | 94 Python tests passed in the aggregate Pixi suite. Firmware host/build results are tracked separately. |

## Checklist matrix

| Item | Status | Evidence and divergence |
|---|---|---|
| A.1 | **Partial / unverified** | Bluetooth RFCOMM and USB-ACM STM links are implemented. No repository-managed WiFi AP, static-IP or example webserver setup was found. Simultaneous physical operation is unverified. |
| A.2 | **Implemented; live verification required** | `mdp_perception` subscribes to camera frames, runs ONNX detection and publishes annotated images; the detector draws boxes and labels. Detection accuracy at 20–50 cm and evaluator display are unverified. |
| A.3 | **Unverified** | Straight movement commands and tests exist, but no measured 80–120 cm run proving ±6% accuracy is recorded. |
| A.4 | **Unverified** | Turn commands exist, but no measured 90–360° physical result is recorded. |
| A.5 | **Partial** | Planner/perception/orbit-recovery code supports obstacle approach and face sampling. A live search demonstration and missing-obstacle recovery are not evidenced. |
| B.1 | **Implemented** | Pygame simulator includes a 2 m × 2 m arena, start zone, obstacles, robot pose and movement. |
| B.2 | **Implemented in algorithm code** | Planner enumerates tours and tests cover route generation; five-image timed execution is not physically demonstrated. |
| B.3 | **Partial** | Planner minimises Reeds–Shepp path distance. It does not model measured manoeuvre time, acceleration or velocity, so “shortest-time” compliance is unproven. |
| C.1 | **Implemented** | Android/Pi RFCOMM send and receive paths exist. |
| C.2 | **Fail** | Android lists bonded devices only; it does not perform Bluetooth discovery/scanning. See `BluetoothLinkService.java` and `MainActivity.java`. |
| C.3 | **Implemented** | Joystick controls generate movement commands; no manual string-entry workflow is required. |
| C.4 | **Partial** | High-rate telemetry is filtered, but other incoming lines are broadly surfaced in debug output rather than strictly whitelisted as curated status. |
| C.5 | **Implemented** | Arena view renders a 20 × 20 map, numbered obstacles and cardinal robot orientation. |
| C.6 | **Fail / partial** | Dragging and outside-map removal work, but edits are not transmitted on finger-up. Transmission requires a separate “Send Arena” action. |
| C.7 | **Fail / partial** | Face annotation changes local state and appearance, but face/coordinate data is not transmitted immediately on annotation. |
| C.8 | **Fail** | Android disconnect handling returns to the picker; no automatic reconnect loop is present. (The Pi-side RFCOMM worker does retry.) |
| C.9 | **Fail / partial** | Target symbols reach Android and a face marker can be drawn, but target text uses the normal small paint and embeds `\\n` in one `drawText()` call, which does not create a second line or guarantee the required large white display. |
| C.10 | **Partial / protocol risk** | `ROBOT` updates are generated and parsed. The bridge defaults to literal angle brackets and rounds metric coordinates to cells; confirm the evaluator expects that exact wire form and boundary convention. |

## Concrete implementation gaps

1. Add Android Bluetooth discovery (`BLUETOOTH_SCAN`, discovery callbacks and cancellation) to satisfy C.2.
2. Send `ADD`, `SUB` and `FACE` (or the agreed protocol equivalent) from `ArenaView`/`MainActivity` immediately when a drag or face annotation ends.
3. Add Android automatic reconnect with bounded backoff and a clear disconnected state for C.8.
4. Render the target ID as a dedicated large white text element and draw the target-face line independently for C.9.
5. Confirm C.10 wire formatting with the evaluator; remove literal placeholder brackets if they are not part of the required protocol, and document the cell-boundary rule.
6. Document or implement the RPi WiFi AP/static-IP/webserver setup required by A.1.
7. Add a time/velocity objective or measured timing evidence for B.3.
8. Record physical stopping-distance and motion-accuracy measurements before evaluation.
9. Verify clean launch shutdown while a movement is active; the motion controller now
   has an explicit cancellation path for this case.

## Automated test record

Command used:

```bash
PATH=/home/mdp/.pixi/bin:$PATH \
  /home/mdp/.pixi/bin/pixi run -e pi test-all
```

Result:

- Algorithm suite: **13 passed**.
- Hardware/motion-controller suite: **45 passed**.
- Android bridge suite: **16 passed**.
- Planner suite: **20 passed**.

## Evidence locations

- Android Bluetooth discovery/reconnect: `android/app/src/main/java/com/sc2079/group16/controller/BluetoothLinkService.java`, `MainActivity.java`
- Arena interaction and target rendering: `android/app/src/main/java/com/sc2079/group16/controller/ArenaView.java`
- Pi pose/status protocol: `ros2_ws/src/mdp_android_bridge/mdp_android_bridge/android_bridge_node.py`
- Perception boxes/labels: `ros2_ws/src/mdp_perception/mdp_perception/detector.py`
- STM32 hardware loop and stack configuration: `stm32/src/main.c`
- Simulator and route planner: `algorithm/simulator.py`, `algorithm/planner.py`

This document is an engineering audit, not proof of competition performance. Update statuses only when source changes or repeatable test/demo evidence changes.
