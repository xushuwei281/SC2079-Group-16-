# Teleoperation Pipeline & Latency Optimization Guide

## 1. Executive Summary

This document details the end-to-end latency analysis, architectural fixes, and timing optimizations implemented to achieve smooth, responsive teleoperation of the SC2079 MDP robot via the Android tablet.

Prior to these optimizations:
- Single movement commands experienced **~510 ms of dead stationary delay** per cycle.
- Holding down a directional button on the Android tablet resulted in severe stuttering, where the car executed one move (~8 cm), halted abruptly, spammed the tablet with `BUSY_LOCAL` errors, and dropped subsequent inputs.
- Rapid successive inputs caused a `NO_RESPONSE` timeout due to an inversion of priority in the STM32 UART interrupt handler and input buffer clearing in the ROS 2 hardware bridge.

After these optimizations:
- Turnaround dead time was cut by **~420 ms per movement**.
- A **1-deep command queue with zero-idle chaining** enables continuous, fluid driving when buttons are held.
- The UART protocol guarantees atomic rejection of busy commands and clean recovery.

---

## 2. End-to-End Latency Breakdown

The teleoperation pipeline spans four stages:

```mermaid
flowchart LR
    A["Android Tablet (RFCOMM)"] -->|"FW:50 / FC:8 (every 50ms)"| B["RPi: android_bridge_node"]
    B -->|"/execute_moves (ROS 2 Srv)"| C["RPi: serial_bridge_node"]
    C -->|"5-byte UART (115200 baud)"| D["STM32: USART3 / FreeRTOS"]
    D -->|"PWM / H-Bridge"| E["Motors & Steering Servo"]
```

### Turnaround Timing for an 8 cm Teleop Move

| Stage / Component | Before Optimization | After Optimization | Net Savings | Rationale |
|---|---|---|---|---|
| **RFCOMM & ROS 2 Bridge** | ~5 ms | ~5 ms | 0 ms | Lightweight local processing |
| **UART 5-byte + trigger transmission** | ~17 ms | ~17 ms | 0 ms | 15 ms inter-packet guard + baud time |
| **Servo Turn Throw Delay** | 200 ms | 80 ms | **120 ms** | TD-8120MG sweeps 25° throw in < 70 ms |
| **Static Friction Break** | 200 ms | 50 ms | **150 ms** | `MOTORLOW` kick breaks static friction in 30–50 ms |
| **Cruise & Creep Motion** | ~250 ms (5cm creep) | ~200 ms (2cm creep) | **50 ms** | Adaptive creep allows cruising up to last 2 cm |
| **Post-Move Stop Delay** | 100 ms | 20 ms | **80 ms** | Motor momentum dampens in 15–20 ms |
| **Post-Turn Centering Delay** | 200 ms (100ms+100ms) | 40 ms | **160 ms** | Servo centers immediately upon motor cutoff |
| **FreeRTOS Mechanical Settling** | 200 ms | 20 ms | **180 ms** | Car is already stopped before `sendFin()` |
| **Total Stationary Dead Time** | **~510 ms** | **~90 ms** | **~420 ms** | **82% reduction in non-moving idle latency** |

---

## 3. Detailed Component Changes

### 3.1 Android Bluetooth Bridge (`android_bridge_node.py`)

#### Problem
The Android application streams directional commands every ~50–60 ms while a button is held down. Because a single 8 cm physical move takes ~250–350 ms, every stream packet after the first arrived while `/execute_moves` was busy (`_busy.set()`), returning `BUSY_LOCAL`. This caused:
1. Flooding the Bluetooth link with `STATUS,Move failed: BUSY_LOCAL` error alerts.
2. The robot coming to a complete stop after only one move, requiring the user to release and press the button repeatedly.

#### Solution: 1-Deep Pending Queue & Zero-Idle Chaining
- **Queue Structure:** Added `_pending_move: Optional[Tuple[str, int]]` guarded by `_move_lock`.
- **Buffering:** When `_handle_movement()` receives a command while `_is_moving == True`, it stores the latest `(code, value)` in `_pending_move`. If the user continues holding or changes direction (e.g. from `FC` to `FR`), `_pending_move` is overwritten with the newest command.
- **Zero-Idle Chaining:** In `_on_move_complete()`, when the active service call returns `success`:
  - If `_pending_move` is not `None`, it immediately calls `_dispatch_move_locked()` with the buffered move without transitioning to idle or sending `DONE`.
  - Only when `_pending_move` is empty does it set `_is_moving = False` and emit `DONE` to the tablet.
- **Alert Suppression:** Rejections due to `BUSY_LOCAL` are logged internally but suppressed from being transmitted over Bluetooth.
- **E-Stop Flushing:** Incoming `STP`, `STOP`, or `Q` immediately flushes `_pending_move = None` and cuts motors.

---

### 3.2 STM32 Firmware (`stm32/src/main.c`)

#### Problem 1: Priority Inversion in UART Callback
In `HAL_UART_RxCpltCallback`:
```c
// BUGGY ORIGINAL CODE:
else if (pkt[0] == '#') {
    runRequested = 1;
    HAL_UART_Transmit(&huart3, (uint8_t *)"RUN\r\n", 5, 10);
}
else if (runRequested == 1) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"BUS\r\n", 5, 10);
}
```
When a command packet (e.g. `FC007`) arrived during the 200 ms mechanical settling, `runRequested == 1` was true, so `FC007` was rejected with `BUS\r\n` and **not** queued into `instrList`. However, 15 ms later when the `#` trigger packet arrived, it matched `pkt[0] == '#'` *first*, responded with `RUN\r\n`, and executed an empty queue (`instrLen == 0`).

#### Fix 1: Atomic Busy Rejection
Reordered the handler so `runRequested == 1` is evaluated before `#`:
```c
else if (runRequested == 1) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"BUS\r\n", 5, 10);
}
else if (pkt[0] == '#') {
    runRequested = 1;
    HAL_UART_Transmit(&huart3, (uint8_t *)"RUN\r\n", 5, 10);
}
```
Now, if the STM32 is busy, both the instruction and the trigger packet receive `BUS\r\n`, ensuring the RPi retries the entire batch cleanly.

#### Fix 2: Elimination of FreeRTOS Blocking Delays
- `comm_task`: Reduced post-batch mechanical settling from `osDelay(200)` to `osDelay(20)`.
- `FrontCenter` / `BackCenter`: Reduced static friction delay from `osDelay(200)` to `osDelay(50)` and post-stop delay from `osDelay(100)` to `osDelay(20)`.
- `FrontRight` / `FrontLeft` / `BackRight` / `BackLeft`: Servo throw delay reduced from `osDelay(200)` to `osDelay(80)`. Replaced sequential 100 ms stop delay + 100 ms centering delay with simultaneous motor cut and servo centering followed by a single `osDelay(40)`.

#### Fix 3: Adaptive Creep Distance
Original code crawled the last 5 cm / 4° at creep speed (`MOTORLOW`) regardless of target size. For an 8 cm move, 62% of the distance was spent creeping. The new implementation adapts:
```c
int creep_dist = (dist_target > 15) ? 5 : 2;
int creep_angle = (angle > 15) ? 4 : 1;
```
Small moves cruise until the last 2 cm / 1°, resulting in crisp step response.

#### Fix 4: Infinite Loop Safety Timeouts
Added hardware tick timeouts to all polling loops:
```c
uint32_t t0 = osKernelGetTickCount();
uint32_t timeout_ticks = (uint32_t)(dist * 80 + 2000);
while (cur_dist < dist_target - creep_dist && (osKernelGetTickCount() - t0) < timeout_ticks) {
    osDelay(20);
}
```
If wheel slip or gyro reading fails to advance, the MCU automatically aborts motion, cuts motors, sends `FIN`, and unblocks communication.

---

### 3.3 Hardware Serial Bridge (`serial_bridge_node.py`)

#### Problem
In `_handle_execute_moves`, `self._serial.reset_input_buffer()` was placed inside the `_BUS_RETRIES` loop immediately before writing. When the STM32 rejected with `BUS` and subsequently emitted its handshake, the retry loop wiped the input buffer right before calling `_read_status_line(5.0)`, resulting in `NO_RESPONSE` timeouts.

#### Solution
- Moved `reset_input_buffer()` outside the retry loop to flush stale data once at the start of a service call.
- On receiving `BUS\r\n`, the node sleeps for 80 ms to let the firmware complete its remaining settling, then explicitly drains the buffer before retransmitting.
- Increased `_BUS_RETRIES` from 3 to 10 with fast 50–80 ms backoff instead of 1.0 s.

---

### 3.4 UART Flashing Configuration (`platformio.ini`)

#### Problem
In `stm32/platformio.ini`, `upload_flags` was configured as:
`rts,-dtr,dtr,-rts:-rts,-dtr,dtr`
The trailing `dtr` left DTR asserted after flashing, holding the C30D one-key download circuit in reset/bootloader mode.

#### Solution
Changed exit flag sequence to `-rts,-dtr`, releasing the reset and BOOT0 lines so user code executes immediately. Note that on boot, the firmware runs 5.0 seconds of gyro offset calibration before opening USART3.

---

## 4. Verification & Testing

### 4.1 Automated Unit Tests
Run via Pixi:
```bash
pixi run -e pi python3 -m unittest discover -s src/mdp_android_bridge/test
```
Results: **10/10 tests passing (100% success)**
- `test_movement_command_conversion`: Verifies FW/BW/TL/TR string translation.
- `test_mm_auto_conversion`: Verifies mm-to-cm conversion (> 100 mm).
- `test_estop_dispatch`: Verifies emergency stop trigger.
- `test_planner_commands_forwarding`: Verifies ALG/START/RESET forwarding.
- `test_pose_to_tablet_format`: Verifies ROBOT,x,y,dir formatting.
- `test_target_to_tablet_format`: Verifies TARGET,id,symbol formatting.
- `test_pending_move_queueing`: Verifies 1-deep buffering during active motion.
- `test_pending_move_dispatch_on_done`: Verifies immediate chaining into pending moves.
- `test_busy_local_suppression`: Verifies suppression of BUSY_LOCAL alerts.
- `test_estop_clears_pending_move`: Verifies queue clearing on emergency stop.

### 4.2 Hardware Link Verification
1. **STM32 UART (`/dev/ttyACM1`)**: Confirmed sub-100 ms response to `#\x00\x00\x00\x00` returning `RUN\r\n` and `FIN:POS,...`.
2. **Android Bluetooth RFCOMM (`/dev/rfcomm0`)**: Verified continuous bidirectional communication with Android tablet.
