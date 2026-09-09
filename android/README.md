# Android Remote Controller

Java Android app for manual robot control and status display, connected over
classic Bluetooth SPP to the Raspberry Pi's `android_bridge_node`
(`ros2_ws/src/mdp_android_bridge/`). Speaks the wire format in
[`docs/protocol.md`](../docs/protocol.md) — `FW:<mm>`/`BW:<mm>`/`TL:<deg>`/
`TR:<deg>`/`STP` out, `STATUS,<text>`/`DONE` in.

- Language: Java (no Kotlin, no external libraries — plain Android
  framework APIs only, to keep the build self-contained).
- `minSdk` 26 (Android 8.0), `targetSdk`/`compileSdk` 34.
- Package: `com.sc2079.group16.controller`.

## How it works

- `MainActivity` — single screen: a list of already-**paired** Bluetooth
  devices to connect to, then a control panel with a **3-tab mode switcher**:
  - **Manual Tab** (`[Manual]`): displays `JoystickView` for continuous, low-latency manual driving (Checklist C.2–C.10).
  - **Task 1 Tab** (`[Task 1]`): displays mission summary and green `START TASK 1 (EXPLORE)` button (sends `START` to activate autonomous TSP exploration).
  - **Task 2 Tab** (`[Task 2]`): displays sprint summary and blue `START TASK 2 (FASTEST CAR)` button (sends `START_TASK2` to trigger reactive slalom sprint & carpark return).
  - **Global Controls**: persistent `STOP` button (sends `STP` E-STOP) and `Reset` button (sends `RESET`), accessible in all modes.
  - **Status Log**: scrolling display for curated `STATUS: ...`, `DONE`, live `🎯 TARGET: ...` recognitions, and `📍 POSE: ...` telemetry.
- `BluetoothLinkService` — owns the `BluetoothSocket` connect + read/write
  loop on a background thread, decoupled from the UI thread. Connects with
  `createInsecureRfcommSocketToServiceRecord()` against the well-known SPP
  UUID (`00001101-0000-1000-8000-00805F9B34FB`) — this matches the SDP
  record the Pi registers in `ros2_ws/bluetooth-setup/
  bring-up-and-register.sh`. Insecure rather than secure: the link is
  already authenticated by the one-time pairing bond (see below), and
  Android's secure-socket handshake against a plain BlueZ SPP service is a
  common source of spurious connection failures that the insecure variant
  avoids.

**The app never scans for or discovers devices** — it only lists devices
already paired at the OS level (`BluetoothAdapter.getBondedDevices()`).
Pairing with the Pi is a one-time manual step done outside the app; see
[`../ros2_ws/bluetooth-setup/`](../ros2_ws/bluetooth-setup/)
(`pair-agent.sh` on the Pi side, standard Android Bluetooth settings on the
tablet side). This also means the app needs no location permission — only
`BLUETOOTH_CONNECT` (Android 12+) / `BLUETOOTH`+`BLUETOOTH_ADMIN` (older).

## Building — no Android Studio required

This is a plain Gradle project; everything below runs from a terminal.
(Android Studio can still open this folder directly if you prefer its UI —
nothing about the project structure requires it.)

```bash
cd android
./gradlew :app:assembleDebug
```

Output APK: `app/build/outputs/apk/debug/app-debug.apk`.

`local.properties` (gitignored, machine-specific) must point `sdk.dir` at
your Android SDK — e.g. `sdk.dir=/Users/<you>/Library/Android/sdk` — or set
the `ANDROID_HOME` environment variable instead.

## Installing on your tablet over USB

USB here is just how you get the APK onto the tablet and see logs — the
Bluetooth link to the Pi at runtime is still wireless, over the tablet's own
Bluetooth radio, not through the USB cable.

1. On the tablet: Settings → About tablet → tap "Build number" 7 times to
   unlock Developer Options, then Settings → Developer Options → enable
   "USB debugging".
2. Connect the tablet to your computer via USB. Accept the "Allow USB
   debugging?" prompt that appears on the tablet.
3. Confirm it's visible: `~/Library/Android/sdk/platform-tools/adb devices`
   should list it (not `unauthorized` — if it shows that, check the tablet
   screen for the prompt again).
4. Install: `~/Library/Android/sdk/platform-tools/adb install -r
   app/build/outputs/apk/debug/app-debug.apk`.
5. Pair the tablet with the Pi once via the tablet's normal Bluetooth
   settings (Settings → Connected devices → Pair new device), after running
   `pair-agent.sh` on the Pi side so it's discoverable. Then open the app —
   the Pi should appear in the "Paired devices" list.
6. `adb logcat` if you need to debug — there's no Android Studio Logcat
   panel in this workflow, but it's the same log stream.
