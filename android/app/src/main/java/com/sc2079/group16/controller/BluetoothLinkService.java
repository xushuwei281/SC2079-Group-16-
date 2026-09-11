package com.sc2079.group16.controller;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Owns the classic Bluetooth SPP link to the Raspberry Pi's
 * android_bridge_node (ros2_ws/src/mdp_android_bridge/).
 * Speaks the ASCII line protocol from docs/protocol.md: continuous
 * VEL:&lt;m/s&gt;,&lt;rad/s&gt;, legacy FW/BW/TL/TR, and STP out; STATUS,&lt;text&gt;,
 * DONE, ROBOT,&lt;x&gt;,&lt;y&gt;,&lt;dir&gt; (Checklist C.10, coarse) and
 * POSE,&lt;x_cm&gt;,&lt;y_cm&gt;,&lt;yaw_deg&gt; (supplemental, full precision) in.
 *
 * Connects to an already-paired (bonded) device only -- pairing itself is a
 * one-time manual step done outside the app (see
 * ros2_ws/bluetooth-setup/pair-agent.sh), so this never scans
 * or requests discovery, only enumerates BluetoothAdapter.getBondedDevices().
 *
 * Uses an insecure RFCOMM socket rather than the secure variant: the link
 * is already authenticated by the one-time pairing bond, and Android's
 * secure-socket SDP handshake against a plain BlueZ-registered SPP service
 * (ros2_ws/bluetooth-setup/bring-up-and-register.sh) is a
 * commonly reported source of connection failures that the insecure
 * variant avoids.
 */
class BluetoothLinkService {

    private static final String TAG = "BluetoothLinkService";

    /** Well-known Serial Port Profile UUID -- matches the SDP record
     * bring-up-and-register.sh registers on the Pi ({@code sdptool add
     * --channel=1 SP}). */
    private static final UUID SPP_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    interface Listener {
        void onConnected(BluetoothDevice device);

        void onStatusLine(String text); // curated STATUS,<text> payload

        void onDone(); // DONE

        void onDisconnected(String reason);

        // Raw traffic in both directions, for on-device debugging -- every
        // line sent and every line received, regardless of whether it's
        // part of the curated protocol or not. Lets you see what's actually
        // crossing the link without SSHing into the Pi.
        void onDebug(String line);

        // ROBOT,<x>,<y>,<dir> -- live dead-reckoned pose (Checklist C.10
        // format: x/y are grid cells 0-19, dir is N/E/S/W), published by
        // android_bridge_node's /robot_pose subscription after each
        // completed move. Delivered already converted to cm + degrees.
        void onRobotPose(float xCm, float yCm, float headingDeg);

        // TARGET,<obstacle_id>,<symbol_id> -- a perception result relayed
        // from android_bridge_node's /android/target subscription.
        void onTarget(int obstacleId, int symbolId);

        // T1_STATE,<state>,<current_leg>,<total_legs>,<obs_id>,<face>,<rem_dist_cm>
        void onT1State(String state, int currentLeg, int totalLegs, int obsId, String face, float remDistCm);

        // T1_TARGET,<obs_id>,<symbol_id>,<symbol_name>,<confidence>,<face>
        void onT1Target(int obsId, int symbolId, String symbolName, float confidence, String face);

        // T2_STATE,<state>,<step_idx>,<step_desc>,<elapsed_sec>
        void onT2State(String state, int stepIdx, String stepDesc, float elapsedSec);

        // T2_ARROW,<obs_num>,<LEFT|RIGHT>,<symbol_id>,<confidence>
        void onT2Arrow(int obsNum, String direction, int symbolId, float confidence);

        // SENSORS,<us_cm>,<ir_left_cm>,<ir_right_cm>
        void onSensors(float usCm, float irLeftCm, float irRightCm);

        // ESTOP_ALERT,<sensor>,<dist_cm>,<threshold_cm>[,<reason>]
        void onEstopAlert(String sensor, float distanceCm, float thresholdCm, String reason);
    }

    private final Listener listener;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService ioExecutor = Executors.newSingleThreadExecutor();

    private volatile BluetoothSocket socket;
    private volatile OutputStream outputStream;
    private volatile boolean stopping;

    BluetoothLinkService(Listener listener) {
        this.listener = listener;
    }

    /** Connects on a background thread; safe to call from the UI thread. */
    void connect(BluetoothDevice device) {
        stopping = false;
        ioExecutor.execute(() -> runConnection(device));
    }

    /** Queues a line for writing; safe to call from the UI thread. Silently
     * dropped if nothing is connected right now. */
    void sendLine(String line) {
        mainHandler.post(() -> listener.onDebug("… queuing " + line)); // proves sendLine() itself was invoked
        ioExecutor.execute(() -> {
          try {
            OutputStream out = outputStream;
            if (out == null) {
                mainHandler.post(() -> listener.onDebug("→ DROPPED (" + line + "): not connected"));
                return;
            }
            try {
                out.write((line + "\n").getBytes(StandardCharsets.US_ASCII));
                out.flush();
                mainHandler.post(() -> listener.onDebug("→ " + line));
            } catch (IOException e) {
                // Don't rely on the read loop to notice this: it's blocked
                // on a blocking read() and may not see a write-side failure
                // for a long time (or ever, if the link is half-dead) --
                // that left taps silently vanishing with no feedback at all.
                // Surface it immediately and treat it as a disconnect.
                mainHandler.post(() -> listener.onDebug("→ FAILED (" + line + "): " + e.getMessage()));
                postDisconnected("write failed: " + e.getMessage());
                closeQuietly();
            }
        } catch (Throwable t) {
            // Catch-all: anything unexpected here was previously vanishing
            // with zero feedback. Surface it no matter what it is.
            Log.e(TAG, "sendLine crashed", t);
            mainHandler.post(() -> listener.onDebug("→ CRASHED (" + line + "): " + t));
          }
        });
    }

    void disconnect() {
        stopping = true;
        closeQuietly();
    }

    /** Stops accepting further work and releases the link. Call from
     * Activity#onDestroy. */
    void shutdown() {
        disconnect();
        ioExecutor.shutdownNow();
    }

    @SuppressLint("MissingPermission") // caller only reaches here after the
    // BLUETOOTH_CONNECT runtime permission (API 31+) has been granted --
    // see MainActivity#ensurePermissionThenListDevices.
    private void runConnection(BluetoothDevice device) {
        BluetoothSocket sock;
        try {
            // No cancelDiscovery() call here -- this app never starts
            // discovery in the first place (only connects to already-bonded
            // devices), so there's nothing to cancel.
            sock = device.createInsecureRfcommSocketToServiceRecord(SPP_UUID);
            sock.connect();
        } catch (IOException e) {
            Log.e(TAG, "connect() failed", e);
            postDisconnected("connect failed: " + e.getMessage());
            return;
        }

        socket = sock;
        InputStream in;
        try {
            outputStream = sock.getOutputStream();
            in = sock.getInputStream();
        } catch (IOException e) {
            postDisconnected("stream setup failed: " + e.getMessage());
            closeQuietly();
            return;
        }

        mainHandler.post(() -> listener.onConnected(device));
        // Run on its own thread, not ioExecutor: this loop blocks for the
        // entire life of the connection (in.read() only returns on new
        // data or disconnect), and ioExecutor is also where sendLine()
        // queues writes. Running it there starved every write forever --
        // the executor's one thread never got free to service them.
        new Thread(() -> readLoop(in), "BluetoothReadLoop").start();
    }

    private void readLoop(InputStream in) {
        StringBuilder line = new StringBuilder();
        byte[] buf = new byte[256];
        try {
            int n;
            while ((n = in.read(buf)) != -1) {
                for (int i = 0; i < n; i++) {
                    char c = (char) (buf[i] & 0xFF);
                    if (c == '\n') {
                        dispatchLine(line.toString().trim());
                        line.setLength(0);
                    } else if (c != '\r') {
                        line.append(c);
                    }
                }
            }
            postDisconnected("connection closed by peer");
        } catch (IOException e) {
            if (!stopping) {
                postDisconnected("read error: " + e.getMessage());
            }
        } finally {
            closeQuietly();
        }
    }

    private void dispatchLine(String text) {
        if (text.isEmpty()) {
            return;
        }
        // ROBOT/POSE/SENSORS stream continuously (roughly 20Hz, whether or
        // not the robot is actually moving). Always echo the raw line here --
        // MainActivity decides whether to display it (verboseLogSwitch), so
        // that choice is a runtime UI toggle instead of a fixed source-level
        // drop. Their data still fully reaches the UI via the structured
        // callbacks below (onRobotPose/onSensors/...) regardless.
        mainHandler.post(() -> listener.onDebug("← " + text));
        if (text.equals("DONE")) {
            mainHandler.post(listener::onDone);
        } else if (text.startsWith("STATUS,")) {
            String payload = text.substring("STATUS,".length());
            mainHandler.post(() -> listener.onStatusLine(payload));
        } else if (text.startsWith("ROBOT,")) {
            parseRobotPose(text.substring("ROBOT,".length()));
        } else if (text.startsWith("POSE,")) {
            parseHiResPose(text.substring("POSE,".length()));
        } else if (text.startsWith("TARGET,")) {
            parseTarget(text.substring("TARGET,".length()));
        } else if (text.startsWith("T1_STATE,")) {
            parseT1State(text.substring("T1_STATE,".length()));
        } else if (text.startsWith("T1_TARGET,")) {
            parseT1Target(text.substring("T1_TARGET,".length()));
        } else if (text.startsWith("T2_STATE,")) {
            parseT2State(text.substring("T2_STATE,".length()));
        } else if (text.startsWith("T2_ARROW,")) {
            parseT2Arrow(text.substring("T2_ARROW,".length()));
        } else if (text.startsWith("ESTOP_ALERT,")) {
            parseEstopAlert(text.substring("ESTOP_ALERT,".length()));
        } else if (text.startsWith("SENSORS,")) {
            parseSensors(text.substring("SENSORS,".length()));
        }
    }

    // android_bridge_node's default Checklist C.10 pose format wraps each
    // field in literal angle brackets and reports grid cells [0..19] over
    // the 200cm arena (10cm/cell) -- see docs/task1-fsm-and-orbit-recovery.md
    // §4.1. This used to plain-Float.parseFloat("<5>"), which always threw
    // and silently dropped every pose update (caught below), so the arena
    // view never moved. Strip the brackets and convert to what ArenaView
    // actually draws in: centimetres + degrees CCW from East.
    private static final float CM_PER_GRID_CELL = 10f;

    private void parseRobotPose(String payload) {
        String[] parts = payload.split(",");
        if (parts.length != 3) {
            return;
        }
        try {
            float gridX = Float.parseFloat(stripBrackets(parts[0]));
            float gridY = Float.parseFloat(stripBrackets(parts[1]));
            float heading = parseHeading(parts[2]);
            // +half a cell so the marker sits at the cell's center rather
            // than its corner.
            float xCm = gridX * CM_PER_GRID_CELL + CM_PER_GRID_CELL / 2f;
            float yCm = gridY * CM_PER_GRID_CELL + CM_PER_GRID_CELL / 2f;
            mainHandler.post(() -> listener.onRobotPose(xCm, yCm, heading));
        } catch (NumberFormatException ignored) {
            // Malformed line -- already visible via onDebug, nothing more to do.
        }
    }

    private static String stripBrackets(String field) {
        String trimmed = field.trim();
        if (trimmed.length() >= 2 && trimmed.startsWith("<") && trimmed.endsWith(">")) {
            return trimmed.substring(1, trimmed.length() - 1);
        }
        return trimmed;
    }

    /** Accepts a Checklist C.10 cardinal letter (N/E/S/W, matching
     * ArenaView's "degrees CCW from East" convention) or, if
     * android_bridge_node's direction_as_cardinal param is ever set False,
     * a plain numeric heading in degrees. */
    private static float parseHeading(String field) {
        String value = stripBrackets(field).toUpperCase(java.util.Locale.US);
        switch (value) {
            case "E":
                return 0f;
            case "N":
                return 90f;
            case "W":
                return 180f;
            case "S":
                return 270f;
            default:
                return Float.parseFloat(value);
        }
    }

    // POSE,<x_cm>,<y_cm>,<yaw_deg> -- supplemental, non-checklist pose line
    // android_bridge_node sends alongside (not instead of) ROBOT, at full
    // precision and up to hires_pose_rate_hz. ROBOT's 4-heading/10cm-cell
    // resolution makes rotation (e.g. circling) look like it isn't being
    // tracked at all; this feeds the same onRobotPose() callback with
    // values that actually change every update.
    private void parseHiResPose(String payload) {
        String[] parts = payload.split(",");
        if (parts.length != 3) {
            return;
        }
        try {
            float xCm = Float.parseFloat(parts[0].trim());
            float yCm = Float.parseFloat(parts[1].trim());
            float headingDeg = Float.parseFloat(parts[2].trim());
            mainHandler.post(() -> listener.onRobotPose(xCm, yCm, headingDeg));
        } catch (NumberFormatException ignored) {
            // Malformed line -- already visible via onDebug, nothing more to do.
        }
    }

    private void parseTarget(String payload) {
        String[] parts = payload.split(",");
        if (parts.length != 2) {
            return;
        }
        try {
            int obstacleId = Integer.parseInt(parts[0].trim());
            int symbolId = Integer.parseInt(parts[1].trim());
            mainHandler.post(() -> listener.onTarget(obstacleId, symbolId));
        } catch (NumberFormatException ignored) {
            // Malformed line -- already visible via onDebug, nothing more to do.
        }
    }

    // T1_STATE,<state>,<current_leg>,<total_legs>,<obs_id>,<face>,<rem_dist_cm>
    private void parseT1State(String payload) {
        String[] parts = payload.split(",");
        if (parts.length < 5) {
            return;
        }
        try {
            String state = parts[0].trim();
            int currentLeg = Integer.parseInt(parts[1].trim());
            int totalLegs = Integer.parseInt(parts[2].trim());
            int obsId = Integer.parseInt(parts[3].trim());
            String face = parts[4].trim();
            float remDistCm = parts.length > 5 ? Float.parseFloat(parts[5].trim()) : 0f;
            mainHandler.post(() -> listener.onT1State(state, currentLeg, totalLegs, obsId, face, remDistCm));
        } catch (Exception ignored) {
        }
    }

    // T1_TARGET,<obs_id>,<symbol_id>,<symbol_name>,<confidence>,<face>
    private void parseT1Target(String payload) {
        String[] parts = payload.split(",");
        if (parts.length < 5) {
            return;
        }
        try {
            int obsId = Integer.parseInt(parts[0].trim());
            int symbolId = Integer.parseInt(parts[1].trim());
            String symbolName = parts[2].trim();
            float confidence = Float.parseFloat(parts[3].trim());
            String face = parts[4].trim();
            mainHandler.post(() -> listener.onT1Target(obsId, symbolId, symbolName, confidence, face));
        } catch (Exception ignored) {
        }
    }

    // T2_STATE,<state>,<step_idx>,<step_desc>,<elapsed_sec>
    private void parseT2State(String payload) {
        String[] parts = payload.split(",", 4);
        if (parts.length < 4) {
            return;
        }
        try {
            String state = parts[0].trim();
            int stepIdx = Integer.parseInt(parts[1].trim());
            String stepDesc = parts[2].trim();
            float elapsedSec = Float.parseFloat(parts[3].trim());
            mainHandler.post(() -> listener.onT2State(state, stepIdx, stepDesc, elapsedSec));
        } catch (Exception ignored) {
        }
    }

    // T2_ARROW,<obs_num>,<LEFT|RIGHT>,<symbol_id>,<confidence>
    private void parseT2Arrow(String payload) {
        String[] parts = payload.split(",");
        if (parts.length < 4) {
            return;
        }
        try {
            int obsNum = Integer.parseInt(parts[0].trim());
            String direction = parts[1].trim().toUpperCase(java.util.Locale.US);
            int symbolId = Integer.parseInt(parts[2].trim());
            float confidence = Float.parseFloat(parts[3].trim());
            mainHandler.post(() -> listener.onT2Arrow(obsNum, direction, symbolId, confidence));
        } catch (Exception ignored) {
        }
    }

    // SENSORS,<us_cm>,<ir_left_cm>,<ir_right_cm>
    private void parseSensors(String payload) {
        String[] parts = payload.split(",");
        if (parts.length < 3) {
            return;
        }
        try {
            float usCm = Float.parseFloat(parts[0].trim());
            float irLeftCm = Float.parseFloat(parts[1].trim());
            float irRightCm = Float.parseFloat(parts[2].trim());
            mainHandler.post(() -> listener.onSensors(usCm, irLeftCm, irRightCm));
        } catch (Exception ignored) {
        }
    }

    // ESTOP_ALERT,<sensor>,<dist_cm>,<threshold_cm>[,<reason>]
    private void parseEstopAlert(String payload) {
        String[] parts = payload.split(",");
        if (parts.length < 2) {
            return;
        }
        try {
            String sensor = parts[0].trim();
            float distCm = Float.parseFloat(parts[1].trim());
            float threshCm = parts.length > 2 ? Float.parseFloat(parts[2].trim()) : 12.0f;
            String reason = parts.length > 3 ? parts[3].trim() : "Proximity Violation";
            mainHandler.post(() -> listener.onEstopAlert(sensor, distCm, threshCm, reason));
        } catch (Exception ignored) {
        }
    }

    private void postDisconnected(String reason) {
        mainHandler.post(() -> listener.onDisconnected(reason));
    }

    private void closeQuietly() {
        outputStream = null;
        BluetoothSocket sock = socket;
        socket = null;
        if (sock != null) {
            try {
                sock.close();
            } catch (IOException ignored) {
                // already going away
            }
        }
    }
}
