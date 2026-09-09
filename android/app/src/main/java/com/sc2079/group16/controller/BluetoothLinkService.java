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
 * Speaks the ASCII line protocol from docs/protocol.md: FW:&lt;mm&gt;,
 * BW:&lt;mm&gt;, TL:&lt;deg&gt;, TR:&lt;deg&gt;, STP out; STATUS,&lt;text&gt;
 * and DONE in.
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
        mainHandler.post(() -> listener.onDebug("← " + text));
        if (text.equals("DONE")) {
            mainHandler.post(listener::onDone);
        } else if (text.startsWith("STATUS,")) {
            String payload = text.substring("STATUS,".length());
            mainHandler.post(() -> listener.onStatusLine(payload));
        } else if (text.startsWith("ROBOT,")) {
            parseRobotPose(text.substring("ROBOT,".length()));
        } else if (text.startsWith("TARGET,")) {
            parseTarget(text.substring("TARGET,".length()));
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
