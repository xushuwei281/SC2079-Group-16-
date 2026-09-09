package com.sc2079.group16.controller;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

/**
 * Single-screen remote control: a list of already-paired devices to connect
 * to, then a movement control panel once connected. Speaks the wire format
 * in docs/protocol.md over the link owned by {@link BluetoothLinkService}.
 */
public class MainActivity extends Activity implements BluetoothLinkService.Listener {

    private static final int REQUEST_BT_CONNECT = 1001;

    private enum Mode {
        MANUAL,
        TASK1,
        TASK2
    }

    private Mode currentMode = Mode.MANUAL;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLinkService linkService;

    private final List<BluetoothDevice> bondedDevices = new ArrayList<>();

    private ListView deviceListView;
    private View devicePickerPanel;
    private View controlPanel;
    private TextView connectedDeviceLabel;
    private View statusDot;
    private TextView linkStatusText;
    private TextView statusTextView;
    private ScrollView statusScrollView;
    private JoystickView joystickView;
    private View joystickPanel;
    private View arenaPanel;
    private ArenaView arenaView;
    private Button driveModeButton;
    private Button arenaModeButton;

    private View panelManual;
    private View panelTask1;
    private View panelTask2;
    private Button tabManual;
    private Button tabTask1;
    private Button tabTask2;

    // ---- Joystick -> discrete move translation ----------------------------
    // The STM32 firmware has no continuous-velocity primitive -- every move
    // is one discrete, blocking maneuver (see mdp_hardware_bridge). The
    // joystick fakes continuous control by repeatedly issuing small moves
    // while held, pacing itself off each move's actual completion (DONE / a
    // terminal STATUS) rather than a fixed timer, so it never outruns
    // /execute_moves's one-call-at-a-time arbitration on the Pi.
    private static final float JOYSTICK_DEADZONE = 0.15f;
    private static final double STRAIGHT_ANGLE_DEG = 20.0; // within this of dead-ahead/dead-astern -> FC/BC
    private static final int MIN_STEP_DIST_CM = 2;
    private static final int MAX_STEP_DIST_CM = 8; // stays < 100 so android_bridge_node's mm-heuristic never fires
    private static final int MIN_STEP_TURN_DEG = 2; // the "2-degree interval" ask -- FL/FR/BL/BR already support this
    private static final int MAX_STEP_TURN_DEG = 20;
    private static final long JOYSTICK_POLL_MS = 70;
    private static final long MOVE_STUCK_TIMEOUT_MS = 4000; // recover if a response never arrives

    private final Handler joystickHandler = new Handler(Looper.getMainLooper());
    private boolean moveInFlight = false;
    private long lastMoveSentAt = 0L;

    // ---- Link liveness indicator -------------------------------------
    // Ground truth for "is the app actually talking to the robot right
    // now" is "have we heard anything back over Bluetooth recently" --
    // a still-open socket doesn't prove the Pi-side process is alive.
    private static final long LINK_STATUS_TICK_MS = 1000;
    private static final long LINK_STALE_AFTER_MS = 6000;

    private final Handler linkStatusHandler = new Handler(Looper.getMainLooper());
    private boolean linkConnected = false;
    private long lastLinkRxAtMillis = 0L;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        deviceListView = findViewById(R.id.deviceList);
        devicePickerPanel = findViewById(R.id.devicePickerPanel);
        controlPanel = findViewById(R.id.controlPanel);
        connectedDeviceLabel = findViewById(R.id.connectedDeviceLabel);
        statusDot = findViewById(R.id.statusDot);
        linkStatusText = findViewById(R.id.linkStatusText);
        statusTextView = findViewById(R.id.statusText);
        statusScrollView = findViewById(R.id.statusScrollView);
        joystickView = findViewById(R.id.joystick);
        joystickPanel = findViewById(R.id.joystickPanel);
        arenaPanel = findViewById(R.id.arenaPanel);
        arenaView = findViewById(R.id.arenaView);
        driveModeButton = findViewById(R.id.driveModeButton);
        arenaModeButton = findViewById(R.id.arenaModeButton);

        panelManual = findViewById(R.id.panelManual);
        panelTask1 = findViewById(R.id.panelTask1);
        panelTask2 = findViewById(R.id.panelTask2);

        tabManual = findViewById(R.id.tabManual);
        tabTask1 = findViewById(R.id.tabTask1);
        tabTask2 = findViewById(R.id.tabTask2);

        tabManual.setOnClickListener(v -> setMode(Mode.MANUAL));
        tabTask1.setOnClickListener(v -> setMode(Mode.TASK1));
        tabTask2.setOnClickListener(v -> setMode(Mode.TASK2));

        findViewById(R.id.refreshButton).setOnClickListener(v -> refreshDeviceList());
        findViewById(R.id.disconnectButton).setOnClickListener(v -> disconnect());
        findViewById(R.id.startTask1Button).setOnClickListener(v -> linkService.sendLine("START"));
        findViewById(R.id.startTask2Button).setOnClickListener(v -> linkService.sendLine("START_TASK2"));
        findViewById(R.id.resetButton).setOnClickListener(v -> linkService.sendLine("RESET"));
        findViewById(R.id.stopButton).setOnClickListener(v -> linkService.sendLine("STP"));
        findViewById(R.id.driveModeButton).setOnClickListener(v -> showDriveMode());
        findViewById(R.id.arenaModeButton).setOnClickListener(v -> showArenaMode());
        findViewById(R.id.addObstacleButton).setOnClickListener(v -> arenaView.addObstacle());
        findViewById(R.id.clearObstaclesButton).setOnClickListener(v -> arenaView.clearObstacles());
        findViewById(R.id.sendArenaButton).setOnClickListener(v -> sendArenaLayout());
        findViewById(R.id.clearLogsButton).setOnClickListener(v -> statusTextView.setText(""));

        deviceListView.setOnItemClickListener(
                (parent, view, position, id) -> connectTo(bondedDevices.get(position)));

        linkService = new BluetoothLinkService(this);

        BluetoothManager btManager = getSystemService(BluetoothManager.class);
        bluetoothAdapter = btManager != null ? btManager.getAdapter() : null;

        ensurePermissionThenListDevices();
        showDriveMode(); // sets the initial active/inactive button styling

        joystickHandler.post(this::joystickTick);
        linkStatusHandler.post(this::linkStatusTick);
    }

    private void ensurePermissionThenListDevices() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(
                        new String[] {Manifest.permission.BLUETOOTH_CONNECT}, REQUEST_BT_CONNECT);
                return;
            }
        }
        refreshDeviceList();
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_BT_CONNECT) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                refreshDeviceList();
            } else {
                Toast.makeText(
                                this,
                                "Bluetooth permission is required to connect to the robot",
                                Toast.LENGTH_LONG)
                        .show();
            }
        }
    }

    @SuppressLint("MissingPermission") // guarded by ensurePermissionThenListDevices()
    private void refreshDeviceList() {
        if (bluetoothAdapter == null) {
            Toast.makeText(this, "This device has no Bluetooth adapter", Toast.LENGTH_LONG).show();
            return;
        }
        if (!bluetoothAdapter.isEnabled()) {
            Toast.makeText(this, "Turn Bluetooth on, then tap Refresh", Toast.LENGTH_LONG).show();
            return;
        }

        bondedDevices.clear();
        bondedDevices.addAll(bluetoothAdapter.getBondedDevices());

        String[] labels = new String[bondedDevices.size()];
        for (int i = 0; i < bondedDevices.size(); i++) {
            BluetoothDevice d = bondedDevices.get(i);
            labels[i] = safeName(d) + "\n" + d.getAddress();
        }
        deviceListView.setAdapter(
                new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, labels));

        if (bondedDevices.isEmpty()) {
            Toast.makeText(
                            this,
                            "No paired devices -- pair with the Pi first "
                                    + "(see ros2_ws/bluetooth-setup/)",
                            Toast.LENGTH_LONG)
                    .show();
        }
    }

    private void connectTo(BluetoothDevice device) {
        // Toast, not appendStatus(): the status log lives inside controlPanel,
        // which is still View.GONE at this point (it only becomes visible in
        // onConnected()) -- writing there now would be invisible to the user.
        Toast.makeText(this, "Connecting to " + safeName(device) + "...", Toast.LENGTH_SHORT)
                .show();
        linkService.connect(device);
    }

    private void disconnect() {
        linkService.disconnect();
        showDevicePicker();
    }

    /** Runs every {@link #JOYSTICK_POLL_MS} for the life of the Activity; a
     * held-still joystick produces no touch events, so this polls its
     * current position rather than reacting to callbacks. */
    private void joystickTick() {
        joystickHandler.postDelayed(this::joystickTick, JOYSTICK_POLL_MS);

        if (controlPanel.getVisibility() != View.VISIBLE || currentMode != Mode.MANUAL) {
            return;
        }

        if (moveInFlight) {
            if (System.currentTimeMillis() - lastMoveSentAt > MOVE_STUCK_TIMEOUT_MS) {
                moveInFlight = false; // no DONE/STATUS ever arrived -- don't wedge the joystick
            } else {
                return; // previous step still executing; let it finish
            }
        }

        float right = joystickView.getStickRight();
        float forward = joystickView.getStickForward();
        float magnitude = (float) Math.hypot(right, forward);
        if (magnitude < JOYSTICK_DEADZONE) {
            return;
        }
        magnitude = Math.min(magnitude, 1f);

        double angleFromForwardDeg = Math.toDegrees(Math.atan2(right, forward));
        String command;
        int value;
        if (Math.abs(angleFromForwardDeg) <= 90.0) {
            if (Math.abs(angleFromForwardDeg) <= STRAIGHT_ANGLE_DEG) {
                command = "FC";
                value = scaleStep(magnitude, MIN_STEP_DIST_CM, MAX_STEP_DIST_CM);
            } else {
                command = angleFromForwardDeg > 0 ? "FR" : "FL";
                value = scaleStep(magnitude, MIN_STEP_TURN_DEG, MAX_STEP_TURN_DEG);
            }
        } else {
            double deviationFromBack = 180.0 - Math.abs(angleFromForwardDeg);
            if (deviationFromBack <= STRAIGHT_ANGLE_DEG) {
                command = "BC";
                value = scaleStep(magnitude, MIN_STEP_DIST_CM, MAX_STEP_DIST_CM);
            } else {
                command = right > 0 ? "BR" : "BL";
                value = scaleStep(magnitude, MIN_STEP_TURN_DEG, MAX_STEP_TURN_DEG);
            }
        }

        moveInFlight = true;
        lastMoveSentAt = System.currentTimeMillis();
        linkService.sendLine(command + ":" + value);
    }

    /** Runs every {@link #LINK_STATUS_TICK_MS} for the life of the Activity,
     * independent of mode/panel visibility, so the indicator keeps counting
     * up even while a task panel or the arena view is showing. */
    private void linkStatusTick() {
        linkStatusHandler.postDelayed(this::linkStatusTick, LINK_STATUS_TICK_MS);
        updateLinkStatus();
    }

    private void updateLinkStatus() {
        if (!linkConnected) {
            setDotColor(R.color.text_disabled);
            linkStatusText.setText(R.string.link_status_not_connected);
            return;
        }
        long ageMs = System.currentTimeMillis() - lastLinkRxAtMillis;
        if (ageMs < LINK_STALE_AFTER_MS) {
            setDotColor(R.color.status_accent_fill);
            linkStatusText.setText(
                    ageMs < 1000
                            ? getString(R.string.link_status_live_now)
                            : getString(R.string.link_status_live, ageMs / 1000));
        } else {
            setDotColor(R.color.status_error_fill);
            linkStatusText.setText(getString(R.string.link_status_stale, ageMs / 1000));
        }
    }

    private void setDotColor(int colorRes) {
        statusDot.setBackgroundTintList(ColorStateList.valueOf(getColor(colorRes)));
    }

    /** Maps a deflection in [DEADZONE, 1] onto [min, max]. */
    private int scaleStep(float magnitude, int min, int max) {
        float t = (magnitude - JOYSTICK_DEADZONE) / (1f - JOYSTICK_DEADZONE);
        t = Math.max(0f, Math.min(1f, t));
        return Math.round(min + t * (max - min));
    }

    private void showDriveMode() {
        joystickPanel.setVisibility(View.VISIBLE);
        arenaPanel.setVisibility(View.GONE);
        setModeButtonActive(driveModeButton, arenaModeButton);
    }

    private void showArenaMode() {
        joystickPanel.setVisibility(View.GONE);
        arenaPanel.setVisibility(View.VISIBLE);
        setModeButtonActive(arenaModeButton, driveModeButton);
    }

    /** Gives the two mode-toggle buttons a real selected-state distinction --
     * without this, DRIVE and ARENA are visually identical regardless of
     * which one is actually showing. */
    private void setModeButtonActive(Button active, Button inactive) {
        active.setBackgroundResource(R.drawable.ripple_button_primary);
        active.setTextColor(getColor(R.color.on_accent));
        inactive.setBackgroundResource(R.drawable.ripple_button_secondary);
        inactive.setTextColor(getColor(R.color.text_primary));
    }

    /** Sends the current obstacle layout as one ALG|id,x,y,face|... command --
     * this is the "at the end of the interaction" transmission step: the
     * whole layout goes in one message, matching planner_node.py's parser
     * exactly (see _parse_and_plan). */
    private void sendArenaLayout() {
        if (arenaView.getObstacleCount() == 0) {
            Toast.makeText(this, "No obstacles placed yet", Toast.LENGTH_SHORT).show();
            return;
        }
        linkService.sendLine(arenaView.buildAlgCommand());
    }

    private void showDevicePicker() {
        devicePickerPanel.setVisibility(View.VISIBLE);
        controlPanel.setVisibility(View.GONE);
    }

    private void showControlPanel() {
        devicePickerPanel.setVisibility(View.GONE);
        controlPanel.setVisibility(View.VISIBLE);
        setMode(Mode.MANUAL);
    }

    private void setMode(Mode mode) {
        currentMode = mode;
        panelManual.setVisibility(mode == Mode.MANUAL ? View.VISIBLE : View.GONE);
        panelTask1.setVisibility(mode == Mode.TASK1 ? View.VISIBLE : View.GONE);
        panelTask2.setVisibility(mode == Mode.TASK2 ? View.VISIBLE : View.GONE);

        tabManual.setBackgroundTintList(ColorStateList.valueOf(
                getColor(mode == Mode.MANUAL ? R.color.tab_active : R.color.tab_inactive)));
        tabManual.setTextColor(getColor(mode == Mode.MANUAL ? android.R.color.white : android.R.color.black));

        tabTask1.setBackgroundTintList(ColorStateList.valueOf(
                getColor(mode == Mode.TASK1 ? R.color.tab_active : R.color.tab_inactive)));
        tabTask1.setTextColor(getColor(mode == Mode.TASK1 ? android.R.color.white : android.R.color.black));

        tabTask2.setBackgroundTintList(ColorStateList.valueOf(
                getColor(mode == Mode.TASK2 ? R.color.tab_active : R.color.tab_inactive)));
        tabTask2.setTextColor(getColor(mode == Mode.TASK2 ? android.R.color.white : android.R.color.black));
    }

    private void appendStatus(String text) {
        Log.i("StatusLog", text); // mirror to logcat -- `adb logcat -s StatusLog` for debugging
        statusTextView.append(text + "\n");
        // statusTextView is wrap_content inside statusScrollView -- it's the
        // ScrollView that actually scrolls, not the TextView itself, so that's
        // what needs telling to follow new lines. Posted because the layout
        // pass for the just-appended line hasn't happened yet on this call.
        statusScrollView.post(() -> statusScrollView.fullScroll(View.FOCUS_DOWN));
    }

    @SuppressLint("MissingPermission")
    private String safeName(BluetoothDevice device) {
        String name = device.getName();
        return name != null ? name : device.getAddress();
    }

    // ---- BluetoothLinkService.Listener --------------------------------

    @Override
    public void onConnected(BluetoothDevice device) {
        connectedDeviceLabel.setText(getString(R.string.connected_prefix) + " " + safeName(device));
        appendStatus("Connected to " + safeName(device));
        linkConnected = true;
        lastLinkRxAtMillis = System.currentTimeMillis();
        updateLinkStatus();
        showControlPanel();
    }

    @Override
    public void onStatusLine(String text) {
        appendStatus("STATUS: " + text);
        // "Moving ..." is an ack sent the instant a move is dispatched, before
        // it actually completes -- every other STATUS line here is terminal
        // (a failure/rejection), so it's the joystick loop's cue to proceed.
        if (!text.startsWith("Moving")) {
            moveInFlight = false;
        }
    }

    @Override
    public void onDone() {
        appendStatus("DONE");
        moveInFlight = false;
    }

    @Override
    public void onDebug(String line) {
        appendStatus(line);
        // Any actual inbound traffic (not our own "queuing"/"→ sent" echoes)
        // is proof the Pi side is alive and talking back -- reset the
        // staleness clock immediately rather than waiting for the next tick.
        if (line.startsWith("← ")) {
            lastLinkRxAtMillis = System.currentTimeMillis();
            updateLinkStatus();
        }
    }

    @Override
    public void onRobotPose(float xCm, float yCm, float headingDeg) {
        arenaView.setRobotPose(xCm, yCm, headingDeg);
    }

    @Override
    public void onTarget(int obstacleId, int symbolId) {
        arenaView.setRecognizedSymbol(obstacleId, symbolId);
    }

    @Override
    public void onDisconnected(String reason) {
        // Toast as well as appendStatus(): this can fire before a connection
        // ever succeeded (e.g. the initial connect attempt itself failing),
        // in which case controlPanel -- and its status log -- has never been
        // shown, so appendStatus() alone would be silently invisible.
        Toast.makeText(this, "Disconnected: " + reason, Toast.LENGTH_LONG).show();
        appendStatus("Disconnected (" + reason + ")");
        moveInFlight = false; // don't carry stale in-flight state into the next connection
        linkConnected = false;
        updateLinkStatus();
        showDevicePicker();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        linkService.shutdown();
    }
}
