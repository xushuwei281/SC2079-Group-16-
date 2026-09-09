package com.sc2079.group16.controller;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Typeface;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ListView;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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
    private FrameLayout manualArenaContainer;
    private FrameLayout task1ArenaContainer;
    private Button driveModeButton;
    private Button arenaModeButton;

    private View panelManual;
    private View panelTask1;
    private View panelTask2;
    private Button tabManual;
    private Button tabTask1;
    private Button tabTask2;

    // ---- Task 1 UI & Stopwatch -------------------------------------------
    private TextView t1TimerText;
    private TextView t1StateBadge;
    private TextView t1LegText;
    private TextView t1SensorText;
    private TextView t1TargetsText;
    private final Handler t1TimerHandler = new Handler(Looper.getMainLooper());
    private boolean t1TimerRunning = false;
    private long t1StartTimeMillis = 0L;
    private long t1ElapsedMillis = 0L;
    private final Map<Integer, String> t1RecognizedTargets = new LinkedHashMap<>();

    // ---- Task 2 UI & Stopwatch -------------------------------------------
    private TextView t2TimerText;
    private TextView t2StateBadge;
    private TextView t2Obs1Arrow;
    private TextView t2Obs1Details;
    private TextView t2Obs2Arrow;
    private TextView t2Obs2Details;
    private TextView t2StepTitle;
    private final TextView[] t2Steps = new TextView[7];
    private static final String[] T2_STEP_LABELS = {
        "1. Approach Obstacle 1",
        "2. Scan Arrow 1 (Left / Right)",
        "3. Slalom Obstacle 1",
        "4. Approach Obstacle 2",
        "5. Scan Arrow 2 (Left / Right)",
        "6. Round Obs 2 & Return Loop",
        "7. Carpark Sprint & Finish"
    };
    private TextView t2SensorText;
    private TextView t2ManeuverText;
    private final Handler t2TimerHandler = new Handler(Looper.getMainLooper());
    private boolean t2TimerRunning = false;
    private long t2StartTimeMillis = 0L;
    private long t2ElapsedMillis = 0L;

    // ---- Fallback Regex for Legacy / Mixed Text Logs ---------------------
    private static final Pattern OBS_ARROW_PATTERN = Pattern.compile(
            "Obs\\s*([12]):\\s*Arrow\\s*(LEFT|RIGHT)(?:\\s*\\(symbol\\s*(\\d+)(?:,\\s*conf\\s*([0-9.]+))?\\))?",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern T2_COMPLETE_PATTERN = Pattern.compile(
            "Task\\s*2\\s*Complete!?(?:\\s*Time:\\s*([0-9.]+)s?)?",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern T1_COMPLETE_PATTERN = Pattern.compile(
            "Mission completed(?: in ([0-9.]+)s)?",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern FSM_TRANSITION_PATTERN = Pattern.compile(
            "FSM:\\s*\\w+\\s*->\\s*(\\w+)",
            Pattern.CASE_INSENSITIVE);

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
        manualArenaContainer = findViewById(R.id.manualArenaContainer);
        task1ArenaContainer = findViewById(R.id.task1ArenaContainer);
        driveModeButton = findViewById(R.id.driveModeButton);
        arenaModeButton = findViewById(R.id.arenaModeButton);

        panelManual = findViewById(R.id.panelManual);
        panelTask1 = findViewById(R.id.panelTask1);
        panelTask2 = findViewById(R.id.panelTask2);

        tabManual = findViewById(R.id.tabManual);
        tabTask1 = findViewById(R.id.tabTask1);
        tabTask2 = findViewById(R.id.tabTask2);

        // Task 1 widget bindings
        t1TimerText = findViewById(R.id.t1TimerText);
        t1StateBadge = findViewById(R.id.t1StateBadge);
        t1LegText = findViewById(R.id.t1LegText);
        t1SensorText = findViewById(R.id.t1SensorText);
        t1TargetsText = findViewById(R.id.t1TargetsText);

        // Task 2 widget bindings
        t2TimerText = findViewById(R.id.t2TimerText);
        t2StateBadge = findViewById(R.id.t2StateBadge);
        t2Obs1Arrow = findViewById(R.id.t2Obs1Arrow);
        t2Obs1Details = findViewById(R.id.t2Obs1Details);
        t2Obs2Arrow = findViewById(R.id.t2Obs2Arrow);
        t2Obs2Details = findViewById(R.id.t2Obs2Details);
        t2StepTitle = findViewById(R.id.t2StepTitle);
        t2Steps[0] = findViewById(R.id.t2Step1);
        t2Steps[1] = findViewById(R.id.t2Step2);
        t2Steps[2] = findViewById(R.id.t2Step3);
        t2Steps[3] = findViewById(R.id.t2Step4);
        t2Steps[4] = findViewById(R.id.t2Step5);
        t2Steps[5] = findViewById(R.id.t2Step6);
        t2Steps[6] = findViewById(R.id.t2Step7);
        t2SensorText = findViewById(R.id.t2SensorText);
        t2ManeuverText = findViewById(R.id.t2ManeuverText);

        tabManual.setOnClickListener(v -> setMode(Mode.MANUAL));
        tabTask1.setOnClickListener(v -> setMode(Mode.TASK1));
        tabTask2.setOnClickListener(v -> setMode(Mode.TASK2));

        findViewById(R.id.refreshButton).setOnClickListener(v -> refreshDeviceList());
        findViewById(R.id.disconnectButton).setOnClickListener(v -> disconnect());

        findViewById(R.id.startTask1Button).setOnClickListener(v -> {
            resetT1Views();
            startT1Timer();
            updateT1StateBadge("PLANNING");
            linkService.sendLine("START");
        });

        findViewById(R.id.startTask2Button).setOnClickListener(v -> {
            resetT2Views();
            startT2Timer();
            updateT2StateBadge("APPROACH_OBS1");
            linkService.sendLine("START_TASK2");
        });

        findViewById(R.id.resetButton).setOnClickListener(v -> {
            resetT1Views();
            resetT2Views();
            linkService.sendLine("RESET");
        });

        findViewById(R.id.stopButton).setOnClickListener(v -> {
            stopT1Timer();
            stopT2Timer();
            updateT1StateBadge("ESTOP");
            updateT2StateBadge("ESTOP");
            linkService.sendLine("STP");
        });

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

        resetT1Views();
        resetT2Views();

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

        // Seamless 2D Arena reparenting: share single ArenaView between Manual and Task 1
        if (mode == Mode.TASK1) {
            if (arenaView != null && task1ArenaContainer != null && arenaView.getParent() != task1ArenaContainer) {
                if (arenaView.getParent() != null) {
                    ((ViewGroup) arenaView.getParent()).removeView(arenaView);
                }
                task1ArenaContainer.addView(arenaView, new FrameLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        } else {
            if (arenaView != null && manualArenaContainer != null && arenaView.getParent() != manualArenaContainer) {
                if (arenaView.getParent() != null) {
                    ((ViewGroup) arenaView.getParent()).removeView(arenaView);
                }
                manualArenaContainer.addView(arenaView, new FrameLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        }
    }

    // ---- Task 1 Stopwatch & State Helpers --------------------------------

    private void startT1Timer() {
        if (t1TimerRunning) return;
        t1TimerRunning = true;
        t1StartTimeMillis = System.currentTimeMillis() - t1ElapsedMillis;
        t1TimerHandler.post(t1TimerRunnable);
    }

    private void stopT1Timer() {
        t1TimerRunning = false;
        t1TimerHandler.removeCallbacks(t1TimerRunnable);
    }

    private void resetT1Timer() {
        stopT1Timer();
        t1ElapsedMillis = 0L;
        if (t1TimerText != null) {
            t1TimerText.setText(R.string.timer_default_t1);
            t1TimerText.setTextColor(getColor(R.color.text_primary));
        }
    }

    private final Runnable t1TimerRunnable = new Runnable() {
        @Override
        public void run() {
            if (!t1TimerRunning) return;
            t1ElapsedMillis = System.currentTimeMillis() - t1StartTimeMillis;
            long totalSec = t1ElapsedMillis / 1000;
            long mins = totalSec / 60;
            long secs = totalSec % 60;
            t1TimerText.setText(String.format(Locale.US, "%02d:%02d / 06:00", mins, secs));
            if (totalSec >= 300) {
                t1TimerText.setTextColor(getColor(R.color.status_error_fill));
            } else {
                t1TimerText.setTextColor(getColor(R.color.text_primary));
            }
            t1TimerHandler.postDelayed(this, 250);
        }
    };

    private void updateT1StateBadge(String state) {
        if (state == null || state.isEmpty()) state = "IDLE";
        t1StateBadge.setText(state);
        int colorRes;
        if (state.equalsIgnoreCase("IDLE")) {
            colorRes = R.color.badge_idle;
        } else if (state.equalsIgnoreCase("PLANNING")) {
            colorRes = R.color.badge_warning;
        } else if (state.equalsIgnoreCase("NAVIGATING") || state.equalsIgnoreCase("EXPLORING")) {
            colorRes = R.color.badge_active;
        } else if (state.equalsIgnoreCase("SCANNING") || state.equalsIgnoreCase("SAMPLING_TARGET") || state.contains("ORBIT")) {
            colorRes = R.color.badge_warning;
        } else if (state.equalsIgnoreCase("DONE") || state.equalsIgnoreCase("COMPLETED") || state.equalsIgnoreCase("MISSION_COMPLETE")) {
            colorRes = R.color.badge_success;
        } else if (state.equalsIgnoreCase("ESTOP") || state.equalsIgnoreCase("ERROR")) {
            colorRes = R.color.badge_estop;
        } else {
            colorRes = R.color.badge_active;
        }
        t1StateBadge.setBackgroundTintList(ColorStateList.valueOf(getColor(colorRes)));
    }

    private void resetT1Views() {
        resetT1Timer();
        updateT1StateBadge("IDLE");
        t1LegText.setText("Leg: Ready | Target: --");
        t1SensorText.setText("US: -- cm | IR: -- cm");
        t1RecognizedTargets.clear();
        t1TargetsText.setText("🎯 Targets: Standing by for mission start");
    }

    // ---- Task 2 Stopwatch & Stepper Helpers ------------------------------

    private void startT2Timer() {
        if (t2TimerRunning) return;
        t2TimerRunning = true;
        t2StartTimeMillis = System.currentTimeMillis() - t2ElapsedMillis;
        t2TimerHandler.post(t2TimerRunnable);
    }

    private void stopT2Timer() {
        t2TimerRunning = false;
        t2TimerHandler.removeCallbacks(t2TimerRunnable);
    }

    private void resetT2Timer() {
        stopT2Timer();
        t2ElapsedMillis = 0L;
        if (t2TimerText != null) {
            t2TimerText.setText(R.string.timer_default_t2);
        }
    }

    private final Runnable t2TimerRunnable = new Runnable() {
        @Override
        public void run() {
            if (!t2TimerRunning) return;
            t2ElapsedMillis = System.currentTimeMillis() - t2StartTimeMillis;
            float sec = t2ElapsedMillis / 1000.0f;
            t2TimerText.setText(String.format(Locale.US, "%05.2fs", sec));
            t2TimerHandler.postDelayed(this, 50);
        }
    };

    private void updateT2StateBadge(String state) {
        if (state == null || state.isEmpty()) state = "IDLE";
        t2StateBadge.setText(state);
        int colorRes;
        if (state.equalsIgnoreCase("IDLE")) {
            colorRes = R.color.badge_idle;
        } else if (state.equalsIgnoreCase("COMPLETE") || state.equalsIgnoreCase("DONE") || state.equalsIgnoreCase("COMPLETED")) {
            colorRes = R.color.badge_success;
        } else if (state.equalsIgnoreCase("ESTOP") || state.equalsIgnoreCase("ERROR")) {
            colorRes = R.color.badge_estop;
        } else if (state.contains("DETECT") || state.contains("SCAN")) {
            colorRes = R.color.badge_warning;
        } else {
            colorRes = R.color.badge_active;
        }
        t2StateBadge.setBackgroundTintList(ColorStateList.valueOf(getColor(colorRes)));
    }

    private void updateT2PipelineStep(int activeStep) {
        for (int i = 0; i < t2Steps.length; i++) {
            if (t2Steps[i] == null) continue;
            int stepNum = i + 1;
            if (activeStep == 0) {
                t2Steps[i].setText(T2_STEP_LABELS[i]);
                t2Steps[i].setTextColor(getColor(R.color.text_secondary));
                t2Steps[i].setTypeface(null, Typeface.NORMAL);
            } else if (stepNum < activeStep || activeStep >= 8) {
                t2Steps[i].setText("✓ " + T2_STEP_LABELS[i]);
                t2Steps[i].setTextColor(getColor(R.color.badge_success));
                t2Steps[i].setTypeface(null, Typeface.BOLD);
            } else if (stepNum == activeStep) {
                t2Steps[i].setText("▶ " + T2_STEP_LABELS[i]);
                t2Steps[i].setTextColor(getColor(R.color.task2_blue));
                t2Steps[i].setTypeface(null, Typeface.BOLD);
            } else {
                t2Steps[i].setText(T2_STEP_LABELS[i]);
                t2Steps[i].setTextColor(getColor(R.color.text_secondary));
                t2Steps[i].setTypeface(null, Typeface.NORMAL);
            }
        }
    }

    private void resetT2Views() {
        resetT2Timer();
        updateT2StateBadge("IDLE");
        t2Obs1Arrow.setText(R.string.waiting_label);
        t2Obs1Arrow.setTextColor(getColor(R.color.text_disabled));
        t2Obs1Details.setText("Symbol: -- | Conf: --");
        t2Obs2Arrow.setText(R.string.waiting_label);
        t2Obs2Arrow.setTextColor(getColor(R.color.text_disabled));
        t2Obs2Details.setText("Symbol: -- | Conf: --");
        t2StepTitle.setText("Pipeline: Ready");
        updateT2PipelineStep(0);
        t2SensorText.setText("Ultrasonic: -- cm");
        t2ManeuverText.setText("Maneuver: Ready");
    }

    private String formatSensorDist(float cm) {
        if (Float.isInfinite(cm) || Float.isNaN(cm) || cm <= 0f || cm > 500f) {
            return "--";
        }
        return String.format(Locale.US, "%.1f", cm);
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

        // Fallback resilience for mixed text logs or CLI echoes
        Matcher mArrow = OBS_ARROW_PATTERN.matcher(text);
        if (mArrow.find()) {
            try {
                int obsNum = Integer.parseInt(mArrow.group(1));
                String dir = mArrow.group(2).toUpperCase(Locale.US);
                int sym = mArrow.group(3) != null ? Integer.parseInt(mArrow.group(3)) : (dir.equals("LEFT") ? 39 : 38);
                float conf = mArrow.group(4) != null ? Float.parseFloat(mArrow.group(4)) : 0.95f;
                onT2Arrow(obsNum, dir, sym, conf);
            } catch (Exception ignored) {}
        }

        Matcher mT2Comp = T2_COMPLETE_PATTERN.matcher(text);
        if (mT2Comp.find()) {
            float t = 0f;
            try {
                if (mT2Comp.group(1) != null) t = Float.parseFloat(mT2Comp.group(1));
            } catch (Exception ignored) {}
            onT2State("COMPLETE", 8, "Sprint Completed", t);
        }

        Matcher mT1Comp = T1_COMPLETE_PATTERN.matcher(text);
        if (mT1Comp.find()) {
            updateT1StateBadge("COMPLETED");
            stopT1Timer();
        }

        Matcher mFsm = FSM_TRANSITION_PATTERN.matcher(text);
        if (mFsm.find()) {
            updateT1StateBadge(mFsm.group(1));
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
        String entry = String.format(Locale.US, "Obs %d: #%d", obstacleId, symbolId);
        t1RecognizedTargets.put(obstacleId, entry);
        t1TargetsText.setText("🎯 Targets (" + t1RecognizedTargets.size() + "): " + TextUtils.join(" | ", t1RecognizedTargets.values()));
    }

    @Override
    public void onT1State(String state, int currentLeg, int totalLegs, int obsId, String face, float remDistCm) {
        updateT1StateBadge(state);
        if (totalLegs > 0) {
            String obsStr = (obsId > 0) ? ("Obs " + obsId + " (" + face + ")") : "--";
            t1LegText.setText(String.format(Locale.US, "Leg %d/%d (%s) | Rem: %.0f cm",
                    currentLeg, totalLegs, obsStr, remDistCm));
        } else {
            t1LegText.setText("Leg: Ready | Target: --");
        }

        if (state.equalsIgnoreCase("MISSION_COMPLETE") || state.equalsIgnoreCase("DONE") || state.equalsIgnoreCase("COMPLETED")) {
            stopT1Timer();
        } else if (state.equalsIgnoreCase("ESTOP") || state.equalsIgnoreCase("IDLE")) {
            stopT1Timer();
        } else if (!t1TimerRunning && (state.equalsIgnoreCase("NAVIGATING") || state.equalsIgnoreCase("SAMPLING_TARGET") || state.equalsIgnoreCase("ORBIT_RECOVERY"))) {
            startT1Timer();
        }
    }

    @Override
    public void onT1Target(int obsId, int symbolId, String symbolName, float confidence, String face) {
        arenaView.setRecognizedSymbol(obsId, symbolId);
        String entry = String.format(Locale.US, "Obs %d (%s): %s (#%d, %.0f%%)",
                obsId, face, symbolName, symbolId, confidence * 100f);
        t1RecognizedTargets.put(obsId, entry);
        t1TargetsText.setText("🎯 Targets (" + t1RecognizedTargets.size() + "): " + TextUtils.join(" | ", t1RecognizedTargets.values()));
    }

    @Override
    public void onT2State(String state, int stepIdx, String stepDesc, float elapsedSec) {
        updateT2StateBadge(state);
        t2StepTitle.setText("Pipeline: " + stepDesc);
        updateT2PipelineStep(stepIdx);
        t2ManeuverText.setText("Maneuver: " + stepDesc);

        if (state.equalsIgnoreCase("COMPLETE") || state.equalsIgnoreCase("DONE") || state.equalsIgnoreCase("COMPLETED") || stepIdx >= 8) {
            stopT2Timer();
            if (elapsedSec > 0f) {
                t2TimerText.setText(String.format(Locale.US, "%.2fs", elapsedSec));
            }
        } else if (state.equalsIgnoreCase("IDLE")) {
            resetT2Timer();
        } else if (state.equalsIgnoreCase("ESTOP")) {
            stopT2Timer();
        } else if (!t2TimerRunning && stepIdx >= 1 && stepIdx <= 7) {
            startT2Timer();
        }
    }

    @Override
    public void onT2Arrow(int obsNum, String direction, int symbolId, float confidence) {
        String dirUpper = direction.toUpperCase(Locale.US);
        boolean isLeft = dirUpper.contains("LEFT");
        String arrowDisplay = isLeft ? "⬅ LEFT" : "➡ RIGHT";
        int color = getColor(isLeft ? R.color.arrow_left : R.color.arrow_right);
        String details = String.format(Locale.US, "Symbol: %d | Conf: %.0f%%", symbolId, confidence * 100f);

        if (obsNum == 1) {
            t2Obs1Arrow.setText(arrowDisplay);
            t2Obs1Arrow.setTextColor(color);
            t2Obs1Details.setText(details);
        } else if (obsNum == 2) {
            t2Obs2Arrow.setText(arrowDisplay);
            t2Obs2Arrow.setTextColor(color);
            t2Obs2Details.setText(details);
        }
    }

    @Override
    public void onSensors(float usCm, float irLeftCm, float irRightCm) {
        String usStr = formatSensorDist(usCm);
        String irLStr = formatSensorDist(irLeftCm);
        String irRStr = formatSensorDist(irRightCm);

        t1SensorText.setText(String.format(Locale.US, "US: %s cm | IR: %s / %s cm", usStr, irLStr, irRStr));
        t2SensorText.setText(String.format(Locale.US, "Ultrasonic: %s cm", usStr));
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
        stopT1Timer();
        stopT2Timer();
        updateLinkStatus();
        showDevicePicker();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        joystickHandler.removeCallbacksAndMessages(null);
        linkStatusHandler.removeCallbacksAndMessages(null);
        t1TimerHandler.removeCallbacksAndMessages(null);
        t2TimerHandler.removeCallbacksAndMessages(null);
        linkService.shutdown();
    }
}
