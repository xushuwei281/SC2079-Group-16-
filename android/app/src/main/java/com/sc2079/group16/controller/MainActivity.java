package com.sc2079.group16.controller;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ListView;
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

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLinkService linkService;

    private final List<BluetoothDevice> bondedDevices = new ArrayList<>();

    private ListView deviceListView;
    private View devicePickerPanel;
    private View controlPanel;
    private TextView connectedDeviceLabel;
    private TextView statusTextView;
    private EditText distanceInput;
    private EditText angleInput;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        deviceListView = findViewById(R.id.deviceList);
        devicePickerPanel = findViewById(R.id.devicePickerPanel);
        controlPanel = findViewById(R.id.controlPanel);
        connectedDeviceLabel = findViewById(R.id.connectedDeviceLabel);
        statusTextView = findViewById(R.id.statusText);
        statusTextView.setMovementMethod(new ScrollingMovementMethod());
        distanceInput = findViewById(R.id.distanceInput);
        angleInput = findViewById(R.id.angleInput);

        findViewById(R.id.refreshButton).setOnClickListener(v -> refreshDeviceList());
        findViewById(R.id.disconnectButton).setOnClickListener(v -> disconnect());
        findViewById(R.id.forwardButton).setOnClickListener(v -> sendMove("FW", distanceInput));
        findViewById(R.id.backwardButton).setOnClickListener(v -> sendMove("BW", distanceInput));
        findViewById(R.id.turnLeftButton).setOnClickListener(v -> sendMove("TL", angleInput));
        findViewById(R.id.turnRightButton).setOnClickListener(v -> sendMove("TR", angleInput));
        findViewById(R.id.stopButton).setOnClickListener(v -> linkService.sendLine("STP"));

        deviceListView.setOnItemClickListener(
                (parent, view, position, id) -> connectTo(bondedDevices.get(position)));

        linkService = new BluetoothLinkService(this);

        BluetoothManager btManager = getSystemService(BluetoothManager.class);
        bluetoothAdapter = btManager != null ? btManager.getAdapter() : null;

        ensurePermissionThenListDevices();
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

    private void sendMove(String kind, EditText valueInput) {
        String raw = valueInput.getText().toString().trim();
        appendStatus(kind + " pressed (input: \"" + raw + "\")");
        if (raw.isEmpty()) {
            appendStatus(kind + " not sent: no value entered");
            Toast.makeText(this, "Enter a value first", Toast.LENGTH_SHORT).show();
            return;
        }
        int value;
        try {
            value = Integer.parseInt(raw);
        } catch (NumberFormatException e) {
            appendStatus(kind + " not sent: \"" + raw + "\" is not a number");
            Toast.makeText(this, "Not a number", Toast.LENGTH_SHORT).show();
            return;
        }
        linkService.sendLine(kind + ":" + value);
    }

    private void showDevicePicker() {
        devicePickerPanel.setVisibility(View.VISIBLE);
        controlPanel.setVisibility(View.GONE);
    }

    private void showControlPanel() {
        devicePickerPanel.setVisibility(View.GONE);
        controlPanel.setVisibility(View.VISIBLE);
    }

    private void appendStatus(String text) {
        Log.i("StatusLog", text); // mirror to logcat -- `adb logcat -s StatusLog` for debugging
        statusTextView.append(text + "\n");
        final int scrollAmount = statusTextView.getLayout() == null
                ? 0
                : statusTextView.getLayout().getLineTop(statusTextView.getLineCount())
                        - statusTextView.getHeight();
        if (scrollAmount > 0) {
            statusTextView.scrollTo(0, scrollAmount);
        }
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
        showControlPanel();
    }

    @Override
    public void onStatusLine(String text) {
        appendStatus("STATUS: " + text);
    }

    @Override
    public void onDone() {
        appendStatus("DONE");
    }

    @Override
    public void onDebug(String line) {
        appendStatus(line);
    }

    @Override
    public void onDisconnected(String reason) {
        // Toast as well as appendStatus(): this can fire before a connection
        // ever succeeded (e.g. the initial connect attempt itself failing),
        // in which case controlPanel -- and its status log -- has never been
        // shown, so appendStatus() alone would be silently invisible.
        Toast.makeText(this, "Disconnected: " + reason, Toast.LENGTH_LONG).show();
        appendStatus("Disconnected (" + reason + ")");
        showDevicePicker();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        linkService.shutdown();
    }
}
