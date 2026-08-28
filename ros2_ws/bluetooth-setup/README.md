# Bluetooth setup (Pi side)

OS-level provisioning for the classic Bluetooth SPP link `android_bridge_node`
(package `mdp_android_bridge`) uses to talk to the Android tablet. This is
plain shell + systemd, not part of the ROS2/pixi workspace — it configures
the Pi's BlueZ stack, which `pixi` doesn't manage.

`android_bridge_node` opens `/dev/rfcomm0` with `pyserial`, same as
`serial_bridge_node` does for the STM32's USB-serial port — see
`../ARCHITECTURE.md`. Three things have to be true before that open()
succeeds: the adapter is on, the tablet is paired, and something is
listening for an incoming RFCOMM connection and binding it to
`/dev/rfcomm0` when one arrives. This directory covers all three,
persistently, so none of it depends on a human being logged into the Pi.

**Note on `rfcomm bind` vs `rfcomm watch`:** an earlier note in
`ARCHITECTURE.md` suggested `sudo rfcomm bind 0 <MAC> 1`. `rfcomm bind` is
for the opposite role — it has *this* machine act as the RFCOMM client,
connecting out to a known remote device's channel. Here the **Android app**
is the one calling `BluetoothSocket.connect()`; the Pi has to be the
*server*, accepting an incoming connection on the channel its SDP record
advertises. The right tool for that is `rfcomm watch` (see
`rfcomm-listen.sh`), not `rfcomm bind`.

## One-time setup (per Pi)

```bash
sudo apt install bluez
```

**Confirmed needed on the team's actual Pi (BlueZ 5.82):** modern
`bluetoothd` no longer exposes the legacy SDP socket `sdptool` talks to
unless started with `--compat`. Without this, `sdptool add`/`browse` fail
with `Failed to connect to SDP server ...: No such file or directory`.
Install the override, then reload:

```bash
sudo mkdir -p /etc/systemd/system/bluetooth.service.d
sudo cp systemd/bluetooth-compat-override.conf /etc/systemd/system/bluetooth.service.d/override.conf
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
bluetoothctl power on   # service restart drops adapter power state
```

Also confirmed: even in `--compat` mode, the SDP socket
(`/var/run/sdp` / `/run/sdp`) is `root:root`, mode `660` — `sdptool` only
works as root (`sudo sdptool ...`). `bring-up-and-register.sh` runs as root
under systemd already (no `User=` override in its unit), so this is only a
gotcha when testing `sdptool` by hand as a non-root user.

**Also confirmed needed:** the Bluetooth radio came up soft
rfkill-blocked (`/sys/class/rfkill/rfkillN/soft == 1` for the `hci0`
entry) — `bluetoothctl power on` fails with `org.bluez.Error.Failed` until
that's cleared. `bring-up-and-register.sh` clears it automatically (only
ever a *soft* block, safe to auto-clear — see the script); this note is
here so a manual `bluetoothctl power on` that mysteriously fails doesn't
send you down the wrong path.

## One-time setup (per Android tablet)

```bash
./pair-agent.sh
```

Put the tablet into Bluetooth pairing/discoverable mode first. The script
lists nearby devices; once you see the tablet's MAC address, Ctrl+C the scan
and run the two commands it prints (`bluetoothctl pair <MAC>` then
`bluetoothctl trust <MAC>`). BlueZ persists the bond in
`/var/lib/bluetooth/`, so this survives reboots — no need to re-pair unless
the tablet itself forgets the Pi. You also need to pair from the tablet's
own Bluetooth settings — pairing is mutual, not just something the Pi side
does.

## Persistent setup (every boot, no human needed)

Both systemd units invoke their scripts from a fixed path
(`/opt/mdp/bluetooth-setup/`) rather than wherever this repo happens to be
checked out, so they keep working regardless of how/where the workspace is
cloned on the Pi:

```bash
sudo mkdir -p /opt/mdp/bluetooth-setup
sudo cp bring-up-and-register.sh rfcomm-listen.sh /opt/mdp/bluetooth-setup/
sudo chmod +x /opt/mdp/bluetooth-setup/bring-up-and-register.sh /opt/mdp/bluetooth-setup/rfcomm-listen.sh
sudo cp systemd/mdp-bluetooth-setup.service systemd/mdp-rfcomm-listen.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mdp-bluetooth-setup mdp-rfcomm-listen
```

- `mdp-bluetooth-setup.service` (oneshot) runs `bring-up-and-register.sh` at
  boot — powers the adapter on and re-registers the SPP SDP record. SDP
  records live in `bluetoothd`'s memory and do **not** survive a reboot the
  way pairing bonds do, so this has to run every time, not just once.
- `mdp-rfcomm-listen.service` (long-running, `Restart=always`) runs
  `rfcomm-listen.sh`, which loops `rfcomm watch /dev/rfcomm0 1` forever —
  each time the Android app disconnects, `rfcomm watch` exits and the loop
  immediately starts listening again for the next connection, so
  `android_bridge_node` never needs restarting just because the tablet
  dropped off and reconnected.
- `bluetooth-compat-override.conf` (systemd drop-in on the stock
  `bluetooth.service`) — see the compat-mode note above. Install this
  once; it's not one of the `mdp-*` units and doesn't need re-enabling.

## Verifying

```bash
which bluetoothctl sdptool rfcomm   # confirm all three are actually installed
sdptool browse local                # should list a "Serial Port" service, channel 1
systemctl status mdp-rfcomm-listen  # should be active (running)
ls -l /dev/rfcomm0                  # appears once a device actually connects
```

If `sdptool`/`rfcomm` are missing on your Pi OS image, this setup doesn't
work as written — some Debian derivatives have split/dropped the classic
`bluez` CLI tools. The fallback in that case is registering the SPP profile
over D-Bus (`org.bluez.ProfileManager1`) instead, which isn't implemented
here.
