#!/bin/bash
# One-time pairing helper -- run interactively on the Pi (SSH or console)
# once per new Android tablet. Put the tablet into Bluetooth
# pairing/discoverable mode first.
#
# BlueZ persists pairing bonds in /var/lib/bluetooth/ across reboots, so
# this only needs to run once per tablet, not once per boot -- contrast
# with bring-up-and-register.sh, which re-registers the SDP record every
# boot because that part does NOT persist.
#
# Each `bluetoothctl <cmd>` call below runs bluetoothctl in its one-shot,
# non-interactive mode (a single command, then exit) rather than piping a
# script into its interactive shell over stdin -- piping into interactive
# bluetoothctl makes it quit as soon as stdin closes, which would cut the
# scan off almost immediately instead of leaving it running.
set -e

bluetoothctl power on
bluetoothctl agent NoInputNoOutput
bluetoothctl default-agent
bluetoothctl pairable on
bluetoothctl discoverable on

echo "Scanning for 10s -- make sure the tablet is in pairing/discoverable mode now..."
timeout 10 bluetoothctl scan on || true

echo
echo "Devices seen:"
bluetoothctl devices

cat <<'MSG'

Find the tablet's MAC address above (its Bluetooth device name should be
recognizable), then run:

  bluetoothctl pair <MAC>
  bluetoothctl trust <MAC>
  bluetoothctl discoverable off

("discoverable off" is optional but recommended once every tablet you need
is paired -- a bonded/trusted device can still reconnect without the Pi
staying discoverable.)
MSG
