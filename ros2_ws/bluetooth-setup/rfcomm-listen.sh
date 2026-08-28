#!/bin/bash
# Persistent RFCOMM server for android_bridge_node's /dev/rfcomm0 link.
#
# `rfcomm bind <dev> <bdaddr> <channel>` (mentioned in an earlier draft of
# ARCHITECTURE.md's android_bridge_node notes) is for the opposite role: it
# has this machine act as the RFCOMM *client*, connecting out to a known
# remote device's channel. Here it's the Android app that calls
# BluetoothSocket.connect() -- the Pi has to be the *server*, accepting an
# incoming connection on the channel advertised by the SDP record
# bring-up-and-register.sh registers. The right BlueZ tool for that role is
# `rfcomm watch`, not `rfcomm bind`.
#
# `rfcomm watch` returns once a connection is accepted (or the link drops),
# so this wraps it in a restart loop -- systemd's Restart=always achieves
# the same thing at the unit level, but looping here too means this script
# also behaves correctly if ever run by hand outside systemd.
set -e

CHANNEL="${MDP_RFCOMM_CHANNEL:-1}"
DEVICE="${MDP_RFCOMM_DEVICE:-0}"

rfcomm release "${DEVICE}" >/dev/null 2>&1 || true

while true; do
    rfcomm watch "${DEVICE}" "${CHANNEL}" || true
    sleep 1
done
