#!/bin/bash
# Idempotent, run on every boot (see systemd/mdp-bluetooth-setup.service).
#
# Powers the adapter on and registers the Serial Port Profile (SPP, UUID
# 1101) SDP record on a fixed channel, so Android's
# createRfcommSocketToServiceRecord() can discover the channel
# android_bridge_node listens on (via /dev/rfcomm0 -- see rfcomm-listen.sh,
# which does the actual accept-and-bind). Unlike pairing bonds (persisted
# by BlueZ across reboots), SDP records live only in the running
# bluetoothd process's memory -- this must re-register on every boot, not
# just once.
set -e

CHANNEL="${MDP_RFCOMM_CHANNEL:-1}"

# Some Pi OS images boot with the Bluetooth radio soft rfkill-blocked
# (confirmed on the team's actual Pi: /sys/class/rfkill/rfkillN/soft == 1
# for the hci0 entry) -- `bluetoothctl power on` fails outright
# (org.bluez.Error.Failed) until that's cleared. The `rfkill` CLI isn't
# installed on this image, so clear it directly via sysfs. Only ever
# touches a *soft* block (software-set, safe to clear automatically) --
# never touches `hard`, which reflects a physical switch and has no sysfs
# write path anyway.
for f in /sys/class/rfkill/rfkill*/name; do
    dir="$(dirname "$f")"
    if [ "$(cat "$f")" = "hci0" ] && [ "$(cat "$dir/soft")" = "1" ]; then
        echo 0 > "$dir/soft"
    fi
done

bluetoothctl power on

# sdptool add has no built-in idempotency guard -- re-running it blindly
# would register a duplicate SP record every boot, so check first.
if ! sdptool browse local 2>/dev/null | grep -q "Serial Port"; then
    sdptool add --channel="${CHANNEL}" SP
fi
