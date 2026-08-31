# Flashing the C30D STM32F407 Board over UART

This project is configured to build and flash the WHEELTEC C30D v2.0/v2.1
motor-controller board through its onboard USB-to-UART converter. An ST-Link is
not required.

## Board connection and safety

Follow the SC2079 technical-material flashing guide:

1. Put the board on an insulated surface, not on its anti-static bag.
2. Switch off and disconnect the 12 V supply.
3. Disconnect the Raspberry Pi supply/connection.
4. Connect a USB Type-C cable to **Port 1**, the Type-C socket nearest the
   potentiometer.

Port 1 contains a CH9102F USB-to-UART bridge connected to STM32 USART1:

- CH9102F TXD -> STM32 PA10 (USART1 RX)
- CH9102F RXD <- STM32 PA9 (USART1 TX)
- DTR/RTS -> the onboard reset and BOOT0 one-key download circuit

Do not use Port 2 for flashing. Port 2 is connected to USART3 and is intended
for normal communication with the Raspberry Pi.

## PlatformIO configuration

The relevant settings are already in `platformio.ini`:

```ini
upload_protocol = serial
upload_port = /dev/ttyACM0
upload_flags =
    -i
    rts,-dtr,dtr,-rts:-rts,-dtr,dtr
```

The `-i` sequence drives the C30D board's CH9102F DTR/RTS circuit. It selects
the STM32 system-memory bootloader, pulses reset, and restores normal execution
after programming. There is no need to move a BOOT0 jumper manually.

## Build and flash

From this `stm32` directory, run:

```bash
platformio run --target upload
```

On success, `stm32flash` identifies device `0x0413`, writes the firmware from
address `0x08000000`, and reports output similar to:

```text
Device ID    : 0x0413 (STM32F40xxx/41xxx)
Wrote address ... (100.00%) Done.
Starting execution at address 0x08000000... done.
[SUCCESS]
```

After flashing:

1. Disconnect the USB cable from Port 1 if it is no longer needed.
2. Reconnect and switch on the 12 V supply.
3. Check that the MCU and IMU do not become unusually hot before testing the
   car.

## Troubleshooting

### Upload port changed

Linux device numbers can change after reconnecting or rebooting. List the
available onboard serial ports:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

If Port 1 is no longer `/dev/ttyACM0`, either update `upload_port` in
`platformio.ini` or override it for one upload:

```bash
platformio run --target upload --upload-port /dev/ttyACM1
```

When both C30D Type-C ports are connected, they appear as similar QinHeng
CH9102F devices. Port 1 must be identified by the physical USB connection; the
device assigned the lower number is not guaranteed to remain the same.

### `Failed to init device, timeout`

Check all of the following:

- The cable is connected to Port 1 nearest the potentiometer.
- The cable supports data, not charging only.
- No serial monitor or Raspberry Pi process has the port open.
- `upload_flags` still contains the exact DTR/RTS sequence shown above.
- The current user has permission to access the serial device (normally via
  membership in the `dialout` group).

To confirm that a candidate port reaches the USART1 bootloader without writing
flash, run:

```bash
~/.platformio/packages/tool-stm32flash/stm32flash \
  -b 115200 \
  -i 'rts,-dtr,dtr,-rts:-rts,-dtr,dtr' \
  /dev/ttyACM0
```

A correct Port 1 connection reports the STM32 device ID. A Port 2 connection
times out.

### PlatformIO cannot write its cache or lock files

PlatformIO stores tools and lock files under `~/.platformio`. Ensure the user
running the upload owns that directory and can write to it.

## Reference material

The board-specific source documents are located outside this repository at:

- `/home/mdp/dev/SC2079 Techincal Material/Technical Materials/STM32407/Flashing STM32 motor controller card for STM32F407VET6 C30D version.pdf`
- `/home/mdp/dev/SC2079 Techincal Material/Technical Materials/STM32407/STM32F407VET6(C30D-V2.1)_English.pdf`

The second document's page 2 schematic labels the circuit **UART1 one-key
download circuit**.
