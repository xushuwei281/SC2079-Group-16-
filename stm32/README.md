# STM32 firmware

Robot movement and sensor firmware for the STM32F407VET6 controller board
("C30D" version). Ported from the course's `STM_Ref` reference project,
built with PlatformIO instead of STM32CubeIDE.

- Toolchain: PlatformIO (`platform = ststm32`, `framework = stm32cube`)
- Responsibilities: motor control, encoders, IMU, OLED, serial comms with
  the Raspberry Pi (see `../ros2_ws/ARCHITECTURE.md` for the
  UART protocol this firmware speaks)

## Build

```
pio run
```

Builds for all supported hosts (macOS, Linux x86_64, Linux aarch64 —
including running directly on a Raspberry Pi). `platformio.ini` pins a
newer ARM toolchain version than the `ststm32` platform's default,
because the default range has no `linux_aarch64` build.

## Flash

The board flashes over UART, through a USB-serial chip built into the
board itself (no external programmer). The port shows up as
`/dev/ttyACM0` on Linux (or `/dev/tty.usbserial-*` on macOS) once
connected.

```
stm32flash -i "rts,-dtr,dtr,-rts" -w .pio/build/genericSTM32F407VET6/firmware.bin -v -g 0x0 /dev/ttyACM0
```

No button presses needed. The board has a one-key auto-download circuit
(see `Technical Materials/STM32407/STM32F407VET6(C30D-V2.1)_English.pdf`,
sheet 2, "UART1 one-key download circuit"): the USB-serial chip's RTS
line drives BOOT0 through a transistor, and its DTR line drives NRST
through another. The `-i` flag above reproduces the standard sequence
(RTS high, DTR pulse low then high, RTS low), which puts the chip into
its ROM bootloader and resets it, entirely over the software serial
connection.

`stm32flash` ships with PlatformIO's `ststm32` platform once you've run
`pio run -t upload` at least once (it lives at
`~/.platformio/packages/tool-stm32flash/stm32flash`), or install it
separately (`apt install stm32flash` on Debian/Raspberry Pi OS,
`brew install stm32flash` on macOS).

### Why not the ST-Link header?

The board also has an SWD header, and `platformio.ini` still has
`upload_protocol = stlink` set. In practice this didn't work reliably in
testing (the ST-Link connection dropped intermittently regardless of
cable, port, or host machine), and the course's own documentation never
covers ST-Link for this board at all, only the UART method above. Use
`pio run -t upload` only if you've got a working ST-Link setup verified
some other way; otherwise use the `stm32flash` command directly.

### Two-wire alternative: FlyMcu

The course's own guide
(`Technical Materials/STM32407/Flashing STM32 motor controller card for STM32F407VET6 C30D version.pdf`)
documents flashing with FlyMcu (Windows only) instead of `stm32flash`.
Same underlying protocol and same auto-download circuit, different tool.
That guide also covers power sequencing: disconnect the 12V supply
before flashing, USB power only, reconnect 12V afterward.
