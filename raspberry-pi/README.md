# Raspberry Pi

Comms hub between Android, STM32, and the Algorithm PC, plus camera capture
and image recognition.

- Language: Python
- Responsibilities: Bluetooth link to Android, serial link to STM32, camera
  capture, running/calling the image recognition model, forwarding
  path/obstacle data to the Algorithm module

This is the **non-ROS track** — raw Bluetooth/UART/TCP sockets, speaking
[`../docs/protocol.md`](../docs/protocol.md). The parallel ROS 2 track moved
to [`../ros2_ws/`](../ros2_ws/) (top level, since its graph spans the Pi and
a laptop rather than living on the Pi alone).

## Setup

TODO: virtualenv / requirements.txt, how to run on the Pi.
