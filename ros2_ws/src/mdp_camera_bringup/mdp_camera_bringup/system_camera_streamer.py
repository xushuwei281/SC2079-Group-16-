#!/usr/bin/env python3
"""Hardware-ISP Camera Daemon running on system Python (Picamera2).

Streams pristine 640x480 RGB8 frames from Sony IMX219 via Broadcom VideoCore ISP
into shared memory (/dev/shm/mdp_camera.raw).
"""

import mmap
import os
import sys
import time

try:
    from picamera2 import Picamera2
except ImportError:
    print("[CameraDaemon] Error: Picamera2 is required in system Python.")
    sys.exit(1)

SHM_PATH = "/dev/shm/mdp_camera.raw"
FRAME_SIZE = 640 * 480 * 3  # 921,600 bytes


def main():
    # Pre-allocate shared memory file
    with open(SHM_PATH, "wb") as f:
        f.write(b"\x00" * FRAME_SIZE)

    shm_fd = os.open(SHM_PATH, os.O_RDWR)
    shm = mmap.mmap(shm_fd, FRAME_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "BGR888"}
    )
    picam2.configure(config)
    try:
        picam2.set_controls({
            "AwbEnable": True,
            "AeEnable": True,
            "Saturation": 1.0,
            "Contrast": 1.0,
            "Sharpness": 1.0,
        })
    except Exception as e:
        print(f"[CameraDaemon] Control warning: {e}")
    picam2.start()

    time.sleep(1.5)  # Let AEC/AWB fully converge
    print("[CameraDaemon] Picamera2 Hardware ISP streaming active (BGR888, natural color balance).")

    try:
        while True:
            # Capture directly into numpy array and copy into shared memory
            frame = picam2.capture_array()
            shm.seek(0)
            shm.write(frame.tobytes())
            time.sleep(0.033)  # ~30 FPS
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        picam2.close()
        shm.close()
        os.close(shm_fd)


if __name__ == "__main__":
    main()
