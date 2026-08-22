"""Pi-side inference loop skeleton for Track A (edge).

Three threads sharing bounded queues:
  capture_thread -> frame_q  -> infer_thread -> result_q -> report_thread

report_thread is also responsible for buffering the raw (non-annotated)
frame behind every accepted detection, so it can be stitched into the grid
image required for live verification on Android/PC (see the Admin
briefing's "stitched RAW images" requirement) — a plain PIL paste-loop is
enough, no need for a real panorama stitch.

This is a structural skeleton, not a finished integration: the pieces that
depend on the rest of the system (the actual comms link to the algorithm
module, the Android/PC display channel, camera setup) are left as TODOs
for whoever owns that interface.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

CONF_THRESHOLD = 0.5
MODEL_PATH = "./weights/best_ncnn_model"


@dataclass
class Detection:
    obstacle_id: str
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    raw_frame: "object"  # numpy array — the frame the detection came from
    timestamp: float


def capture_thread(frame_q: "queue.Queue[object]", stop: threading.Event) -> None:
    # TODO: replace with the real camera source (e.g. picamera2).
    # Deliberately keeps only the latest frame — a full queue means the
    # consumer is behind, and stale frames aren't useful for a live loop.
    while not stop.is_set():
        frame = _capture_one_frame()
        if frame_q.full():
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
        frame_q.put(frame)


def infer_thread(
    frame_q: "queue.Queue[object]",
    result_q: "queue.Queue[Detection]",
    stop: threading.Event,
    obstacle_id_provider,
) -> None:
    model = _load_model(MODEL_PATH)
    while not stop.is_set():
        try:
            frame = frame_q.get(timeout=0.5)
        except queue.Empty:
            continue

        label, confidence, bbox = model.infer(frame)
        if confidence < CONF_THRESHOLD:
            # Below threshold: report as uncertain rather than guessing —
            # this is what triggers the algorithm module's
            # reverse-and-recheck fallback (see the Algorithms briefing,
            # "What if the image is not found").
            label = "UNCERTAIN"

        result_q.put(
            Detection(
                obstacle_id=obstacle_id_provider(),
                label=label,
                confidence=confidence,
                bbox=bbox,
                raw_frame=frame,
                timestamp=time.time(),
            )
        )


def report_thread(
    result_q: "queue.Queue[Detection]",
    stop: threading.Event,
    raw_frame_buffer: dict[str, object],
) -> None:
    while not stop.is_set():
        try:
            det = result_q.get(timeout=0.5)
        except queue.Empty:
            continue

        # TODO: send (obstacle_id, label, confidence) over the existing
        # RPi<->algorithm link.
        _send_result(det)

        # Buffer the raw frame per obstacle for the verification stitch —
        # only on a confident detection, not every frame.
        if det.label != "UNCERTAIN":
            raw_frame_buffer[det.obstacle_id] = det.raw_frame


def build_verification_stitch(raw_frame_buffer: dict[str, object]):
    """Tiles buffered raw frames into one grid image for the Android/PC
    verification display. Simple PIL paste loop — no real stitching needed.
    """
    from PIL import Image

    frames = list(raw_frame_buffer.values())
    if not frames:
        return None

    thumb_w, thumb_h = 160, 120
    cols = 4
    rows = (len(frames) + cols - 1) // cols
    grid = Image.new("RGB", (thumb_w * cols, thumb_h * rows))

    for i, frame in enumerate(frames):
        thumb = Image.fromarray(frame).resize((thumb_w, thumb_h))
        x, y = (i % cols) * thumb_w, (i // cols) * thumb_h
        grid.paste(thumb, (x, y))

    return grid


def _capture_one_frame():
    raise NotImplementedError("TODO: hook up the actual camera")


def _load_model(path: str):
    raise NotImplementedError("TODO: load the NCNN/TFLite model")


def _send_result(det: Detection) -> None:
    raise NotImplementedError("TODO: hook up the RPi<->algorithm link")


def main() -> None:
    frame_q: "queue.Queue[object]" = queue.Queue(maxsize=1)
    result_q: "queue.Queue[Detection]" = queue.Queue()
    raw_frame_buffer: dict[str, object] = {}
    stop = threading.Event()

    current_obstacle_id = "obstacle_0"  # TODO: driven by the algorithm module

    threads = [
        threading.Thread(target=capture_thread, args=(frame_q, stop), daemon=True),
        threading.Thread(
            target=infer_thread,
            args=(frame_q, result_q, stop, lambda: current_obstacle_id),
            daemon=True,
        ),
        threading.Thread(
            target=report_thread, args=(result_q, stop, raw_frame_buffer), daemon=True
        ),
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
        for t in threads:
            t.join(timeout=2)


if __name__ == "__main__":
    main()
