# CV integration checklist (model trained → wired into the robot)

Assumes `train_edge.sh`/`train_server.sh` (`raspberry-pi/cv/`) has already
produced weights. This is what's left to get from "trained model" to
"robot reports recognized images during a run." Builds on
[`week2-checklist.md`](week2-checklist.md) (comms loop) and adds the
message format in [`protocol.md`](protocol.md#image-recognition-results-raspberry-pi--algorithm-pc--android).

| Task | Owner | Acceptance check |
|---|---|---|
| Export + pull weights onto the actual target — `export_edge.sh`/`build_engine_4060.sh`, then `hf download` on the Pi/4060 | RPi/CV owner | Model loads on-device without error (not just on the A100) |
| Wire `pi_infer.py`: replace `_capture_one_frame`, `_load_model`, `_send_result` TODOs with the real camera, real NCNN/TFLite load, and the `image_result`/`STATUS` messages from `protocol.md` | RPi/CV owner | A known printed symbol held in front of the camera produces a correct `STATUS,recognized <label> at <obstacle_id>` on Android |
| Algorithm PC: consume `image_result` messages, build the stitched grid (reuse the logic sketched in `pi_infer.py`'s `build_verification_stitch()` rather than re-deriving it) | Algorithm owner | Stitched image updates live as obstacles are recognized during a test run |
| Confidence threshold sanity pass — `CONF_THRESHOLD` in `pi_infer.py` is a placeholder (0.5) | RPi/CV owner | Deliberately show the camera a bull's-eye and an ambiguous/blurry frame — confirm `UNCERTAIN` fires instead of a wrong confident label, since that's what triggers the algorithm's reverse-and-recheck fallback |
| Real-camera accuracy check against the training validation numbers | RPi/CV owner | Run the model against ~20 frames captured live on the actual robot camera/lighting; if accuracy is meaningfully worse than the Roboflow validation split, that's the signal to do the "supplement with real captures" step already flagged in `raspberry-pi/cv/README.md` — don't wait until the Week 7 checklist to find out |
| Timing check against the real constraint | RPi/CV owner | Full recognition (capture → infer → report) comfortably fits inside the ~45s/obstacle budget (6 min ÷ up to 8 obstacles) — inference itself should be nowhere near the limit; if it is, that's a routing/link problem, not a model problem (see the "reality check" callout in the CV pipeline doc: neither track's inference latency was expected to be the bottleneck) |
| End-to-end test: robot approaches a real obstacle, recognizes it, reports over both channels, continues | Everyone | One full pass: Android shows the status text, PC shows the stitched image, and the run continues to the next obstacle without manual intervention |
