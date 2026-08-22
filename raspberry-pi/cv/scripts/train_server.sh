#!/usr/bin/env bash
# Track B — trains the model that will be served from the RTX 4060 for
# inference. Run on the A100 (NSCC). The 4060 never trains anything, it
# only serves the exported TensorRT engine (see export_server_onnx.sh +
# build_engine_4060.sh).
set -euo pipefail

DATA="${1:-./data/data.yaml}"

# Same reasoning as train_edge.sh: fliplr=0.0, degrees=0, and reduced hsv
# jitter to avoid compounding the augmentation already baked into the
# Yukto S&C export. Bigger batch/imgsz/model than the edge track since the
# A100 can afford it and the 4060 only has to serve the result, not train it.
yolo detect train \
  model=yolov8m.pt \
  data="$DATA" \
  imgsz=768 epochs=150 patience=30 \
  batch=96 fliplr=0.0 degrees=0 hsv_s=0.3 hsv_v=0.2 amp=True \
  device=0 project=runs/server name=yolov8m_mdp

# Optional: train yolov8l.pt alongside as an accuracy ceiling. The A100 can
# afford it even if it's never deployed — useful to sanity-check how much
# the 'm' export is leaving on the table.
