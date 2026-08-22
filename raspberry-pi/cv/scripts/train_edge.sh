#!/usr/bin/env bash
# Track A — trains the model that will be deployed to the Raspberry Pi.
# Run on the A100 (NSCC). The Pi never trains anything, it only runs the
# exported/quantised weights (see export_edge.sh).
set -euo pipefail

DATA="${1:-./data/data.yaml}"

# fliplr=0.0 — a mirrored left-arrow is a right-arrow, a mirrored letter/digit
# is wrong or nonsensical; never flip this label set.
#
# degrees=0, hsv_s=0.3, hsv_v=0.2 — the Yukto S&C export already bakes in
# ±4° rotation and saturation/brightness/exposure jitter. Adding Ultralytics'
# own rotation/color augmentation on top just compounds the same transforms,
# it doesn't add real new variety.
#
# batch=128 — the A100's headroom isn't the constraint for a nano model;
# this mainly buys faster, more stable hyperparameter sweeps.
yolo detect train \
  model=yolov8n.pt \
  data="$DATA" \
  imgsz=640 epochs=180 patience=30 \
  batch=128 fliplr=0.0 degrees=0 hsv_s=0.3 hsv_v=0.2 amp=True \
  device=0 project=runs/edge name=yolov8n_mdp
