#!/usr/bin/env bash
# Run this ON THE 4060 HOST ONLY — not on the A100. TensorRT engines are
# compiled for a specific compute capability; a .engine built on the A100
# (Ampere, sm_80) will not run on the 4060 (Ada Lovelace, sm_89). Pulls the
# architecture-neutral ONNX export from HF and builds the device-matched
# engine locally.
set -euo pipefail

hf download Frieddeli/mdp-symbols yolov8m_mdp/best.onnx \
  --repo-type model --local-dir ./weights

yolo export model=./weights/yolov8m_mdp/best.onnx \
  format=engine half=True imgsz=640 device=0

echo "Engine built at ./weights/yolov8m_mdp/best.engine"
