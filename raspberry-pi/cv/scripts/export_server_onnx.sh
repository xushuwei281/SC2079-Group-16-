#!/usr/bin/env bash
# Exports the Track B (server) weights to ONNX and pushes to HF. Run on the
# A100 job right after train_server.sh. ONNX is architecture-neutral — the
# TensorRT engine build (a separate, required step) must happen on the 4060
# host itself, NOT here: TensorRT engines are compiled for a specific compute
# capability, and an engine built on the A100 (Ampere, sm_80) will not run
# on the 4060 (Ada Lovelace, sm_89). See build_engine_4060.sh.
set -euo pipefail

WEIGHTS="${1:-runs/server/yolov8m_mdp/weights/best.pt}"

yolo export model="$WEIGHTS" format=onnx imgsz=640 simplify=True

ONNX_FILE="$(dirname "$WEIGHTS")/best.onnx"
hf upload Frieddeli/mdp-symbols "$ONNX_FILE" yolov8m_mdp/best.onnx \
  --repo-type model

echo "Pushed. On the 4060 host, run build_engine_4060.sh"
