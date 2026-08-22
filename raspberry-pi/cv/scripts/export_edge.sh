#!/usr/bin/env bash
# Exports the Track A (edge) weights and pushes them to HF for the Pi to
# pull. Run this on the A100 job right after train_edge.sh — NCNN/TFLite
# are portable formats, so unlike the 4060's TensorRT engine, this is the
# only transfer step needed; no separate "build on target device" step.
set -euo pipefail

WEIGHTS="${1:-runs/edge/yolov8n_mdp/weights/best.pt}"
DATA="${2:-./data/data.yaml}"

# NCNN — fastest CPU-only path on a Pi 4/5
yolo export model="$WEIGHTS" format=ncnn imgsz=480

# TFLite INT8 — alternative if the team standardises on the Keras/TF stack
# taught in the official RPi briefing materials
yolo export model="$WEIGHTS" format=tflite int8=True data="$DATA"

NCNN_DIR="$(dirname "$WEIGHTS")/best_ncnn_model"
hf upload Frieddeli/mdp-symbols "$NCNN_DIR" yolov8n_mdp/best_ncnn_model \
  --repo-type model

echo "Pushed. On the Pi, pull with:"
echo "  hf download Frieddeli/mdp-symbols yolov8n_mdp/best_ncnn_model --repo-type model --local-dir ./weights"
