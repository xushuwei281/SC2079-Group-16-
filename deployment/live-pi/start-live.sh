#!/usr/bin/env bash
set -euo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
exec pixi run --frozen -e pi python -u "$BUNDLE/run_live.py" --ros-args   -p model_path:=/home/mdp/mdp-cv/candidates/baseline-yolov8n-v61-full-20260908T142129Z-r2/best.onnx   -p output_dir:=/home/mdp/mdp-cv/reports/live-v1-20260909   "$@"
