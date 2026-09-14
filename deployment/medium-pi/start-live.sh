#!/usr/bin/env bash
set -euo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
exec pixi run --frozen -e pi python -u "$BUNDLE/run_live.py" --ros-args \
  -r __node:=perception_medium_candidate \
  --params-file "$BUNDLE/parameters.yaml" "$@"
