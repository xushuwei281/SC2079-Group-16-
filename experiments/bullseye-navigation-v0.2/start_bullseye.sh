#!/usr/bin/env bash
# One entry point, usable from any directory. Defaults to the complete mission.
set -euo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
WORKSPACE=/home/mdp/dev/SC2079-Group-16/ros2_ws
if [[ ! -x "$WORKSPACE/.pixi/envs/pi/bin/python" ]]; then
  echo "The existing Pi ROS environment is missing: $WORKSPACE/.pixi/envs/pi" >&2
  exit 2
fi
cd "$WORKSPACE"
exec pixi run --frozen -e pi python -u "$BUNDLE/start_task.py" "$@"
