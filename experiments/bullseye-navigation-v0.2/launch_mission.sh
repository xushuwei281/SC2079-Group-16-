#!/usr/bin/env bash
set -euo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
cd /home/mdp/dev/SC2079-Group-16/ros2_ws
# --enable-motion must be provided explicitly by the operator. No implicit reset.
exec pixi run --frozen -e pi python -u "$BUNDLE/run.py" mission "$@"
