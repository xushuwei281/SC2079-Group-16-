#!/usr/bin/env bash
# Manual, foreground detector only: no camera, router, mission or boot service.
set -euo pipefail
if [[ $# != 4 ]]; then
  echo "Usage: $0 ROS_WORKSPACE nano|medium MODEL_ONNX NEW_SESSION_DIRECTORY" >&2
  exit 2
fi
workspace="$1"
profile="$2"
model="$3"
session="$4"
for path in "$workspace" "$model" "$session"; do
  if [[ "$path" != /* ]]; then
    echo "Use absolute paths for workspace, model and session directory: $path" >&2
    exit 2
  fi
done
bundle="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
if [[ ! -f "$workspace/pixi.toml" ]]; then
  echo "Missing ROS workspace manifest: $workspace/pixi.toml" >&2
  exit 1
fi
exec pixi run --frozen --manifest-path "$workspace/pixi.toml" -e pi \
  python -u "$bundle/candidate.py" start --profile "$profile" \
  --model "$model" --output "$session"
