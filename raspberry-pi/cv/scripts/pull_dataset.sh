#!/usr/bin/env bash
# Pulls the Yukto S&C dataset (already uploaded to HF) and fixes the
# Roboflow export's data.yaml, which points train/val/test one directory
# above where the images actually are (`../train/images` etc). Ultralytics
# resolves those paths relative to data.yaml's own directory when no `path:`
# key is set, so as exported they point outside the dataset folder entirely.
set -euo pipefail

DEST="${1:-./data}"

hf download Frieddeli/mdp-symbols --repo-type dataset --local-dir "$DEST"

sed -i.bak -E 's#^(train|val|test): \.\./#\1: #' "$DEST/data.yaml"
rm -f "$DEST/data.yaml.bak"

echo "Dataset ready at $DEST"
echo "---"
cat "$DEST/data.yaml"
