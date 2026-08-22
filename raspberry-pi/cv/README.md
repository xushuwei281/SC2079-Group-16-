# CV pipeline — image recognition

Two inference targets, one training rig:

- **Edge (Raspberry Pi)** — self-contained, no Wi-Fi dependency, slower. YOLOv8n, exported to
  NCNN for CPU inference on the Pi.
- **Server (RTX 4060)** — higher accuracy, needs a Wi-Fi link back to the robot. YOLOv8m,
  served as a TensorRT fp16 engine.

Both train on the **A100 40GB (NSCC)** — the Pi and the 4060 are inference-only, neither
trains anything. All weights and the dataset move between machines via private HF repos
(`Frieddeli/mdp-symbols` for both the dataset and the model weights, distinguished by
`--repo-type`), not scp — see `scripts/` below.

## Dataset

[Yukto S&C](https://universe.roboflow.com/my-space-gprvy/yukto-s-c/dataset/61) (Roboflow),
version 61, downloaded and used **as-is** — 31 classes (`marker` + IDs 11–40), 8,106 images,
already split 6,948 / 811 / 347 (train/valid/test), YOLOv8 box format.

We deliberately did **not** try to de-duplicate the underlying source photos even though the
Roboflow "Images" pool (3,474) turned out to already contain rotated/grayscale/exposure
duplicate variants uploaded as if independent — perceptual-hash de-dup is unreliable against
exactly those transforms, and it's not worth the engineering time for a 10-week course
project. Instead:

- Training commands below **do not** add extra rotation (`degrees=0`) or push
  `hsv_s`/`hsv_v` above a mild level — the dataset already bakes in ±4° rotation and
  saturation/brightness/exposure jitter, so we're not compounding it with a second live
  augmentation pass.
- `fliplr=0.0` is set explicitly (and matches what's already baked into the Roboflow export):
  a mirrored left-arrow is a right-arrow, and a mirrored letter/digit is wrong or nonsensical.
- If the checklist demo shows misses under our actual arena lighting or robot camera, plan to
  top up with a small batch (50–100 images) of our own captures rather than re-collecting the
  whole set.

## Scripts

| Script | Runs on | Does |
|---|---|---|
| `scripts/pull_dataset.sh` | anywhere (needs `hf auth login`) | Downloads the dataset from HF, fixes the Roboflow `../` path bug in `data.yaml` |
| `scripts/train_edge.sh` | A100 (NSCC) | Trains YOLOv8n for the Pi target |
| `scripts/train_server.sh` | A100 (NSCC) | Trains YOLOv8m for the 4060 target |
| `scripts/export_edge.sh` | A100 (NSCC) | Exports NCNN + TFLite INT8, pushes to HF for the Pi to pull |
| `scripts/export_server_onnx.sh` | A100 (NSCC) | Exports ONNX (architecture-neutral), pushes to HF |
| `scripts/build_engine_4060.sh` | **the 4060 host itself** | Pulls the ONNX from HF, builds the TensorRT `.engine` — must run on the 4060, not the A100 (TensorRT engines are compute-capability-specific: A100 is sm_80, 4060 is sm_89) |
| `pi_infer.py` | the Pi | Multithreaded capture → infer (NCNN) → report loop skeleton |

## Setup

```bash
pip install -r requirements.txt
hf auth login   # once per machine
```
