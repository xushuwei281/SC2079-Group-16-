# Verified historical nano and medium ONNX baselines

These are the exact candidate files used by the recorded September 2026 CV workflow. Both are
stored using Git LFS. Adding them does not change `models/best.onnx`, the NCNN model, or any robot
launch default.

| Model | Path | Bytes | Best epoch | SHA-256 |
|---|---|---:|---:|---|
| YOLOv8n | [`nano/best.onnx`](nano/best.onnx) | 12,288,971 | 99 | `0207a7fb6ab117ce0a62fd683185da8d8ca065c92f1a0ee1d155e65f14e2c48a` |
| YOLOv8m | [`medium/best.onnx`](medium/best.onnx) | 103,694,827 | 88 | `37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031` |

Each directory includes a `manifest.json` and `SHA256SUMS`. Both graphs have FP32 input
`[1,3,640,640]`, raw output `[1,35,8400]`, static batch one, opset 18 and no embedded NMS.
Class order is strings `11` through `40`, then `marker`. The matching runtime performs
letterboxing, RGB conversion, normalization, NMS and official-ID mapping.

## Retrieve and verify

From the repository root:

```bash
git lfs pull --include="models/cv-baselines/*/best.onnx" --exclude=""
git lfs fsck
cd models/cv-baselines/nano
sha256sum -c SHA256SUMS
cd ../medium
sha256sum -c SHA256SUMS
```

On macOS use `shasum -a 256 -c SHA256SUMS` in each model directory. A model containing a few text
lines starting with the Git LFS pointer header has not been downloaded. Do not pass a pointer to
ONNX Runtime. Downloading a source ZIP is not a substitute for checking LFS content and hashes.

Use the [isolated candidate tooling](../../deployment/cv-baselines/) and
[Pi testing instructions](../../docs/cv/pi-testing.md) for the matching timing/profile settings.
Use [results](../../docs/cv/results.md) to decide which experiment to reproduce: nano achieved
approximately 1 FPS in the physical F test; medium approximately 0.20 FPS in saved-image Pi replay.

## Provenance and limitations

- Nano job: https://huggingface.co/jobs/xxshuwei/6aa027d5900620b5c77e29ad
- Medium job: https://huggingface.co/jobs/xxshuwei/6aa3621921047bf1b03747ec
- Raw original split retained for full baselines; leakage/annotation/provenance issues remain.
- Nano physical evidence covers digit 1, F and one background, not all classes.
- Medium evidence is saved-image replay, not a fresh physical camera session.
- `best.pt`, `last.pt`, periodic checkpoints and training plots are retained in the original
  Hugging Face/workstation archives. They are not required to run these ONNX models.
- A previous nano-named cloud output prefix was replaced with medium artifacts after backup.
  Identify artifacts using the manifest and SHA, not a cloud directory name alone.

These manifests describe historical model artifacts. They do not certify current Pi installation,
active process state, calibration, dataset licence redistribution rights or mission readiness.
