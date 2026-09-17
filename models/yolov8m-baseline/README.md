# YOLOv8m full-baseline ONNX

`best.onnx` is the verified medium model from run
`baseline-yolov8m-v61-full-20260911T013018Z`. The exact exported file is tracked
with Git LFS. It is a stationary-test candidate; selecting this file does not
change the default detector or deploy it to a robot.

## Provenance and identity

| Item | Value |
| --- | --- |
| Architecture | YOLOv8m, initialized from `yolov8m.pt` |
| Training job | [Hugging Face job 6aa3621921047bf1b03747ec](https://huggingface.co/jobs/xxshuwei/6aa3621921047bf1b03747ec) |
| Completed / selected epoch | 118 / 88, with early stopping |
| File size | 103,694,827 bytes (98.891 MiB) |
| SHA-256 | `37329aa47f45f324e97945a8e85dea23b59e41c29ab1594aec894daf62aef031` |
| Input | Float32 RGB tensor `[1, 3, 640, 640]` |
| Output | `[1, 35, 8400]`, raw detection predictions |
| Export | ONNX opset 18, FP32, static batch 1, no embedded NMS |
| Class order | Strings `11` through `40`, followed by `marker` |

The cloud artifact is available to authorized users at:

```text
hf://buckets/xxshuwei/jobs-artifacts/baseline-yolov8m-v61-full-20260911T013018Z/runs/detect/baseline-yolov8m-v61-full-20260911T013018Z/weights/best.onnx
```

The [original export manifest](../../deployment/medium-pi/model-reference.json)
contains the training metrics, PyTorch checkpoint checksums and export parity
results. Its deployment-status field records the state at cloud export. See the
[later Pi validation records](../../docs/validation/medium-pi-20260914) for
subsequent deployment and saved-image replay evidence.

`models/best.onnx`, `models/best.pt`, `models/data.yaml` and `models/mapping.json`
are older artifacts with a different class convention. They do not describe this
medium model. Use its embedded class names with this branch's
`mdp_perception` class-contract loader. Class `40` is the official circle;
`marker` is a separate class and is filtered from accepted target messages.

## Retrieve and verify the actual model

Install Git LFS before downloading this branch. On macOS with Homebrew:

```bash
brew install git-lfs
git lfs install
```

After the branch has been pushed, use these commands from a checkout of
`codex/medium-baseline-candidate`:

```bash
git lfs pull --include="models/yolov8m-baseline/best.onnx" --exclude=""
git lfs fsck
cd models/yolov8m-baseline
shasum -a 256 -c SHA256SUMS
```

On the Raspberry Pi/Linux, use `sha256sum -c SHA256SUMS` for the last command.
It must report `best.onnx: OK`. The downloaded file must be 103,694,827 bytes.
If it is only a few lines starting with `version https://git-lfs.github.com/spec/v1`,
it is an LFS pointer; run `git lfs pull` before loading it with ONNX Runtime.
GitHub's source ZIP may contain a pointer depending on repository settings;
use a Git LFS checkout and verify the checksum.

Git LFS stores the model content separately from its small pointer in Git
history. The normal `git push` hook uploads the model content along with this
branch. Do not bypass the LFS hook or upload only the pointer. See
[GitHub's Git LFS documentation](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage).

## Use on the Raspberry Pi

The already transferred candidate is byte-identical and remains at:

```text
/home/mdp/mdp-cv/candidates/baseline-yolov8m-v61-full-20260911T013018Z/best.onnx
```

Follow the [medium stationary-test guide](../../deployment/medium-pi/README.md)
for its runtime bundle, camera/router setup, motor preparation, confirmation
timing and result topic. That guide's default profile already selects the
verified candidate above.

If using the file from a Git LFS checkout at
`/home/mdp/dev/SC2079-Group-16` instead, first verify it as described above, then
start the existing medium bundle with an explicit path override:

```bash
/home/mdp/mdp-cv/live-runtime/medium-v1-20260914/start-live.sh \
  -p model_path:=/home/mdp/dev/SC2079-Group-16/models/yolov8m-baseline/best.onnx
```

The runtime handles resizing/letterboxing, RGB conversion, normalization,
non-maximum suppression and class mapping. Use the medium timing profile:
on this Pi, saved-image ROS replay achieved about 0.20 images/second, with
median inference 4.38 seconds and first confirmed F after 16.65 seconds.
Three-frame confirmation therefore needs a steadily held card, not a
4-FPS expectation. The existing black-on-white card setup uses frame inversion.

## Validation limits

This exact ONNX passed graph, shape and class checks; cloud export matched
PyTorch on six images. The Pi benchmark and ROS replay passed digit 1, F,
background rejection, duplicate suppression, reappearance and obstacle-ID reset.
The evidence is a saved-image replay, not a new physical camera test.

The original dataset still contains split leakage, conflicting annotations and
unverified empty labels. Its high validation/test metrics are diagnostic rather
than a clean independent accuracy estimate. Broader physical tests remain
necessary before choosing this model for moving-robot operation. The nano
profile remains available for comparison.

Training checkpoints (`best.pt`, `last.pt`), epoch checkpoints and plots remain
in the Hugging Face run and workstation archive. They are not required for ONNX
inference and are not duplicated in this model package.
