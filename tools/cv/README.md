# CV workflow tools

Start with the [workflow index](../../docs/cv/README.md).

| Tool family | Purpose | Documentation |
|---|---|---|
| `training/` | Audit/quarantine data, train a baseline, export ONNX and compare backends | [Training guide](../../docs/cv/training.md) and the directory README |
| `../../deployment/cv-baselines/` | Verify exact models, build isolated historical Pi bundles, review observations | [Pi testing](../../docs/cv/pi-testing.md) and deployment README |
| `check_documentation.py` | Check local file links in the added CV documentation | Usage below |

## Check documentation links

From the repository root, with Python 3.12 (standard library only):

```bash
python tools/cv/check_documentation.py
```

The default repository is derived from the script's location. To inspect another checkout use
`--repository /path/to/checkout`. It reads Markdown under the CV docs/tools/deployment/models and
historical experiment directories, skips network URLs and code fences, and checks whether local
link targets exist. It does not fetch URLs or validate heading anchors. It prints any missing
targets and returns exit status 1 for missing links, otherwise 0. It does not write files.

These tools do not launch a robot mission or submit a cloud job automatically. Live commands in
the documentation are for manual operator execution in a prepared environment.
