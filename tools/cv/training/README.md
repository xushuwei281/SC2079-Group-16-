# CV training tools

Read [the complete script-by-script workflow](../../../docs/cv/training.md) before running a
training command. These tools preserve input snapshots and require fresh output names. No script
submits paid Jobs, promotes a model, changes ROS defaults or operates the robot automatically.

For a CPU-only sanity check in an environment with `requirements-audit.txt` installed:

```bash
python -m unittest discover -s tools/cv/training -p 'test_*.py' -v
python tools/cv/training/train_baseline.py --help
```

`--dry-run` performs training preflight only. Actual training requires the separate pinned training
dependencies and explicit dataset/output/run arguments. Export and real-image parity are separate
steps. Requirements record observed working package versions; they are not a complete environment
lock. Retain every run's generated environment and provenance record.
