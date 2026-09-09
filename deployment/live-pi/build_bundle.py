#!/usr/bin/env python3
"""Build the isolated Pi runtime from version-controlled sources; no model download."""

import argparse
import hashlib
import shutil
from pathlib import Path


def build(output):
    templates = Path(__file__).resolve().parent
    repo = templates.parents[1]
    source = repo / 'ros2_ws/src/mdp_perception'
    output.mkdir(parents=True, exist_ok=False)
    (output / 'package/mdp_perception').mkdir(parents=True)
    (output / 'package/test').mkdir()
    for name in ['__init__.py', 'class_contract.py', 'detector.py',
                 'live_confirmation.py', 'live_perception_node.py']:
        shutil.copy2(source / 'mdp_perception' / name, output / 'package/mdp_perception' / name)
    for name in ['test_class_contract.py', 'test_detector_contract.py', 'test_live_confirmation.py']:
        shutil.copy2(source / 'test' / name, output / 'package/test' / name)
    for name in ['run_live.py', 'start-live.sh', 'stop-live.sh']:
        shutil.copy2(templates / name, output / name)
    shutil.copy2(source / 'LIVE_TESTING.md', output / 'README.md')
    files = sorted(p for p in output.rglob('*') if p.is_file())
    (output / 'SHA256SUMS').write_text(''.join(
        f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(output)}\n'
        for p in files))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New destination directory; existing directories are rejected')
    args = parser.parse_args()
    build(args.output.resolve())
    print(args.output.resolve())
