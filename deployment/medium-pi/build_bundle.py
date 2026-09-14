#!/usr/bin/env python3
"""Build the medium stationary candidate using the existing live detector source."""
import argparse
import hashlib
import runpy
import shutil
from pathlib import Path


def build(output):
    templates = Path(__file__).resolve().parent
    nano_builder = templates.parent / 'live-pi/build_bundle.py'
    runpy.run_path(str(nano_builder))['build'](output)
    for source, dest in [('start-live.sh', 'start-live.sh'),
                         ('parameters.yaml', 'parameters.yaml'),
                         ('README.md', 'README.md'),
                         ('model-reference.json', 'model-reference.json')]:
        shutil.copy2(templates / source, output / dest)
    (output / 'start-live.sh').chmod(0o755)
    shutil.copy2(templates / 'test_live_worker.py', output / 'package/test/test_live_worker.py')
    files = sorted(p for p in output.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
    (output / 'SHA256SUMS').write_text(''.join(
        f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(output)}\n'
        for p in files))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    build(parser.parse_args().output.resolve())
