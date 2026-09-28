#!/usr/bin/env python3
"""Regression on previously saved Pi images; not a new physical camera test."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cv2
from bullseye.symbols import SymbolReader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reader = SymbolReader(args.model, args.sha256, threads=2, invert=True)
    records = []
    for filename, expected in [('digit1.png', 11), ('letter-f.png', 25), ('background.png', None)]:
        frame = cv2.imread(str(args.cases/filename))
        if frame is None:
            raise ValueError('Missing saved case: '+filename)
        started = time.monotonic()
        detections = reader.predict(frame)
        symbols = [x['symbol_id'] for x in detections if x['symbol_id'] is not None]
        accepted = symbols[0] if symbols else None
        records.append({'image': filename, 'expected': expected, 'actual': accepted,
                        'passed': accepted == expected, 'elapsed_s': time.monotonic()-started,
                        'detections': detections})
    args.output.write_text(json.dumps({'passed': all(x['passed'] for x in records),
                                      'source': 'Previously captured Pi cases, not new live observations',
                                      'cases': records}, indent=2)+'\n')
    print(args.output.read_text())
    if not all(x['passed'] for x in records):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
