#!/usr/bin/env python3
"""Fit camera intrinsics from measured checkerboard photographs. No robot control."""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, required=True)
    parser.add_argument('--columns', type=int, required=True, help='Internal corner columns, not square count')
    parser.add_argument('--rows', type=int, required=True, help='Internal corner rows')
    parser.add_argument('--square-size-m', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new path to preserve the old calibration')
    if min(args.columns, args.rows) < 3 or args.square_size_m <= 0:
        parser.error('Invalid checkerboard dimensions')
    shape = (args.columns, args.rows)
    obj = np.zeros((args.columns*args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)*args.square_size_m
    objects, images, centres, spans = [], [], [], []
    size = None
    for path in sorted(args.images.iterdir()):
        if path.suffix.lower() not in {'.png', '.jpg', '.jpeg'}:
            continue
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        current_size = gray.shape[::-1]
        if size is not None and size != current_size:
            parser.error('Calibration photographs must share one resolution')
        size = current_size
        ok, corners = cv2.findChessboardCornersSB(gray, shape)
        if ok:
            objects.append(obj.copy()); images.append(corners)
            points = corners.reshape(-1, 2)
            centres.append(points.mean(0)/np.asarray(size))
            spans.append(np.ptp(points, axis=0)/np.asarray(size))
    if len(images) < 12:
        parser.error(f'Only {len(images)} usable checkerboard views; need at least 12')
    if max(np.ptp(centres, axis=0)) < .15 or max(np.ptp(spans, axis=0)) < .10:
        parser.error('Views lack position/scale diversity; move and tilt the checkerboard between captures')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, size, None, None)
    if not np.isfinite(matrix).all() or rms > 1.5:
        parser.error(f'Calibration reprojection RMS {rms:.3f}px is too high; improve views')
    template = Path(__file__).resolve().parent/'config/camera_calibration.yaml'
    config = yaml.safe_load(template.read_text())
    config.update({'image_width': size[0], 'image_height': size[1],
                   'matrix': matrix.reshape(-1).tolist(), 'distortion': distortion.reshape(-1).tolist(),
                   'calibration_rms_px': float(rms), 'calibration_views': len(images),
                   'range_verified': False, 'mount_verified': False})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(config, sort_keys=False))
    print(f'Saved {args.output}; RMS {rms:.3f}px. Range and camera mount still require physical verification.')


if __name__ == '__main__':
    main()
