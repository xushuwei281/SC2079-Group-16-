"""Compare saved Pi images with the deployed detector; does not start ROS or motors."""
import argparse
import contextlib
import gc
import hashlib
import io
import json
import platform
import resource
import statistics
import sys
import time
from pathlib import Path

import cv2

p = argparse.ArgumentParser()
p.add_argument('--runtime', type=Path, required=True)
p.add_argument('--candidate-root', type=Path, required=True)
p.add_argument('--cases-dir', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
args = p.parse_args()
sys.path.insert(0, str(args.runtime))
from mdp_perception.detector import TargetDetector

cases = json.loads((args.cases_dir / 'cases.json').read_text())
report = {'host': platform.node(), 'machine': platform.machine(), 'input': 'Saved Pi camera frames from September 9; not new physical observations', 'invert_colors': True, 'runs': []}
for name, threads in [('baseline-yolov8n-v61-full-20260908T142129Z-r2', 2), ('baseline-yolov8m-v61-full-20260911T013018Z', 2), ('baseline-yolov8m-v61-full-20260911T013018Z', 4)]:
    model = args.candidate_root / name / 'best.onnx'
    begin = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        detector = TargetDetector(str(model), onnx_threads=threads)
    run = {'name': name, 'threads': threads, 'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(), 'load_seconds': time.perf_counter()-begin, 'observations': []}
    frames = {key: cv2.imread(str(args.cases_dir / key)) for key in cases}
    assert all(frame is not None for frame in frames.values())
    detector.predict(255 - next(iter(frames.values())))
    for repeat in range(3):
        for image, expected in cases.items():
            start = time.perf_counter()
            detections = detector.predict(255 - frames[image])
            elapsed = (time.perf_counter()-start)*1000
            top = max(detections, key=lambda d:d[2]) if detections else None
            actual = top[1] if top else None
            row = {'image':image, 'repeat':repeat, 'expected':expected, 'actual':actual, 'confidence':float(top[2]) if top else None, 'inference_ms':elapsed, 'passed':actual==expected, 'detections':detections}
            run['observations'].append(row)
            print(json.dumps({'model':name,'threads':threads,**{k:v for k,v in row.items() if k!='detections'}}), flush=True)
    times = [x['inference_ms'] for x in run['observations']]
    run.update(median_ms=statistics.median(times), min_ms=min(times), max_ms=max(times), passed=all(x['passed'] for x in run['observations']), process_peak_rss_native_units=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    report['runs'].append(run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print('SUMMARY '+json.dumps({k:v for k,v in run.items() if k!='observations'}), flush=True)
    del detector
    gc.collect()
