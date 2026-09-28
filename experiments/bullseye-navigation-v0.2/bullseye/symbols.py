"""Strict canonical ONNX reader; marker is never an assessment symbol."""
import ast
import hashlib
from pathlib import Path

import cv2
import numpy as np


def symbol_id(name):
    name = str(name).strip().lower()
    if name in {'marker', 'target', 'bullseye', 'bulls_eye', 'bulls-eye'}:
        return None
    return int(name) if name.isascii() and name.isdigit() and 11 <= int(name) <= 40 else None


class SymbolReader:
    def __init__(self, model, expected_sha256, threads=2, confidence=.5, invert=True):
        import onnxruntime as ort
        model = Path(model)
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        if not expected_sha256 or digest != expected_sha256:
            raise ValueError('ONNX checksum does not match the configured, verified model')
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        self.session = ort.InferenceSession(str(model), sess_options=opts,
                                           providers=['CPUExecutionProvider'])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].shape != [1, 3, 640, 640] or
                inputs[0].type != 'tensor(float)' or len(outputs) != 1 or
                outputs[0].shape != [1, 35, 8400] or outputs[0].type != 'tensor(float)'):
            raise ValueError('Expected static canonical FP32 raw YOLOv8 ONNX')
        names = ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        names = [names[i] for i in range(31)] if isinstance(names, dict) else names
        if names != [str(i) for i in range(11, 41)] + ['marker']:
            raise ValueError('Canonical class order mismatch')
        self.names, self.input_name = names, inputs[0].name
        self.confidence, self.invert = confidence, invert

    def predict(self, frame, roi=None):
        # Restrict recognition to the projected obstacle face before classification.
        offset_x = offset_y = 0
        if roi is not None:
            x1, y1, x2, y2 = roi
            offset_x, offset_y = x1, y1
            frame = frame[y1:y2, x1:x2]
        if frame.size == 0:
            return []
        if self.invert:
            frame = 255-frame
        h, w = frame.shape[:2]
        scale = min(640/w, 640/h)
        nw, nh = round(w*scale), round(h*scale)
        left, top = (640-nw)//2, (640-nh)//2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[top:top+nh, left:left+nw] = cv2.resize(frame, (nw, nh))
        blob = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255
        raw = self.session.run(None, {self.input_name: blob})[0][0].T
        classes = raw[:, 4:].argmax(axis=1)
        scores = raw[:, 4:].max(axis=1)
        boxes, scores_out, names = [], [], []
        for row, cid, score in zip(raw, classes, scores):
            if score < self.confidence:
                continue
            cx, cy, bw, bh = row[:4]
            x = max(0, min(w-1, (float(cx-bw/2)-left)/scale))
            y = max(0, min(h-1, (float(cy-bh/2)-top)/scale))
            x2 = max(x+1, min(w, (float(cx+bw/2)-left)/scale))
            y2 = max(y+1, min(h, (float(cy+bh/2)-top)/scale))
            boxes.append([x, y, x2-x, y2-y]); scores_out.append(float(score)); names.append(self.names[int(cid)])
        kept = cv2.dnn.NMSBoxes(boxes, scores_out, self.confidence, .45)
        result = []
        for i in np.asarray(kept).reshape(-1):
            x, y, w, h = boxes[i]
            name = names[i]
            result.append({'name': name, 'symbol_id': symbol_id(name), 'confidence': scores_out[i],
                           'box': [round(x+offset_x), round(y+offset_y), round(x+w+offset_x), round(y+h+offset_y)]})
        return sorted(result, key=lambda item: item['confidence'], reverse=True)
