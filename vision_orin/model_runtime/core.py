"""Shared YOLO inference and robot decisions. Coordinates always use raw pixels."""
from __future__ import annotations
import contextlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ROI_PATHS = {
    'intake': Path(os.environ.get('LARC_INTAKE_CONFIG', ROOT / 'main_vision/orin_config.json')),
    'separator': Path(os.environ.get('LARC_SEPARATOR_CONFIG', ROOT / 'separator/separator_config.json')),
}
ROI_KEYS = {
    'intake': ('det_cy_l', 'det_cy_r', 'det_thick', 'trigger_x', 'trig_y2'),
    'separator': ('roi_x', 'roi_y', 'roi_w', 'roi_h'),
}

def read_json(path):
    return json.loads(Path(path).read_text())

def load_roi(role):
    cfg = read_json(ROI_PATHS[role])
    if role == 'separator' and isinstance(cfg.get('roi'), dict):
        cfg.update({f'roi_{k}': cfg['roi'][k] for k in ('x', 'y', 'w', 'h')})
    return cfg

def settings(path=None, weights=None, device=None):
    cfg = read_json(path or os.environ.get('LARC_MODEL_CONFIG', ROOT / 'model_config.json'))
    p = Path(weights or os.environ.get('LARC_MODEL_WEIGHTS', cfg['weights'])).expanduser()
    cfg['weights'] = str(p if p.is_absolute() else ROOT / p)
    if device is not None:
        cfg['device'] = device
    if not 0 < cfg['confidence'] <= 1 or not 0 < cfg['max_frame_age_ms'] < 500:
        raise ValueError('confidence must be (0,1]; max_frame_age_ms must be (0,500).')
    return cfg

def regions(role, cfg, shape, expected=None):
    h, w = shape[:2]
    if expected and (w, h) != tuple(expected):
        raise ValueError(f'{role}: actual {w}x{h}, configured {expected[0]}x{expected[1]}; check cameras.json and ROI coordinates')
    if role == 'intake':
        half = w // 2
        lo, hi = sorted((cfg['trigger_x'], cfg['trig_y2']))
        t = cfg['det_thick']
        if w % 2 or t <= 0 or not 0 <= lo < hi <= h:
            raise ValueError('Invalid intake trigger band for this image')
        boxes = []
        for key, offset in (('det_cy_l', 0), ('det_cy_r', half)):
            x = cfg[key]
            if not 0 <= x-t < x+t <= half:
                raise ValueError(f'{key} +/- det_thick is outside this camera half')
            boxes.append((x-t+offset, lo, x+t+offset, hi))
        return boxes
    x,y,rw,rh = (cfg[k] for k in ROI_KEYS['separator'])
    if x < 0 or y < 0 or rw <= 0 or rh <= 0 or x+rw > w or y+rh > h:
        raise ValueError(f'Separator ROI ({x},{y},{rw},{rh}) is outside {w}x{h}. Set its ROI in the preview; outputs remain neutral.')
    return [(x,y,x+rw,y+rh)]

def neutral(role, error=None):
    return {'message': 'VISION:00' if role == 'intake' else 'VISION:FD:00:00',
            'left': False, 'right': False, 'warm': False, 'cool': False,
            'error': error, 'detections': [], 'regions': []}

def decide(role, detections, cfg, shape, options, expected=None):
    out = neutral(role)
    try:
        boxes = regions(role, cfg, shape, expected)
    except (ValueError, KeyError, TypeError) as exc:
        return neutral(role, str(exc))
    out['regions'] = boxes
    out['detections'] = detections
    if role == 'intake':
        half = shape[1] // 2
        for d in detections:
            if d['name'] not in options['intake_pick_classes']:
                continue
            cx,cy,r = d['circle']
            side = 0 if cx < half else 1
            x1,y1,x2,y2 = boxes[side]
            # Preserve center-in-x-strip + enclosing-circle/y-band rule.
            if x1 <= cx <= x2 and cy+r >= y1 and cy-r <= y2:
                out['left' if side == 0 else 'right'] = True
        out['message'] = f"VISION:{int(out['right']) | (int(out['left']) << 1):02X}"
    else:
        x1,y1,x2,y2 = boxes[0]
        names = set()
        for d in detections:
            a,b,c,e = d['box']
            # Positive bounding-box overlap with the separator ROI.
            if min(c,x2) > max(a,x1) and min(e,y2) > max(b,y1):
                names.add(d['name'])
        if 'ball_green' in names:  # Existing separator green-priority policy.
            out['warm'] = True
        else:
            out['warm'] = bool(names & set(options['separator_mature_classes']))
            out['cool'] = bool(names & set(options['separator_overmature_classes']))
        out['message'] = f"VISION:FD:{int(out['warm']):02X}:{int(out['cool']):02X}"
    return out

class Detector:
    def __init__(self, options):
        import torch
        from ultralytics import YOLO
        self.options = options
        self.last_result = {}
        if str(options['device']) in ('0', 'cuda:0') and not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable. Use the Orin CUDA-enabled Python environment, or --device cpu for an offline check.')
        if not Path(options['weights']).is_file():
            raise FileNotFoundError(options['weights'])
        with contextlib.redirect_stdout(sys.stderr):
            self.model = YOLO(options['weights'])
        if self.model.task not in ('detect', 'segment'):
            raise ValueError('Expected an Ultralytics detection or segmentation model')
        names = set(self.model.names.values())
        required = set(options['required_classes'])
        if not required <= names or names - set(options['allowed_classes']):
            raise ValueError(f'Incompatible classes: {sorted(names)}. Expected {sorted(required)}; update model_config.json deliberately for new classes.')
        cameras = read_json(os.environ.get('LARC_CAMERAS_JSON', ROOT/'cameras.json'))
        self.expected = {r: (cameras[r]['width'], cameras[r]['height']) for r in ROI_PATHS}

    def warmup(self, roles):
        """Warm each camera shape without opening cameras or publishing decisions."""
        import numpy as np
        with contextlib.redirect_stdout(sys.stderr):
            for width, height in dict.fromkeys(self.expected[role] for role in roles):
                self.model.predict(np.zeros((height, width, 3), dtype=np.uint8),
                    device=self.options['device'], imgsz=self.options['imgsz'],
                    conf=self.options['confidence'], verbose=False)

    def run(self, frame, role, cfg, captured_at, annotate=False):
        import cv2
        import numpy as np
        start = time.monotonic()
        with contextlib.redirect_stdout(sys.stderr):
            result = self.model.predict(frame, device=self.options['device'],
                imgsz=self.options['imgsz'], conf=self.options['confidence'],
                verbose=False)[0]
        predicted_at = time.monotonic()
        detections = []
        polygons = result.masks.xy if result.masks is not None else None
        for i, row in enumerate(result.boxes.data.cpu().tolist()):
            x1,y1,x2,y2,confidence,cls = row
            circle = ((x1+x2)/2, (y1+y2)/2, max(x2-x1,y2-y1)/2)
            if polygons is not None and len(polygons[i]) >= 3:
                (cx,cy),r = cv2.minEnclosingCircle(np.asarray(polygons[i], dtype=np.float32))
                circle = (cx,cy,r)
            detections.append({'name': result.names[int(cls)], 'confidence': confidence,
                               'box': [x1,y1,x2,y2], 'circle': list(circle)})
        out = decide(role, detections, cfg, frame.shape, self.options, self.expected[role])
        decision_at = time.monotonic()
        image = result.plot() if annotate else None
        completed_at = time.monotonic()
        age = (completed_at-captured_at)*1000
        previous = self.last_result.get(role)
        self.last_result[role] = completed_at
        if age > self.options['max_frame_age_ms']:
            out = neutral(role, f'Frame too old: {age:.0f} ms; outputs neutral')
        out.update(inference_ms=round((predicted_at-start)*1000,1),
                   decision_ms=round((decision_at-start)*1000,1),
                   network_ms=round(result.speed.get('inference', 0),1),
                   processing_ms=round((completed_at-start)*1000,1),
                   frame_age_ms=round(age,1),
                   output_interval_ms=round((completed_at-previous)*1000,1) if previous else None,
                   detection_count=len(detections),
                   width=frame.shape[1], height=frame.shape[0])
        if annotate:
            for x1,y1,x2,y2 in out['regions']:
                cv2.rectangle(image, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,255), 2)
            cv2.putText(image, out['message'], (10,25), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255),2)
        return out, image
