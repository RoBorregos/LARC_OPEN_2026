#!/usr/bin/env python3
"""separator-bridge-xpgm.py -- feeds dispatcher.py's "separator" role with
the HSV classifier from sorter-xpgm.py (xpgm-testVision), instead of the
production separator_vision.py / separator_model.py.

Only used when dispatcher.py's USE_XPGM_TEST flag is True (see
vision_orin/dispatcher/dispatcher.py). Default there is False, so this file
existing changes nothing on its own.

Contract dispatcher.py expects on stdout, one line per frame, unbuffered
(see dispatcher.py's own docstring -- "Script contract"):

    VISION:FD:WW:CC     WW = warm hit (01/00), CC = cool hit (01/00)

Reuses sorter-xpgm.py's classify() / classify_rgb_mean() / masks_for() /
_pct() / DEFAULT_CFG the same way dashboardSorter-xpgm.py already does
(importlib, no live import of separator_vision.py -- keeps the
xpgm-testVision isolation rule). Camera is opened by role via
link/camera_select.py, the one piece of infrastructure dashboardSorter-xpgm.py
already reuses on purpose (USB identity, not classification logic).
separator_config.json is read the same way too, as data, with fallback to
sorter-xpgm.py's DEFAULT_CFG if it is missing.

--method hsv (default)  uses classify() -- the real production HSV logic.
--method rgb             uses classify_rgb_mean() -- nearest BGR centroid,
                         the experiment this sidequest exists to answer.
"""
import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
SEPARATOR_DIR = HERE.parent / 'separator'
LINK_DIR = HERE.parent / 'link'


def load_sorter_xpgm():
    spec = importlib.util.spec_from_file_location("sorter_xpgm", HERE / "sorter-xpgm.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def load_cfg(sx):
    """Read separator_config.json as DATA (merged over sorter-xpgm.py's own
    DEFAULT_CFG) -- same pattern as dashboardSorter-xpgm.py's load_cfg()."""
    cfg = dict(sx.DEFAULT_CFG)
    path = SEPARATOR_DIR / "separator_config.json"
    if path.exists():
        saved = json.loads(path.read_text())
        if "roi" in saved:
            r = saved["roi"]
            saved["roi_x"], saved["roi_y"] = r["x"], r["y"]
            saved["roi_w"], saved["roi_h"] = r["w"], r["h"]
        cfg.update(saved)
    return cfg


def open_separator_camera():
    """The only piece of this file that depends on code outside
    xpgm-testVision: USB-identity camera resolution, not classification
    logic -- see module docstring."""
    sys.path.insert(0, str(LINK_DIR))
    from camera_select import open_role, CameraNotFound
    try:
        return open_role("separator")
    except CameraNotFound as exc:
        raise SystemExit(f"[ERROR] separator camera: {exc}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--method', choices=('hsv', 'rgb'), default='hsv',
                    help='hsv = classify() (produccion); rgb = classify_rgb_mean()')
    args = p.parse_args()

    sx = load_sorter_xpgm()
    cfg = load_cfg(sx)
    cap, cam = open_separator_camera()

    rx, ry, rw, rh = cfg['roi_x'], cfg['roi_y'], cfg['roi_w'], cfg['roi_h']
    roi_pixels = rw * rh

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            roi = frame[ry:ry + rh, rx:rx + rw]

            if args.method == 'rgb':
                mean_bgr = roi.reshape(-1, 3).mean(axis=0)
                _color, rgb_bin, _dist = sx.classify_rgb_mean(mean_bgr)
                warm_hit = rgb_bin == "MADURA"
                cool_hit = rgb_bin == "INMADURA"
            else:
                roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                pct = {name: sx._pct(mask, roi_pixels)
                       for name, mask in sx.masks_for(roi_hsv, cfg).items()}
                warm_hit, cool_hit, _no_ball, _reason = sx.classify(pct, cfg)

            print(f"VISION:FD:{int(warm_hit):02x}:{int(cool_hit):02x}", flush=True)
    finally:
        cap.release()


if __name__ == '__main__':
    main()
