#!/usr/bin/env python3
"""
benefits.py — box colour + alignment detector (benefits phase)

Purpose : Watch three small ROIs (left, centre, right) during the benefits
          phase. When all three see the same colour the box is centred and
          its colour is reported; anything less reports NONE, so the
          Teensy only stops at a box it is lined up with.
Camera  : role "benefits" in ../cameras.json (C920 serial 0E9612EF, MJPG)
Config  : benefits_config.json
Output  : VISION:FE:XX lines on stdout (00 none, 01 red, 02 blue).
          This file owns no serial port.
Tune    : python3 tools/tune_beans_web.py --no-intake --no-separator
Stop    : Ctrl+C
"""

import cv2
import numpy as np
import sys, os, time, json

# Camera selection
# The camera is chosen by IDENTITY (serial / VID:PID) out of cameras.json,
# never by /dev/video number. See ../link/camera_select.py.

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "link"))
from camera_select import open_role, CameraNotFound


# - Config
CONFIG_FILE = "benefits_config.json"

BOX_NONE = 0
BOX_RED  = 1
BOX_BLUE = 2
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}

# p1 = left, p2 = centre, p3 = right, in 640x480 frame pixels
POINTS = ("p1", "p2", "p3")

DEFAULT_CFG = dict(
    p1_x=220, p1_y=240,
    p2_x=320, p2_y=240,
    p3_x=420, p3_y=240,
    roi_size=16,
    red_h_lo1=0,    red_h_hi1=10,
    red_h_lo2=170,  red_h_hi2=180,
    red_s_min=80,   red_v_min=80,
    blue_h_lo=100,  blue_h_hi=130,
    blue_s_min=80,  blue_v_min=60,
    min_pct=60,
    morph_k=3,
    morph_iter=1,
)

def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found -- using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    cfg = {**DEFAULT_CFG, **{k: v for k, v in saved.items() if k in DEFAULT_CFG}}
    print(f"[Config] loaded {CONFIG_FILE}")
    return cfg

# - Detection
def masks_for(bgr, cfg):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mk = max(1, cfg['morph_k'] | 1)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
    n = max(1, cfg['morph_iter'])

    red1 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo1'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi1'], 255,              255],              np.uint8))
    red2 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo2'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi2'], 255,              255],              np.uint8))
    red = cv2.bitwise_or(red1, red2)
    blue = cv2.inRange(hsv,
        np.array([cfg['blue_h_lo'], cfg['blue_s_min'], cfg['blue_v_min']], np.uint8),
        np.array([cfg['blue_h_hi'], 255,               255],               np.uint8))

    def clean(mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=n)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=n)
    return clean(red), clean(blue)

def roi_boxes(cfg, width, height):
    half = max(1, cfg['roi_size'] // 2)
    boxes = []
    for p in POINTS:
        cx = max(half, min(cfg[f'{p}_x'], width - half))
        cy = max(half, min(cfg[f'{p}_y'], height - half))
        boxes.append((cx - half, cy - half, cx + half, cy + half))
    return boxes

def roi_colour(red_pct, blue_pct, cfg):
    if red_pct >= cfg['min_pct'] and red_pct > blue_pct:
        return BOX_RED
    if blue_pct >= cfg['min_pct'] and blue_pct > red_pct:
        return BOX_BLUE
    return BOX_NONE

def analyze(frame, cfg):
    """Returns box (only when centred), align, and per-ROI colours/pcts."""
    h, w = frame.shape[:2]
    boxes = roi_boxes(cfg, w, h)
    x1 = min(b[0] for b in boxes); y1 = min(b[1] for b in boxes)
    x2 = max(b[2] for b in boxes); y2 = max(b[3] for b in boxes)
    red, blue = masks_for(frame[y1:y2, x1:x2], cfg)

    colours, pcts = [], []
    for bx1, by1, bx2, by2 in boxes:
        r = red[by1 - y1:by2 - y1, bx1 - x1:bx2 - x1]
        b = blue[by1 - y1:by2 - y1, bx1 - x1:bx2 - x1]
        total = max(1, r.size)
        red_pct = np.count_nonzero(r) / total * 100
        blue_pct = np.count_nonzero(b) / total * 100
        colours.append(roi_colour(red_pct, blue_pct, cfg))
        pcts.append((red_pct, blue_pct))

    left, centre, right = colours
    if left == centre == right != BOX_NONE:
        align = "CENTER"
    elif left == centre == right == BOX_NONE:
        align = "NONE"
    elif left != BOX_NONE and right == BOX_NONE:
        align = "LEFT"
    elif right != BOX_NONE and left == BOX_NONE:
        align = "RIGHT"
    else:
        align = "MIXED"

    box = left if align == "CENTER" else BOX_NONE
    return dict(box=box, align=align, colours=colours, pcts=pcts, boxes=boxes)

# - Main
def main():
    cfg = load_cfg()

    try:
        cap, cam = open_role("benefits")
    except CameraNotFound as exc:
        sys.exit(f"[ERROR] benefits camera: {exc}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] {cam.device}  {cam.describe()}  {actual_w}x{actual_h}")
    print("[Mode] Dispatcher (VISION tags via stdout)")
    print("[Running] Ctrl+C to stop")

    frame_count = 0
    t0     = time.time()
    t_last = t0
    last_align = "NONE"

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            result = analyze(frame, cfg)
            box_type = result['box']

            # Emit vision data for the dispatcher to forward: VISION:FE:XX (hex)
            print(f"VISION:FE:{box_type:02X}", flush=True)

            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                rois = " ".join(BOX_NAMES[c][0] if c else "-" for c in result['colours'])
                print(f"[FPS] {fps:5.1f}  |  Box: {BOX_NAMES[box_type]:4s}  |  "
                      f"align: {result['align']:6s}  L/C/R: {rois}")
                t_last = now

            if result['align'] != last_align:
                print(f"[ALIGN] {last_align} to {result['align']}  box={BOX_NAMES[box_type]}")
                last_align = result['align']

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  avg {frame_count/elapsed:.1f} fps")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
