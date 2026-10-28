#!/usr/bin/env python3
"""
separator_vision.py — headless ball colour classifier (separator)

Purpose : Watch a fixed ROI, classify the ball sitting there by colour, and
          emit the result as VISION tags on stdout. The dispatcher forwards
          them to the Teensy, which owns all actuation. Drives no hardware.
Camera  : role "separator" in ../cameras.json (C920 046d:08e5, MJPG)
Config  : separator_config.json
Output  : VISION:FD:WW:CC on stdout (WW = warm hit, CC = cool hit)
Classes : white = no ball / idle, warm (R/O/Y) and green = mature,
          cool (blue) and black = immature, unknown = NEUTRAL (see classify())
Debug   : separator_debug.py
Stop    : Ctrl+C
"""

import cv2
import numpy as np
import sys, json, os, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "link"))
from camera_select import open_role, CameraNotFound


CONFIG_FILE = "separator_config.json"

# Autocalibration. Off with --no-autocalib, or "autocalib": false in the

USE_AUTOCALIB = "--no-autocalib" not in sys.argv
CALIB_HZ = 8.0
CALIB_EVERY = 3
# Kept for reference only
FRAME_W     = 640
FRAME_H     = 480

DEFAULT_CFG = dict(
    roi_x=551, roi_y=208, roi_w=197, roi_h=251,
    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,
    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,
    green_h_lo=36, green_h_hi=92,
    green_s_min=60, green_v_min=40,
    black_v_max=50,
    white_s_max=40,
    white_v_min=180,
    warm_frac=5,
    cool_frac=5,
    green_frac=5,
    white_frac=60,
    unknown_to_mature=0,
    unknown_cover_min=35,
    morph_k=5,
    morph_iter=1,
)

# - Config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults", file=sys.stderr)
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    if "roi" in saved:
        r = saved["roi"]
        saved["roi_x"] = r["x"]; saved["roi_y"] = r["y"]
        saved["roi_w"] = r["w"]; saved["roi_h"] = r["h"]
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] loaded — ROI {cfg['roi_w']}x{cfg['roi_h']} @ ({cfg['roi_x']},{cfg['roi_y']})",
          file=sys.stderr)
    return cfg


def _pct(mask, roi_pixels):
    return np.count_nonzero(mask) / roi_pixels * 100


def masks_for(roi_hsv, cfg):
    #Build the warm / cool / green / white masks for one ROI.
    warm = cv2.inRange(roi_hsv,
        np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
        np.array([cfg['warm_h_hi'], 255, 255], np.uint8))
    if cfg['warm_h_lo'] < 10:
        # Red wraps around the hue circle — also catch the high-hue red band
        warm = cv2.bitwise_or(warm, cv2.inRange(roi_hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255, 255], np.uint8)))

    green = cv2.inRange(roi_hsv,
        np.array([cfg['green_h_lo'], cfg['green_s_min'], cfg['green_v_min']], np.uint8),
        np.array([cfg['green_h_hi'], 255, 255], np.uint8))

    cool = cv2.inRange(roi_hsv,
        np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
        np.array([cfg['cool_h_hi'], 255, 255], np.uint8))
    blk  = (roi_hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool = cv2.bitwise_or(cool, blk)

    white = ((roi_hsv[:, :, 1] < cfg['white_s_max']) &
             (roi_hsv[:, :, 2] > cfg['white_v_min'])).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    out = {}
    for name, mask in (("warm", warm), ("cool", cool),
                       ("green", green), ("white", white)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=n)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=n)
        out[name] = mask
    return out


def classify(pct, cfg):
    #Mask percentages - (warm_hit, cool_hit, no_ball, reason).
    if pct['white'] >= cfg['white_frac']:
        return False, False, True, "no ball"

    green_hit = pct['green'] >= cfg['green_frac']
    warm_hit  = pct['warm']  >= cfg['warm_frac']
    cool_hit  = pct['cool']  >= cfg['cool_frac']
    if green_hit:
        return True, False, False, "green -> MATURE"

    if warm_hit or cool_hit:
        if warm_hit and cool_hit:
            return True, True, False, "ambiguous (warm+cool)"
        return warm_hit, cool_hit, False, (
            "warm -> MATURE" if warm_hit else "cool -> IMMATURE")
    if cfg.get('unknown_to_mature', 0):
        if (100.0 - pct['white']) >= cfg['unknown_cover_min']:
            return True, False, False, "unknown -> MATURE (fallback)"

    return False, False, False, "unknown -> NEUTRAL (background?)"


def process(roi_hsv, cfg, roi_pixels):
    """Return (warm_hit, cool_hit, no_ball) for the current ROI."""
    pct = {n: _pct(m, roi_pixels)
           for n, m in masks_for(roi_hsv, cfg).items()}
    warm_hit, cool_hit, no_ball, _ = classify(pct, cfg)
    return warm_hit, cool_hit, no_ball

# - Main
def main():
    cfg = load_cfg()

    try:
        cap, cam = open_role("separator")
    except CameraNotFound as exc:
        sys.exit(f"[ERROR] separator camera: {exc}")

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] {cam.device}  {cam.describe()}  {w}x{h}", file=sys.stderr)

    rx = max(0, min(cfg['roi_x'], w - 2))
    ry = max(0, min(cfg['roi_y'], h - 2))
    rw = min(cfg['roi_w'], w - rx)
    rh = min(cfg['roi_h'], h - ry)
    roi_pixels = rw * rh
    print(f"[ROI] {rw}x{rh} @ ({rx},{ry})  = {roi_pixels} px", file=sys.stderr)

    # autocalibration
    # Runs on its own thread and publishes a new config by atomic rebind.
    calib = None
    if USE_AUTOCALIB and cfg.get("autocalib", True):
        try:
            from separator_autocalib import SeparatorCalibrator, load_seed
            seed = load_seed()
            calib = SeparatorCalibrator(cfg, seed=seed, hz=CALIB_HZ).start()
            print(f"[Calib] on at {CALIB_HZ:.0f} Hz — "
                  + ("seed loaded, hue adaptation "
                     + ("enabled" if calib.hue_enabled else "LOCKED (no reference balls)")
                     if seed else
                     "no seed file, self-seeding; hue LOCKED"),
                  file=sys.stderr)
        except Exception as exc:
            print(f"[Calib] disabled — {exc}", file=sys.stderr)
            calib = None
    else:
        print("[Calib] off", file=sys.stderr)

    frames = 0
    hits_w = hits_c = idle = 0
    hits_g = hits_u = 0
    t0 = time.time()
    t_last = t0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            roi = frame[ry:ry+rh, rx:rx+rw]

            if calib is not None:
                cfg = calib.current
                if frames % CALIB_EVERY == 0:
                    calib.offer(roi.copy())

            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            pct = {n: _pct(m, roi_pixels)
                   for n, m in masks_for(roi_hsv, cfg).items()}
            warm_hit, cool_hit, no_ball, reason = classify(pct, cfg)
            print(f"VISION:FD:{int(warm_hit):02X}:{int(cool_hit):02X}", flush=True)

            if no_ball:  idle   += 1
            if warm_hit: hits_w += 1
            if cool_hit: hits_c += 1
            if reason.startswith("green"): hits_g += 1
            if "fallback" in reason:       hits_u += 1

            frames += 1
            now = time.time()
            if now - t_last >= 2.0:
                fps = frames / (now - t0)
                print(f"[SEP] {fps:5.1f} fps  W:{hits_w}  C:{hits_c}  "
                      f"G:{hits_g}  fallback:{hits_u}  idle:{idle}  | {reason}",
                      file=sys.stderr)
                if calib is not None:
                    print("  " + calib.status(), file=sys.stderr)
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped] {frames} frames  {frames/max(elapsed,0.1):.1f} fps", file=sys.stderr)
    finally:
        if calib is not None:
            calib.stop()
        cap.release()

if __name__ == "__main__":
    main()
