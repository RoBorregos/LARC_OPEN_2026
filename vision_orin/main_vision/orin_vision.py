#!/usr/bin/env python3
"""
orin_vision.py — headless bean detector (intake)

Purpose : Run the bean-detection loop at full camera resolution and emit what
          it sees as VISION tags on stdout. The dispatcher forwards them to
          the Teensy, which owns all actuation. This file drives no hardware.
Camera  : role "intake" in ../cameras.json (ZED 2, matched by VID:PID)
Config  : orin_config.json
Output  : VISION:XX on stdout (bit 0 = right bean, bit 1 = left bean)
Debug   : orin_vision_debug.py — same pipeline with preview windows and
          trackbars. Tune there, then copy orin_config.json onto the Orin.
Stop    : Ctrl+C
"""

import cv2
import numpy as np
import sys, json, os, time

# Camera selection
# The camera is chosen by IDENTITY (serial / VID:PID) out of cameras.json,
# never by /dev/video number — those get reshuffled on every boot. See
# ../link/camera_select.py and ../cameras.json.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "link"))
from camera_select import open_role, CameraNotFound


#Constants
CONFIG_FILE = "orin_config.json"

# Autocalibration (photometric only). Off with --no-autocalib, or
# "autocalib": false in the config. CALIB_EVERY throttles how often a
# frame is handed to the calibrator thread; the published config is still
# read every single frame.
USE_AUTOCALIB = "--no-autocalib" not in sys.argv
CALIB_HZ = 8.0
CALIB_EVERY = 3
# Kept for reference only — the real size comes from ../cameras.json.
FRAME_W     = 1344
FRAME_H     = 376

# - Config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        sys.exit(f"[ERROR] {CONFIG_FILE} not found — run the debug tool first and press S.")
    with open(CONFIG_FILE) as f:
        raw = json.load(f)
    return dict(
        s_min      = raw['s_min'],
        v_dark     = raw['v_dark'],
        bg_h_lo    = raw['bg_h_lo'],
        bg_h_hi    = raw['bg_h_hi'],
        bg_s_min   = raw['bg_s_min'],
        bg_v_min   = raw['bg_v_min'],
        area_min   = raw['area_min'] * 10,
        area_max   = raw['area_max'] * 10,
        rad_min    = max(1, raw['rad_min']),
        rad_max    = max(1, raw['rad_max']),
        circ_min   = raw['circ_min'] * 0.01,
        circ_max   = raw['circ_max'] * 0.01,
        morph_k    = max(1, raw['morph_k'] | 1),
        morph_iter = max(1, raw['morph_iter']),
        det_cy_l   = raw['det_cy_l'],
        det_cy_r   = raw['det_cy_r'],
        det_thick  = max(1, raw['det_thick']),
        trigger_x  = raw['trigger_x'],
        trig_y2    = raw['trig_y2'],
    )

# Kalman ghost tracker
GHOST_FRAMES = 0

def _make_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
    kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],
                                       [0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf

_ghost = {
    'left':  dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
    'right': dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
}

def update_ghost(side, contour_data):
    g    = _ghost[side]
    best = next(((cx,cy,ri) for cx,cy,ri,passed in contour_data if passed), None)
    if best:
        cx, cy, ri = best
        g['kf'].correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
        g['ttl'] = GHOST_FRAMES
        g['r']   = ri
    else:
        g['ttl'] = max(0, g['ttl'] - 1)
    pred     = g['kf'].predict()
    g['pos'] = (int(pred[0].item()), int(pred[1].item()))

def check_trigger(contour_data, cfg, side):
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])
    for cx, cy, ri, passed in contour_data:
        if passed and (cy + ri >= y_lo) and (cy - ri <= y_hi):
            return True
    g = _ghost[side]
    if g['ttl'] > 0:
        gcy, gri = g['pos'][1], g['r']
        if (gcy + gri >= y_lo) and (gcy - gri <= y_hi):
            return True
    return False

# Vision pipeline
def build_mask(half, cfg):
    hsv      = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    raw_cand = ((hsv[:,:,1] >= cfg['s_min']) | (hsv[:,:,2] < cfg['v_dark'])).astype(np.uint8) * 255
    bg_lo    = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], dtype=np.uint8)
    bg_hi    = np.array([cfg['bg_h_hi'], 255,             255],             dtype=np.uint8)
    clean    = cv2.bitwise_and(raw_cand, cv2.bitwise_not(cv2.inRange(hsv, bg_lo, bg_hi)))
    k        = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    morph    = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    morph    = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    return morph

def detect(mask, cfg, side):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        area  = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ      = 4 * np.pi * area / (perim ** 2)
        (x, y), r = cv2.minEnclosingCircle(cnt)
        cx, cy, ri = int(x), int(y), int(r)
        passed = (abs(cx - det_cy) <= det_th and
                  cfg['rad_min']  <= ri   <= cfg['rad_max'] and
                  cfg['area_min'] <= area <= cfg['area_max'] and
                  cfg['circ_min'] <= circ <= cfg['circ_max'])
        out.append((cx, cy, ri, passed))
    return out

# - Main
def main():
    cfg = load_cfg()
    print(f"[Config] loaded {CONFIG_FILE} (full resolution)", file=sys.stderr)

    try:
        cap, cam = open_role("intake")
    except CameraNotFound as exc:
        sys.exit(f"[ERROR] intake camera: {exc}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w   = actual_w // 2
    print(f"[Camera] {cam.device}  {cam.describe()}", file=sys.stderr)
    print(f"[Camera] {actual_w}x{actual_h}  half={half_w}x{actual_h}", file=sys.stderr)

    # autocalibration
    # Photometric only: s_min, v_dark and the background band.
    calib = None
    if USE_AUTOCALIB and cfg.get("autocalib", True):
        try:
            from intake_autocalib import IntakeCalibrator, load_seed
            seed, note = load_seed(cfg)
            calib = IntakeCalibrator(cfg, seed=seed, hz=CALIB_HZ).start()
            print(f"[Calib] on at {CALIB_HZ:.0f} Hz — {note}", file=sys.stderr)
        except Exception as exc:
            print(f"[Calib] disabled — {exc}", file=sys.stderr)
            calib = None
    else:
        print("[Calib] off", file=sys.stderr)

    print("[Running] Ctrl+C to stop", file=sys.stderr)

    frame_count = 0
    hit_counts  = {'left': 0, 'right': 0}
    t0          = time.time()
    t_last      = t0

    try:
        while True:
            t_grab = time.monotonic()
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            t_read = time.monotonic()

            if calib is not None:
                cfg = calib.current
                if frame_count % CALIB_EVERY == 0:
                    calib.offer(frame.copy())

            # Process each half at full resolution
            halves = [
                ('left',  frame[:, :half_w]),
                ('right', frame[:, half_w:]),
            ]

            hits = {}
            for side, half in halves:
                mask   = build_mask(half, cfg)
                cdata  = detect(mask, cfg, side)
                update_ghost(side, cdata)
                hits[side] = check_trigger(cdata, cfg, side)
                if hits[side]:
                    hit_counts[side] += 1

            t_proc = time.monotonic()

            # Serial output: single-byte bitfield the dispatcher forwards to the Teensy.
            bitfield = (int(hits['right']) & 1) | ((int(hits['left']) & 1) << 1)
            print(f"VISION:{bitfield:02X}", flush=True)
            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                l_str = "L" if hits['left'] else " "
                r_str = "R" if hits['right'] else " "
                read_ms = (t_read - t_grab) * 1000
                proc_ms = (t_proc - t_read) * 1000
                total_ms = (t_proc - t_grab) * 1000
                print(f"[FPS] {fps:5.1f}  |  {l_str} {r_str}  |  "
                      f"L:{hit_counts['left']:4d}  R:{hit_counts['right']:4d}  |  "
                      f"read:{read_ms:.0f} proc:{proc_ms:.0f} total:{total_ms:.0f}ms",
                      file=sys.stderr)
                if calib is not None:
                    print("  " + calib.status(), file=sys.stderr)
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  "
              f"avg {frame_count/elapsed:.1f} fps", file=sys.stderr)
    finally:
        if calib is not None:
            calib.stop()
        cap.release()

if __name__ == "__main__":
    main()
