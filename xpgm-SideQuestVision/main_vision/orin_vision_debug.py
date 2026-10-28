#!/usr/bin/env python3
"""
orin_vision_debug.py — intake preview, calibration and servo bench

Purpose : Same pipeline as orin_vision.py plus preview windows, a trackbar
for every value in orin_config.json, and an OPTIONAL direct serial
link to the Teensy. With --teensy it streams real protocol-v2 BEANS
frames itself, so a bean in the trigger band moves the intake servo
with no Orin and no dispatcher in the loop.
Config  : orin_config.json (shared with orin_vision.py)
Runs on : your Mac or the Orin. No flags needed on either.
Camera  : resolved by NAME, not index — name_contains for the "intake" role
in cameras.json, else the first camera that is not a known Apple
built-in. Names need pyobjc-framework-AVFoundation (best) or ffmpeg.

Usage
python3 orin_vision_debug.py           (tuning only)
python3 orin_vision_debug.py --teensy    (servos follow the camera
python3 orin_vision_debug.py --list    (every camera that responds)
python3 orin_vision_debug.py --device 1 --name "C920"
sudo systemctl stop larc-dispatcher     (before --teensy on the Orin)

Keys
    c   autocalibration: observe - apply - off. Observe only REPORTS what
        it would change; your trackbars still drive the pipeline.
    s   save trackbar values into orin_config.json (always YOUR values)
    r   reload the file, discarding edits      p  print values as JSON
    1/2 toggle intake upper / lower            x  HALT   (both need --teensy)
    q / ESC  quit, parks the robot first
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "link"))
sys.path.insert(0, HERE)

from camera_select import CameraNotFound
from debug_camera import open_for_debug, scan

# Absolute, so this runs from any working directory. The old version used a
# bare "orin_config.json" and only worked if you happened to cd in here first.
CONFIG_FILE = os.path.join(HERE, "orin_config.json")

PANEL_W = 420
PANEL_H = 235
FONT = cv2.FONT_HERSHEY_SIMPLEX

CONTROLS_WIN = "CONTROLS"
FILTERS_WIN = "FILTERS (L top, R bottom)"
RESULT_WIN = "RESULT  (L top, R bottom)"

DEFAULT_CFG = dict(
    s_min=145, v_dark=62,
    bg_h_lo=33, bg_h_hi=104, bg_s_min=0, bg_v_min=0,
    area_min=961, area_max=5247,
    rad_min=31, rad_max=125,
    circ_min=66, circ_max=99,
    morph_k=9, morph_iter=3,
    det_cy_l=286, det_cy_r=379, det_thick=97,
    trigger_x=103, trig_y2=376,
)
TRACKBARS = [
    ("s_min", 255), ("v_dark", 255),
    ("bg_h_lo", 180), ("bg_h_hi", 180), ("bg_s_min", 255), ("bg_v_min", 255),
    ("area_min", 20000), ("area_max", 20000),
    ("rad_min", 400), ("rad_max", 400),
    ("circ_min", 100), ("circ_max", 100),
    ("morph_k", 31), ("morph_iter", 10),
    ("det_cy_l", 1920), ("det_cy_r", 1920), ("det_thick", 1920),
    ("trigger_x", 1080), ("trig_y2", 1080),
]
HALF_WIDTH_KEYS = ("det_cy_l", "det_cy_r", "det_thick")
HEIGHT_KEYS = ("trigger_x", "trig_y2")


# config
def load_raw() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults. "
              f"Press s to write it.")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as handle:
        saved = json.load(handle)
    raw = {**DEFAULT_CFG,
           **{k: v for k, v in saved.items() if not k.startswith("_")}}
    print(f"[Config] loaded {CONFIG_FILE}")
    return raw


def derive(raw: dict) -> dict:
    return dict(
        s_min=raw['s_min'],
        v_dark=raw['v_dark'],
        bg_h_lo=raw['bg_h_lo'],
        bg_h_hi=raw['bg_h_hi'],
        bg_s_min=raw['bg_s_min'],
        bg_v_min=raw['bg_v_min'],
        area_min=raw['area_min'] * 10,
        area_max=raw['area_max'] * 10,
        rad_min=max(1, raw['rad_min']),
        rad_max=max(1, raw['rad_max']),
        circ_min=raw['circ_min'] * 0.01,
        circ_max=raw['circ_max'] * 0.01,
        morph_k=max(1, raw['morph_k'] | 1),
        morph_iter=max(1, raw['morph_iter']),
        det_cy_l=raw['det_cy_l'],
        det_cy_r=raw['det_cy_r'],
        det_thick=max(1, raw['det_thick']),
        trigger_x=raw['trigger_x'],
        trig_y2=raw['trig_y2'],
    )


def save_raw(raw: dict) -> None:
    existing = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as handle:
            existing = json.load(handle)
    existing.update({k: int(v) for k, v in raw.items()})
    with open(CONFIG_FILE, "w") as handle:
        json.dump(existing, handle, indent=2)
        handle.write("\n")
    print(f"[Config] saved -> {CONFIG_FILE}")


def read_trackbars() -> dict:
    return {key: cv2.getTrackbarPos(key, CONTROLS_WIN) for key, _ in TRACKBARS}


# pipeline — mirrors orin_vision.py, kept local so this file stands alone
def build_masks(half, cfg):
    hsv = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    coloured = (hsv[:, :, 1] >= cfg['s_min'])
    dark = (hsv[:, :, 2] < cfg['v_dark'])
    raw_cand = (coloured | dark).astype(np.uint8) * 255
    bg_green = cv2.inRange(
        hsv,
        np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], np.uint8),
        np.array([cfg['bg_h_hi'], 255, 255], np.uint8))
    clean = cv2.bitwise_and(raw_cand, cv2.bitwise_not(bg_green))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (cfg['morph_k'], cfg['morph_k']))
    morph = cv2.morphologyEx(clean, cv2.MORPH_OPEN, k, iterations=cfg['morph_iter'])
    morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    return dict(coloured=coloured, dark=dark, bg_green=bg_green, morph=morph)


def detect(mask, cfg, side):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim ** 2)
        (x, y), r = cv2.minEnclosingCircle(cnt)
        cx, cy, ri = int(x), int(y), int(r)
        pz = abs(cx - det_cy) <= det_th
        pr = cfg['rad_min'] <= ri <= cfg['rad_max']
        pa = cfg['area_min'] <= area <= cfg['area_max']
        pc = cfg['circ_min'] <= circ <= cfg['circ_max']
        out.append((cx, cy, ri, area, circ, pz and pr and pa and pc, pz, pr, pa, pc))
    return out


def check_trigger(cdata, cfg):
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])
    for cx, cy, ri, area, circ, passed, *_ in cdata:
        if passed and (cy + ri >= y_lo) and (cy - ri <= y_hi):
            return True
    return False


# visualisation
def panel_filters(half, masks, rotate):
    vis = np.zeros((*half.shape[:2], 3), np.uint8)
    coloured, dark = masks['coloured'], masks['dark']
    bg = masks['bg_green'] > 0
    vis[coloured & ~dark & ~bg] = (53, 130, 255)
    vis[~coloured & dark & ~bg] = (200, 80, 30)
    vis[coloured & dark & ~bg] = (255, 255, 255)
    vis[bg] = (0, 0, 200)
    vis[cv2.Canny(masks['morph'], 50, 150) > 0] = (0, 255, 255)

    out = cv2.resize(vis, (PANEL_W, PANEL_H))
    total = half.shape[0] * half.shape[1]
    cv2.putText(out, f"surv:{np.count_nonzero(masks['morph']) * 100 // total}%",
                (6, PANEL_H - 8), FONT, 0.36, (180, 180, 180), 1)
    return cv2.rotate(out, cv2.ROTATE_90_COUNTERCLOCKWISE) if rotate else out


def panel_result(half, cdata, cfg, side, hit, rotate):
    vis = cv2.resize(half, (PANEL_W, PANEL_H)).copy()
    sx = PANEL_W / half.shape[1]
    sy = PANEL_H / half.shape[0]

    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    zl = int((det_cy - cfg['det_thick']) * sx)
    zr = int((det_cy + cfg['det_thick']) * sx)
    overlay = vis.copy()
    cv2.rectangle(overlay, (zl, 0), (zr, PANEL_H), (0, 60, 0), -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (zl, 0), (zl, PANEL_H), (0, 220, 0), 1)
    cv2.line(vis, (zr, 0), (zr, PANEL_H), (0, 220, 0), 1)

    y1 = int(cfg['trigger_x'] * sy)
    y2 = int(cfg['trig_y2'] * sy)
    ylo, yhi = min(y1, y2), max(y1, y2)
    line_c = (0, 0, 255) if hit else (60, 60, 220)
    fill_c = (0, 0, 120) if hit else (0, 0, 60)
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, ylo), (PANEL_W, yhi), fill_c, -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (0, ylo), (PANEL_W, ylo), line_c, 1)
    cv2.line(vis, (0, yhi), (PANEL_W, yhi), line_c, 1)

    for cx, cy, ri, area, circ, passed, pz, pr, pa, pc in cdata:
        dcx, dcy = int(cx * sx), int(cy * sy)
        dr = max(1, int(ri * (sx + sy) / 2))
        if passed:
            cv2.circle(vis, (dcx, dcy), dr + 2, (0, 220, 0), 2)
            cv2.circle(vis, (dcx, dcy), 3, (255, 255, 255), -1)
            cv2.putText(vis, f"r:{ri} c:{circ:.2f}",
                        (max(0, dcx - dr), max(12, dcy - dr - 4)),
                        FONT, 0.32, (0, 220, 0), 1)
        else:
            reason = ("zone" if not pz else f"r:{ri}" if not pr
                      else f"a:{int(area)}" if not pa else f"c:{circ:.2f}")
            cv2.circle(vis, (dcx, dcy), dr, (60, 60, 180), 1)
            cv2.putText(vis, reason, (max(0, dcx - dr), max(12, dcy - dr - 4)),
                        FONT, 0.30, (80, 80, 200), 1)

    cv2.putText(vis, side.upper(), (6, 16), FONT, 0.5, (220, 220, 220), 1)
    cv2.putText(vis, f"zone {det_cy}+/-{cfg['det_thick']}", (6, 32),
                FONT, 0.32, (0, 200, 0), 1)
    if hit:
        cv2.putText(vis, "HIT", (PANEL_W - 48, 20), FONT, 0.6, (0, 0, 255), 2)
    return cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE) if rotate else vis


# main
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Intake (ZED) tuning + optional direct Teensy control.")
    parser.add_argument("--device",
                        help="camera index (0,1,...) or path. Optional: on the "
                             "Orin the 'intake' role from cameras.json is used, "
                             "and anywhere else the camera is auto-detected.")
    parser.add_argument("--name",
                        help="pick the camera whose name contains this, e.g. "
                             "--name C920. Overrides cameras.json.")
    parser.add_argument("--list", action="store_true",
                        help="list every camera that responds, then exit")
    parser.add_argument("--teensy", action="store_true",
                        help="drive the Teensy directly over serial")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--no-rotate", action="store_true",
                        help="do not rotate the panels 90 deg for display")
    args = parser.parse_args()

    if args.list:
        scan(args.width, args.height)
        return 0

    raw = load_raw()

    try:
        cap, cam = open_for_debug("intake", args.device, name=args.name,
                                  width=args.width, height=args.height)
    except CameraNotFound as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w = max(2, frame_w // 2)
    print(f"[Camera] {cam.device}  {cam.describe()}  "
          f"{frame_w}x{frame_h}  half={half_w}x{frame_h}")

    # optional Teensy link
    link = None
    if args.teensy:
        from teensy_link import TeensyLink
        import vision_protocol as vp
        try:
            link = TeensyLink(device=args.port)
        except Exception as exc:
            print(f"[ERROR] Teensy: {exc}", file=sys.stderr)
            print("        Is the dispatcher still running? "
                  "sudo systemctl stop larc-dispatcher", file=sys.stderr)
            cap.release()
            return 1
        print(f"[Teensy] {link.device} @ {vp.BAUD} — streaming BEANS frames")
        link.set_ready(True)
        link.set_beans_running(True)
    else:
        print("[Teensy] not connected (add --teensy to drive the servos)")

    # windows
    cv2.namedWindow(CONTROLS_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WIN, 520, 700)
    for key, maximum in TRACKBARS:
        if key in HALF_WIDTH_KEYS:
            maximum = half_w
        elif key in HEIGHT_KEYS:
            maximum = frame_h
        cv2.createTrackbar(key, CONTROLS_WIN,
                           min(int(raw.get(key, 0)), maximum), maximum,
                           lambda _v: None)
    cv2.namedWindow(FILTERS_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(RESULT_WIN, cv2.WINDOW_NORMAL)
    cv2.moveWindow(FILTERS_WIN, 540, 30)
    cv2.moveWindow(RESULT_WIN, 540, 420)

    rotate = not args.no_rotate
    manual_upper = manual_lower = False
    frames = 0
    t0 = time.time()
    t_last = t0

    # autocalibration
    # Photometric only, and OBSERVE by default: it reports what it would
    # change while the trackbars still drive. Press c to cycle. Saving
    # always writes the trackbar values, never the calibrated drift.
    from autocalib import DebugHarness
    from intake_autocalib import IntakeCalibrator, load_seed
    _seed, _note = load_seed(derive(read_trackbars()))
    harness = DebugHarness(
        lambda manual: IntakeCalibrator(manual, seed=_seed),
        mode="off" if "--no-autocalib" in sys.argv else "observe")
    print(f"[Calib] {_note}   c = observe/apply/off")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            raw = read_trackbars()
            cfg = harness.update(derive(raw), frame)

            halves = (('left', frame[:, :half_w]), ('right', frame[:, half_w:]))
            filters, results, hits = [], [], {}
            for side, half in halves:
                masks = build_masks(half, cfg)
                cdata = detect(masks['morph'], cfg, side)
                hits[side] = check_trigger(cdata, cfg)
                filters.append(panel_filters(half, masks, rotate))
                results.append(panel_result(half, cdata, cfg, side,
                                            hits[side], rotate))
            upper = hits['right'] or manual_upper
            lower = hits['left'] or manual_lower

            if link:
                import vision_protocol as vp
                link.set_beans(upper, lower, vp.SEP_NEUTRAL)
                for request in link.take_requests():
                    print(f"[teensy] requests "
                          f"{vp.CMD_NAMES.get(request, hex(request))}")
                for line in link.take_messages():
                    print(f"[teensy] {line}")

            cv2.imshow(FILTERS_WIN, np.hstack(filters))
            cv2.imshow(RESULT_WIN, np.hstack(results))

            frames += 1
            now = time.time()
            if now - t_last >= 1.0:
                state = ("L" if hits['left'] else " ") + ("R" if hits['right'] else " ")
                extra = f"  upper={int(upper)} lower={int(lower)}" if link else ""
                print(f"[FPS] {frames / (now - t0):5.1f}  |  {state}{extra}")
                for line in harness.lines(derive(raw)):
                    print("      " + line)
                t_last = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                save_raw(raw)
            elif key == ord('c'):
                print(f"[key] autocalib -> {harness.cycle().upper()}")
            elif key == ord('r'):
                raw = load_raw()
                for name, _ in TRACKBARS:
                    cv2.setTrackbarPos(name, CONTROLS_WIN, int(raw.get(name, 0)))
            elif key == ord('p'):
                print(json.dumps({k: int(v) for k, v in raw.items()}, indent=2))
            elif key == ord('1') and link:
                manual_upper = not manual_upper
                print(f"[key] intake upper forced {'ON' if manual_upper else 'off'}")
            elif key == ord('2') and link:
                manual_lower = not manual_lower
                print(f"[key] intake lower forced {'ON' if manual_lower else 'off'}")
            elif key == ord('x') and link:
                link.halt()
                manual_upper = manual_lower = False
                print("[key] HALT sent — everything safe")
                time.sleep(0.4)

    except KeyboardInterrupt:
        pass
    finally:
        if link:
            link.close()
            print("[Teensy] parked and closed")
        cap.release()
        cv2.destroyAllWindows()
        elapsed = max(time.time() - t0, 0.001)
        print(f"[Stopped] {frames} frames in {elapsed:.1f}s "
              f"({frames / elapsed:.1f} fps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
