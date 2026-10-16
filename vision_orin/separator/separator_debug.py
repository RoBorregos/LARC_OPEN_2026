#!/usr/bin/env python3
"""
separator_debug.py — separator preview, calibration and servo bench

Purpose : Same classifier as separator_vision.py plus preview windows, a
trackbar for every value in separator_config.json, and an OPTIONAL
direct serial link to the Teensy. With --teensy it streams real
protocol-v2 BEANS frames itself, so the separator servo follows the
camera with no Orin and no dispatcher in the loop.
Config  : separator_config.json (shared with separator_vision.py)

Serial  : with --teensy this process becomes what the dispatcher normally is,
the only owner of the port, so the service must be stopped first.
It streams a BEANS frame at 50 Hz and prints what comes back — the
Teensy's request bytes and any text a sketch prints.

Usage
python3 separator_debug.py            (tuning only)
python3 separator_debug.py --teensy    (servo follows the camera)
python3 separator_debug.py --list      (every camera that responds)
python3 separator_debug.py --device 1 --teensy
sudo systemctl stop larc-dispatcher     (before --teensy on the Orin)

Keys
    c   autocalibration: observe -> apply -> off. Observe only REPORTS.
    s   save trackbar values into separator_config.json (always YOUR values)
    r   reload the file, discarding edits      p  print values as JSON
    1/2 toggle intake upper / lower            n  force NEUTRAL for 2 s
    x   HALT, everything safe                  (1/2 and x need --teensy)
    q / ESC  quit, parks the robot first

Windows : CONTROLS (one trackbar per value), MASKS (warm / cool / white and
          their percentages), RESULT (live frame, ROI, verdict).
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

from camera_select import CameraNotFound  # noqa: E402
from debug_camera import open_for_debug, scan  # noqa: E402

CONFIG_FILE = os.path.join(HERE, "separator_config.json")
FONT = cv2.FONT_HERSHEY_SIMPLEX
PANEL_W, PANEL_H = 360, 270

# Same defaults as separator_vision.py — keep them in step.
DEFAULT_CFG = dict(
    roi_x=238, roi_y=133, roi_w=87, roi_h=176,
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

# (config key, trackbar max). ROI maxima are patched once we know the frame.
TRACKBARS = [
    ("roi_x", 1920), ("roi_y", 1080), ("roi_w", 1920), ("roi_h", 1080),
    ("warm_h_lo", 180), ("warm_h_hi", 180),
    ("warm_s_min", 255), ("warm_v_min", 255),
    ("cool_h_lo", 180), ("cool_h_hi", 180),
    ("cool_s_min", 255), ("cool_v_min", 255),
    ("green_h_lo", 180), ("green_h_hi", 180),
    ("green_s_min", 255), ("green_v_min", 255),
    ("black_v_max", 255),
    ("white_s_max", 255), ("white_v_min", 255),
    ("warm_frac", 100), ("cool_frac", 100), ("green_frac", 100),
    ("white_frac", 100), ("unknown_to_mature", 1),
    ("unknown_cover_min", 100),
    ("morph_k", 31), ("morph_iter", 6),
]

CONTROLS_WIN = "CONTROLS"
MASKS_WIN = "MASKS  warm | cool | green | white"
RESULT_WIN = "RESULT"


# config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as handle:
        saved = json.load(handle)
    if "roi" in saved:
        roi = saved["roi"]
        saved.update(roi_x=roi["x"], roi_y=roi["y"], roi_w=roi["w"], roi_h=roi["h"])
    cfg = {**DEFAULT_CFG, **{k: v for k, v in saved.items() if not k.startswith("_")}}
    print(f"[Config] loaded — ROI {cfg['roi_w']}x{cfg['roi_h']} "
          f"@ ({cfg['roi_x']},{cfg['roi_y']})")
    return cfg


def save_cfg(cfg: dict) -> None:
    """Rewrite the file, keeping the _comment keys that document it."""
    existing = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as handle:
            existing = json.load(handle)
    existing.update({k: int(v) for k, v in cfg.items()})
    with open(CONFIG_FILE, "w") as handle:
        json.dump(existing, handle, indent=2)
        handle.write("\n")
    print(f"[Config] saved -> {CONFIG_FILE}")


def read_trackbars() -> dict:
    cfg = {key: cv2.getTrackbarPos(key, CONTROLS_WIN) for key, _ in TRACKBARS}
    cfg["morph_k"] = max(1, cfg["morph_k"] | 1)      # must be odd
    cfg["morph_iter"] = max(1, cfg["morph_iter"])
    cfg["roi_w"] = max(2, cfg["roi_w"])
    cfg["roi_h"] = max(2, cfg["roi_h"])
    return cfg

def process(roi_hsv, cfg, roi_pixels):
    warm = cv2.inRange(roi_hsv,
                       np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
                       np.array([cfg['warm_h_hi'], 255, 255], np.uint8))
    if cfg['warm_h_lo'] < 10:
        warm = cv2.bitwise_or(warm, cv2.inRange(
            roi_hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255, 255], np.uint8)))

    green = cv2.inRange(roi_hsv,
                        np.array([cfg['green_h_lo'], cfg['green_s_min'], cfg['green_v_min']], np.uint8),
                        np.array([cfg['green_h_hi'], 255, 255], np.uint8))

    cool = cv2.inRange(roi_hsv,
                       np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
                       np.array([cfg['cool_h_hi'], 255, 255], np.uint8))
    black = (roi_hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool = cv2.bitwise_or(cool, black)

    white = ((roi_hsv[:, :, 1] < cfg['white_s_max']) &
             (roi_hsv[:, :, 2] > cfg['white_v_min'])).astype(np.uint8) * 255

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    masks = {}
    for name, mask in (("warm", warm), ("cool", cool),
                       ("green", green), ("white", white)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=n)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=n)
        masks[name] = mask

    pct = {name: np.count_nonzero(mask) / roi_pixels * 100
           for name, mask in masks.items()}

    if pct["white"] >= cfg['white_frac']:
        return False, False, True, masks, pct, "no ball"

    green_hit = pct["green"] >= cfg['green_frac']
    warm_hit  = pct["warm"]  >= cfg['warm_frac']
    cool_hit  = pct["cool"]  >= cfg['cool_frac']

    if green_hit:
        return True, False, False, masks, pct, "GREEN -> LEFT (mature)"

    if warm_hit or cool_hit:
        if warm_hit and cool_hit:
            return True, True, False, masks, pct, "ambiguous (both)"
        return warm_hit, cool_hit, False, masks, pct, (
            "WARM -> LEFT" if warm_hit else "COOL -> RIGHT")

    if cfg.get('unknown_to_mature', 0):
        if (100.0 - pct["white"]) >= cfg['unknown_cover_min']:
            return True, False, False, masks, pct, "UNKNOWN -> LEFT (fallback)"

    # Almost always the empty gate failing the white test, not a ball.
    return False, False, False, masks, pct, "UNKNOWN -> NEUTRAL (background?)"


# drawing
def mask_panel(mask, label, pct, threshold, hit):
    vis = cv2.cvtColor(cv2.resize(mask, (PANEL_W, PANEL_H)), cv2.COLOR_GRAY2BGR)
    colour = (0, 220, 0) if hit else (150, 150, 150)
    cv2.rectangle(vis, (0, 0), (PANEL_W, 34), (0, 0, 0), -1)
    cv2.putText(vis, f"{label}  {pct:5.1f}%  (>= {threshold}%)", (8, 23),
                FONT, 0.5, colour, 1, cv2.LINE_AA)
    if hit:
        cv2.rectangle(vis, (1, 1), (PANEL_W - 2, PANEL_H - 2), (0, 220, 0), 2)
    return vis


def result_panel(frame, cfg, verdict, pct, link_note):
    vis = frame.copy()
    x, y = cfg['roi_x'], cfg['roi_y']
    w, h = cfg['roi_w'], cfg['roi_h']
    colour = {"WARM -> LEFT": (60, 160, 255),
              "COOL -> RIGHT": (255, 160, 60),
              "GREEN -> LEFT (mature)": (60, 220, 60),
              "UNKNOWN -> LEFT (fallback)": (60, 220, 200),
              "UNKNOWN -> NEUTRAL (background?)": (170, 170, 170),
              }.get(verdict, (170, 170, 170))
    cv2.rectangle(vis, (x, y), (x + w, y + h), colour, 2)
    cv2.putText(vis, "ROI", (x + 4, max(16, y - 6)), FONT, 0.5, colour, 1, cv2.LINE_AA)

    bar = np.zeros((78, vis.shape[1], 3), np.uint8)
    cv2.putText(bar, verdict, (10, 30), FONT, 0.8, colour, 2, cv2.LINE_AA)
    cv2.putText(bar, f"warm {pct['warm']:5.1f}%   cool {pct['cool']:5.1f}%   "
                     f"green {pct['green']:5.1f}%   white {pct['white']:5.1f}%",
                (10, 54), FONT, 0.45,
                (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(bar, link_note, (10, 72), FONT, 0.42, (140, 200, 140), 1, cv2.LINE_AA)
    return np.vstack([vis, bar])


# main
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Separator tuning + optional direct Teensy control.")
    parser.add_argument("--device",
                        help="camera index (0,1,...) or path. Optional: on the "
                             "Orin the 'separator' role from cameras.json is used, and "
                             "anywhere else the camera is auto-detected.")
    parser.add_argument("--name",
                        help="pick the camera whose name contains this, "
                             "e.g. --name C920. Overrides the name_contains "
                             "in cameras.json.")
    parser.add_argument("--list", action="store_true",
                        help="list every camera that responds, then exit")
    parser.add_argument("--teensy", action="store_true",
                        help="drive the Teensy directly over serial")
    parser.add_argument("--port", help="serial device (default: autodetect)")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    args = parser.parse_args()

    if args.list:
        scan(args.width, args.height)
        return 0

    cfg = load_cfg()

    try:
        cap, cam = open_for_debug("separator", args.device, name=args.name,
                                  width=args.width, height=args.height)
    except CameraNotFound as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] {cam.device}  {cam.describe()}  {frame_w}x{frame_h}")

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

    from autocalib import DebugHarness
    from separator_autocalib import SeparatorCalibrator, load_seed
    _seed = load_seed()
    harness = DebugHarness(
        lambda manual: SeparatorCalibrator(manual, seed=_seed),
        mode="off" if "--no-autocalib" in sys.argv else "observe")
    print("[Calib] " + ("seed loaded" if _seed else "no seed file — "
          "photometric only, hue locked")
          + "   c = observe/apply/off")

    # windows
    cv2.namedWindow(CONTROLS_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WIN, 520, 720)
    for key, maximum in TRACKBARS:
        if key in ("roi_x", "roi_w"):
            maximum = frame_w
        elif key in ("roi_y", "roi_h"):
            maximum = frame_h
        cv2.createTrackbar(key, CONTROLS_WIN, int(cfg.get(key, 0)), maximum,
                           lambda _v: None)
    cv2.namedWindow(MASKS_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(RESULT_WIN, cv2.WINDOW_NORMAL)

    upper = lower = False
    force_neutral_until = 0.0
    frames = 0
    t0 = time.time()
    t_last = t0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            manual = read_trackbars()

            x = max(0, min(manual['roi_x'], frame_w - 2))
            y = max(0, min(manual['roi_y'], frame_h - 2))
            w = max(2, min(manual['roi_w'], frame_w - x))
            h = max(2, min(manual['roi_h'], frame_h - y))
            manual['roi_x'], manual['roi_y'] = x, y
            manual['roi_w'], manual['roi_h'] = w, h

            roi_bgr = frame[y:y + h, x:x + w]
            cfg = harness.update(manual, roi_bgr)
            roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
            warm_hit, cool_hit, no_ball, masks, pct, verdict = process(
                roi_hsv, cfg, w * h)

            link_note = "no Teensy — tuning only"
            if link:
                import vision_protocol as vp
                if time.time() < force_neutral_until:
                    side = vp.SEP_NEUTRAL
                elif warm_hit and not cool_hit:
                    side = vp.SEP_LEFT
                elif cool_hit and not warm_hit:
                    side = vp.SEP_RIGHT
                else:
                    side = vp.SEP_NEUTRAL
                link.set_beans(upper, lower, side)
                link_note = (f"TEENSY {link.device}  sep={vp.SEP_NAMES[side]}  "
                             f"upper={int(upper)} lower={int(lower)}")

                for request in link.take_requests():
                    print(f"[teensy] requests {vp.CMD_NAMES.get(request, hex(request))}")
                for line in link.take_messages():
                    print(f"[teensy] {line}")

            # draw
            cv2.imshow(MASKS_WIN, np.hstack([
                mask_panel(masks["warm"], "WARM", pct["warm"], cfg['warm_frac'],
                           warm_hit),
                mask_panel(masks["cool"], "COOL", pct["cool"], cfg['cool_frac'],
                           cool_hit),
                mask_panel(masks["green"], "GREEN (mature)", pct["green"],
                           cfg['green_frac'], pct["green"] >= cfg['green_frac']),
                mask_panel(masks["white"], "WHITE (no ball)", pct["white"],
                           cfg['white_frac'], no_ball),
            ]))
            cv2.imshow(RESULT_WIN, result_panel(frame, cfg, verdict, pct, link_note))

            frames += 1
            now = time.time()
            if now - t_last >= 2.0:
                print(f"[FPS] {frames / (now - t0):5.1f}   {verdict}")
                for line in harness.lines(manual):
                    print("      " + line)
                t_last = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                save_cfg(manual)
            elif key == ord('c'):
                print(f"[key] autocalib -> {harness.cycle().upper()}")
            elif key == ord('r'):
                reloaded = load_cfg()
                for name, _ in TRACKBARS:
                    cv2.setTrackbarPos(name, CONTROLS_WIN,
                                       int(reloaded.get(name, 0)))
            elif key == ord('p'):
                print(json.dumps({k: int(v) for k, v in manual.items()}, indent=2))
            elif key == ord('n'):
                force_neutral_until = time.time() + 2.0
                print("[key] separator forced NEUTRAL for 2 s")
            elif key == ord('1') and link:
                upper = not upper
                print(f"[key] intake upper -> {'DEPLOY' if upper else 'home'}")
            elif key == ord('2') and link:
                lower = not lower
                print(f"[key] intake lower -> {'DEPLOY' if lower else 'home'}")
            elif key == ord('x') and link:
                link.halt()
                upper = lower = False
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
