#!/usr/bin/env python3
"""
benefits_debug.py — benefits preview, calibration and servo bench

Purpose : Same box colour detector as benefits.py plus preview windows, a
          trackbar for every value in benefits_config.json, and an OPTIONAL
          direct serial link to the Teensy. With --teensy it streams real
          protocol-v2 BENEFITS frames itself, so a red or blue box pulses the
          matching door with no Orin and no dispatcher in the loop.
Config  : benefits_config.json (shared with benefits.py)

Usage
    python3 benefits_debug.py                 tuning only
    python3 benefits_debug.py --teensy        doors follow the camera
    python3 benefits_debug.py --list          every camera that responds
    python3 benefits_debug.py --device 1 --teensy

    sudo systemctl stop larc-dispatcher       before --teensy on the Orin

Keys
    s   save trackbar values into benefits_config.json
    r   reload the file, discarding edits     p  print values as JSON
    7/8 pulse door 1 / door 2 by hand         9  close both and re-arm
    x   HALT, everything safe                 (7/8/9/x need --teensy)
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

from camera_select import CameraNotFound  # noqa: E402
from debug_camera import open_for_debug, scan  # noqa: E402

CONFIG_FILE = os.path.join(HERE, "benefits_config.json")
FONT = cv2.FONT_HERSHEY_SIMPLEX
PANEL_W, PANEL_H = 420, 300

BOX_NONE, BOX_RED, BOX_BLUE = 0, 1, 2
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}
BOX_DOOR = {BOX_RED: 0, BOX_BLUE: 1} # must match dispatcher.py
PULSE_SEC = 0.35 # how long we assert "open"

DEFAULT_CFG = dict(
    roi_x1=200, roi_y1=150,
    roi_x2=440, roi_y2=330,
    red_h_lo1=0,   red_h_hi1=10,
    red_h_lo2=170, red_h_hi2=180,
    red_s_min=80,  red_v_min=80,
    blue_h_lo=100, blue_h_hi=130,
    blue_s_min=80, blue_v_min=60,
    min_pct=5,
    morph_k=5,
    morph_iter=1,
)

TRACKBARS = [
    ("roi_x1", 1920), ("roi_y1", 1080), ("roi_x2", 1920), ("roi_y2", 1080),
    ("red_h_lo1", 180), ("red_h_hi1", 180),
    ("red_h_lo2", 180), ("red_h_hi2", 180),
    ("red_s_min", 255), ("red_v_min", 255),
    ("blue_h_lo", 180), ("blue_h_hi", 180),
    ("blue_s_min", 255), ("blue_v_min", 255),
    ("min_pct", 100),
    ("morph_k", 31), ("morph_iter", 6),
]

CONTROLS_WIN = "CONTROLS"
MASKS_WIN = "MASKS  red | blue"
RESULT_WIN = "RESULT"


# config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as handle:
        saved = json.load(handle)
    cfg = {**DEFAULT_CFG, **{k: v for k, v in saved.items() if not k.startswith("_")}}
    print(f"[Config] loaded {CONFIG_FILE}")
    return cfg


def save_cfg(cfg: dict) -> None:
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
    cfg["morph_k"] = max(1, cfg["morph_k"] | 1)
    cfg["morph_iter"] = max(1, cfg["morph_iter"])
    return cfg


# detector identical maths to benefits.py
def detect_box(roi_bgr, cfg):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total = roi_bgr.shape[0] * roi_bgr.shape[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']

    red = cv2.bitwise_or(
        cv2.inRange(hsv,
                    np.array([cfg['red_h_lo1'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
                    np.array([cfg['red_h_hi1'], 255, 255], np.uint8)),
        cv2.inRange(hsv,
                    np.array([cfg['red_h_lo2'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
                    np.array([cfg['red_h_hi2'], 255, 255], np.uint8)))
    blue = cv2.inRange(hsv,
                       np.array([cfg['blue_h_lo'], cfg['blue_s_min'], cfg['blue_v_min']], np.uint8),
                       np.array([cfg['blue_h_hi'], 255, 255], np.uint8))

    masks = {}
    for name, mask in (("red", red), ("blue", blue)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=n)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=n)
        masks[name] = mask

    pct = {name: np.count_nonzero(mask) / total * 100 for name, mask in masks.items()}

    if pct["red"] >= cfg['min_pct'] and pct["red"] > pct["blue"]:
        box = BOX_RED
    elif pct["blue"] >= cfg['min_pct'] and pct["blue"] > pct["red"]:
        box = BOX_BLUE
    else:
        box = BOX_NONE
    return box, masks, pct


# drawing
def mask_panel(mask, label, pct, threshold, winner, colour):
    vis = cv2.cvtColor(cv2.resize(mask, (PANEL_W, PANEL_H)), cv2.COLOR_GRAY2BGR)
    if winner:
        vis[mask_resized_nonzero(mask)] = colour
    cv2.rectangle(vis, (0, 0), (PANEL_W, 34), (0, 0, 0), -1)
    cv2.putText(vis, f"{label}  {pct:5.1f}%  (>= {threshold}%)", (8, 23),
                FONT, 0.5, colour if winner else (150, 150, 150), 1, cv2.LINE_AA)
    if winner:
        cv2.rectangle(vis, (1, 1), (PANEL_W - 2, PANEL_H - 2), colour, 2)
    return vis


def mask_resized_nonzero(mask):
    return cv2.resize(mask, (PANEL_W, PANEL_H)) > 0


def result_panel(frame, cfg, box, pct, doors, link_note):
    vis = frame.copy()
    x1, y1, x2, y2 = cfg['roi_x1'], cfg['roi_y1'], cfg['roi_x2'], cfg['roi_y2']
    colour = {BOX_RED: (60, 60, 235), BOX_BLUE: (235, 160, 60)}.get(box, (170, 170, 170))
    cv2.rectangle(vis, (x1, y1), (x2, y2), colour, 2)
    cv2.putText(vis, "ROI", (x1 + 4, max(16, y1 - 6)), FONT, 0.5, colour, 1, cv2.LINE_AA)

    bar = np.zeros((80, vis.shape[1], 3), np.uint8)
    cv2.putText(bar, f"BOX: {BOX_NAMES[box]}", (10, 30), FONT, 0.8, colour, 2, cv2.LINE_AA)
    cv2.putText(bar, f"red {pct['red']:5.1f}%   blue {pct['blue']:5.1f}%   "
                     f"door1={'OPEN' if doors[0] else '----'} "
                     f"door2={'OPEN' if doors[1] else '----'}",
                (10, 56), FONT, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(bar, link_note, (10, 74), FONT, 0.42, (140, 200, 140), 1, cv2.LINE_AA)
    return np.vstack([vis, bar])


# main
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benefits tuning + optional direct Teensy control.")
    parser.add_argument("--device",
                        help="camera index (0,1,...) or path. Optional: on the "
                             "Orin the 'benefits' role from cameras.json is used, and "
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
        cap, cam = open_for_debug("benefits", args.device, name=args.name,
                                  width=args.width, height=args.height)
    except CameraNotFound as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] {cam.device}  {cam.describe()}  {frame_w}x{frame_h}")

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
        print(f"[Teensy] {link.device} @ {vp.BAUD} — streaming BENEFITS frames")
        link.set_ready(True)
        link.set_benefits_running(True)
        link.set_phase_benefits()
    else:
        print("[Teensy] not connected (add --teensy to drive the doors)")

    cv2.namedWindow(CONTROLS_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WIN, 520, 640)
    for key, maximum in TRACKBARS:
        if key in ("roi_x1", "roi_x2"):
            maximum = frame_w
        elif key in ("roi_y1", "roi_y2"):
            maximum = frame_h
        cv2.createTrackbar(key, CONTROLS_WIN, int(cfg.get(key, 0)), maximum,
                           lambda _v: None)
    cv2.namedWindow(MASKS_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(RESULT_WIN, cv2.WINDOW_NORMAL)

    last_box = BOX_NONE
    armed = [True, True]
    pulse_until = [0.0, 0.0]
    frames = 0
    t0 = time.time()
    t_last = t0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            cfg = read_trackbars()
            x1 = max(0, min(cfg['roi_x1'], frame_w - 2))
            y1 = max(0, min(cfg['roi_y1'], frame_h - 2))
            x2 = max(x1 + 2, min(cfg['roi_x2'], frame_w))
            y2 = max(y1 + 2, min(cfg['roi_y2'], frame_h))
            cfg['roi_x1'], cfg['roi_y1'], cfg['roi_x2'], cfg['roi_y2'] = x1, y1, x2, y2

            box, masks, pct = detect_box(frame[y1:y2, x1:x2], cfg)

            # Same pulse + re-arm rule the dispatcher uses.
            now = time.time()
            if box != last_box:
                if box == BOX_NONE:
                    armed = [True, True]
                else:
                    door = BOX_DOOR.get(box)
                    if door is not None and armed[door]:
                        pulse_until[door] = now + PULSE_SEC
                        armed[door] = False
                        print(f"[box] {BOX_NAMES[last_box]} -> {BOX_NAMES[box]}"
                              f"  pulsing door {door + 1}")
                last_box = box

            doors = [now < pulse_until[0], now < pulse_until[1]]

            link_note = "no Teensy — tuning only"
            if link:
                import vision_protocol as vp
                link.set_benefits(doors[0], doors[1])
                link_note = f"TEENSY {link.device}  BENEFITS frames streaming"
                for request in link.take_requests():
                    print(f"[teensy] requests {vp.CMD_NAMES.get(request, hex(request))}")
                for line in link.take_messages():
                    print(f"[teensy] {line}")

            cv2.imshow(MASKS_WIN, np.hstack([
                mask_panel(masks["red"], "RED -> door 1", pct["red"],
                           cfg['min_pct'], box == BOX_RED, (60, 60, 235)),
                mask_panel(masks["blue"], "BLUE -> door 2", pct["blue"],
                           cfg['min_pct'], box == BOX_BLUE, (235, 160, 60)),
            ]))
            cv2.imshow(RESULT_WIN, result_panel(frame, cfg, box, pct, doors, link_note))

            frames += 1
            if now - t_last >= 2.0:
                print(f"[FPS] {frames / (now - t0):5.1f}   box={BOX_NAMES[box]}")
                t_last = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                save_cfg(cfg)
            elif key == ord('r'):
                cfg = load_cfg()
                for name, _ in TRACKBARS:
                    cv2.setTrackbarPos(name, CONTROLS_WIN, int(cfg.get(name, 0)))
            elif key == ord('p'):
                print(json.dumps({k: int(v) for k, v in cfg.items()}, indent=2))
            elif key in (ord('7'), ord('8')) and link:
                door = 0 if key == ord('7') else 1
                pulse_until[door] = time.time() + PULSE_SEC
                armed[door] = False
                print(f"[key] manual pulse door {door + 1}")
            elif key == ord('9') and link:
                pulse_until = [0.0, 0.0]
                armed = [True, True]
                print("[key] both doors closed and re-armed")
            elif key == ord('x') and link:
                link.halt()
                pulse_until = [0.0, 0.0]
                armed = [True, True]
                print("[key] HALT sent — everything safe")
                time.sleep(0.4)
                link.set_phase_benefits()

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
