#!/usr/bin/env python3
"""
tune_beans.py — both BEANS cameras, live
Purpose: Open the intake (ZED) and the separator (C920) AT THE SAME TIME,
run the exact pipelines orin_vision.py and separator_vision.py run,
and show what each one is deciding — with a trackbar for every
value in both config files. Drives no hardware, sends no serial.

Usage
sudo systemctl stop larc-dispatcher      it owns the cameras
cd ~/LARC_OPEN_2026/vision_orin
python3 tools/tune_beans.py
...tune, press S, then: sudo systemctl start larc-dispatcher

Keys
    c   autocalibration: observe -> apply -> off, both cameras at once.
        Observe (the default) only REPORTS what it would change; your sliders
        still drive. Saving always writes YOUR values.
    s   save both configs 1  intake only 2  separator only
    h   hide/show the two control windows
    q / ESC  quit

"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# vision_orin/link is a sibling of tools/.
TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(REPO / "link"))
sys.path.insert(0, str(REPO / "separator"))
sys.path.insert(0, str(REPO / "main_vision"))

from camera_select import CameraNotFound, open_bench  # noqa: E402
from autocalib import DebugHarness  # noqa: E402
from separator_vision import (  # noqa: E402
    _pct as _sep_pct,
    classify as _sep_classify,
    masks_for as separator_masks,
)

INTAKE_CONFIG = Path(os.environ.get(
    "LARC_INTAKE_CONFIG", REPO / "main_vision" / "orin_config.json"))
SEPARATOR_CONFIG = Path(os.environ.get(
    "LARC_SEPARATOR_CONFIG", REPO / "separator" / "separator_config.json"))

FONT = cv2.FONT_HERSHEY_SIMPLEX

INTAKE_WIN = "INTAKE"
INTAKE_FILTERS_WIN = "INTAKE FILTERS"
SEPARATOR_WIN = "SEPARATOR"
INTAKE_TUNE_WIN = "TUNE intake"
SEPARATOR_TUNE_WIN = "TUNE separator"
PANEL_W, PANEL_H = 420, 235
SEP_VIEW_W = 640

INTAKE_DEFAULTS = dict(
    s_min=145, v_dark=62,
    bg_h_lo=33, bg_h_hi=104, bg_s_min=0, bg_v_min=0,
    area_min=961, area_max=5247,
    rad_min=31, rad_max=125,
    circ_min=66, circ_max=99,
    morph_k=9, morph_iter=3,
    det_cy_l=286, det_cy_r=379, det_thick=97,
    trigger_x=103, trig_y2=376,
)

# Mirrors DEFAULT_CFG in separator_vision.py.
SEPARATOR_DEFAULTS = dict(
    roi_x=551, roi_y=208, roi_w=197, roi_h=251,
    warm_h_lo=0, warm_h_hi=35, warm_s_min=80, warm_v_min=80,
    cool_h_lo=95, cool_h_hi=135, cool_s_min=60, cool_v_min=40,
    green_h_lo=36, green_h_hi=92, green_s_min=60, green_v_min=40,
    black_v_max=50, white_s_max=40, white_v_min=180,
    warm_frac=5, cool_frac=5, green_frac=5, white_frac=60,
    unknown_to_mature=0, unknown_cover_min=35,
    morph_k=5, morph_iter=1,
)


def load_raw(path: Path, defaults: dict) -> dict:
    if not path.exists():
        print(f"[config] {path.name} not found — starting from defaults")
        return dict(defaults)
    with open(path) as handle:
        saved = json.load(handle)
    if "roi" in saved and isinstance(saved["roi"], dict):
        r = saved["roi"]
        saved.update(roi_x=r.get("x"), roi_y=r.get("y"),
                     roi_w=r.get("w"), roi_h=r.get("h"))

    raw = dict(defaults)
    for key in defaults:
        if isinstance(saved.get(key), (int, float)):
            raw[key] = int(saved[key])
    print(f"[config] loaded {path}")
    return raw


def save_raw(path: Path, raw: dict) -> None:
    document = {}
    if path.exists():
        with open(path) as handle:
            document = json.load(handle)
    document.update({k: int(v) for k, v in raw.items()})
    if isinstance(document.get("roi"), dict):
        document["roi"] = {"x": raw["roi_x"], "y": raw["roi_y"],
                           "w": raw["roi_w"], "h": raw["roi_h"]}
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")
    print(f"[config] SAVED {path}")


def derive_intake(raw: dict) -> dict:
    return dict(
        s_min=raw["s_min"], v_dark=raw["v_dark"],
        bg_h_lo=raw["bg_h_lo"], bg_h_hi=raw["bg_h_hi"],
        bg_s_min=raw["bg_s_min"], bg_v_min=raw["bg_v_min"],
        area_min=raw["area_min"] * 10, area_max=raw["area_max"] * 10,
        rad_min=max(1, raw["rad_min"]), rad_max=max(1, raw["rad_max"]),
        circ_min=raw["circ_min"] * 0.01, circ_max=raw["circ_max"] * 0.01,
        morph_k=max(1, raw["morph_k"] | 1), morph_iter=max(1, raw["morph_iter"]),
        det_cy_l=raw["det_cy_l"], det_cy_r=raw["det_cy_r"],
        det_thick=max(1, raw["det_thick"]),
        trigger_x=raw["trigger_x"], trig_y2=raw["trig_y2"],
    )


def derive_separator(raw: dict) -> dict:
    cfg = dict(raw)
    cfg["morph_k"] = max(1, raw["morph_k"] | 1)
    cfg["morph_iter"] = max(1, raw["morph_iter"])
    return cfg


#  Cameras — one thread each
class CameraThread:

    def __init__(self, role: str, device=None, **open_kwargs) -> None:
        self.role = role
        self.error: str | None = None
        self.cap = None
        self.info = None

        self._frame = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._count = 0
        self._fps = 0.0
        self._t0 = time.monotonic()

        try:
            self.cap, self.info = open_bench(role=role, device=device, **open_kwargs)
        except CameraNotFound as exc:
            self.error = str(exc)
            return
        except Exception as exc:                      # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            return

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._thread = threading.Thread(target=self._run, name=f"cam-{role}",
                                        daemon=True)
        self._thread.start()

    @property
    def ok(self) -> bool:
        return self.error is None

    @property
    def fps(self) -> float:
        return self._fps

    def _run(self) -> None:
        misses = 0
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                misses += 1
                if misses > 300:
                    self.error = "stopped delivering frames (unplugged? bus starved?)"
                    return
                time.sleep(0.005)
                continue
            misses = 0
            with self._lock:
                self._frame = frame
                self._stamp = time.monotonic()
            self._count += 1
            elapsed = time.monotonic() - self._t0
            if elapsed >= 1.0:
                self._fps = self._count / elapsed
                self._count = 0
                self._t0 = time.monotonic()

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def latest_timed(self):
        """Atomically pair a frame with its OpenCV-delivery timestamp."""
        with self._lock:
            return (None if self._frame is None else self._frame.copy(), self._stamp)

    def release(self) -> None:
        self._stop.set()
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


#  Intake pipeline — mirrors orin_vision.py
def intake_masks(half, cfg):
    hsv = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    coloured = hsv[:, :, 1] >= cfg["s_min"]
    dark = hsv[:, :, 2] < cfg["v_dark"]
    raw_cand = (coloured | dark).astype(np.uint8) * 255
    bg_lo = np.array([cfg["bg_h_lo"], cfg["bg_s_min"], cfg["bg_v_min"]], np.uint8)
    bg_hi = np.array([cfg["bg_h_hi"], 255, 255], np.uint8)
    bg_green = cv2.inRange(hsv, bg_lo, bg_hi)
    clean = cv2.bitwise_and(raw_cand, cv2.bitwise_not(bg_green))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg["morph_k"], cfg["morph_k"]))
    morph = cv2.morphologyEx(clean, cv2.MORPH_OPEN, k, iterations=cfg["morph_iter"])
    morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg["morph_iter"])
    return dict(coloured=coloured, dark=dark, bg_green=bg_green, morph=morph)


def intake_detect(mask, cfg, side):
    """Same accept test as orin_vision.py, but it also reports which check
    rejected a contour — that is the number you actually need to move."""
    det_cy = cfg["det_cy_l"] if side == "left" else cfg["det_cy_r"]
    det_th = cfg["det_thick"]
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
        pr = cfg["rad_min"] <= ri <= cfg["rad_max"]
        pa = cfg["area_min"] <= area <= cfg["area_max"]
        pc = cfg["circ_min"] <= circ <= cfg["circ_max"]
        out.append(dict(cx=cx, cy=cy, r=ri, area=area, circ=circ,
                        passed=pz and pr and pa and pc,
                        pz=pz, pr=pr, pa=pa, pc=pc))
    return out


def intake_trigger(cdata, cfg):
    #Note self: no Kalman ghost here
    y_lo = min(cfg["trigger_x"], cfg["trig_y2"])
    y_hi = max(cfg["trigger_x"], cfg["trig_y2"])
    for c in cdata:
        if c["passed"] and (c["cy"] + c["r"] >= y_lo) and (c["cy"] - c["r"] <= y_hi):
            return True
    return False


def panel_intake_filters(half, masks, rotate):
    vis = np.zeros((*half.shape[:2], 3), np.uint8)
    coloured, dark = masks["coloured"], masks["dark"]
    bg = masks["bg_green"] > 0
    vis[coloured & ~dark & ~bg] = (53, 130, 255)
    vis[~coloured & dark & ~bg] = (200, 80, 30)
    vis[coloured & dark & ~bg] = (255, 255, 255)
    vis[bg] = (0, 0, 200)
    vis[cv2.Canny(masks["morph"], 50, 150) > 0] = (0, 255, 255)

    out = cv2.resize(vis, (PANEL_W, PANEL_H))
    total = half.shape[0] * half.shape[1]
    survive = np.count_nonzero(masks["morph"]) * 100 // max(1, total)
    cv2.putText(out, f"surv:{survive}%", (6, PANEL_H - 8),
                FONT, 0.36, (180, 180, 180), 1)
    return cv2.rotate(out, cv2.ROTATE_90_COUNTERCLOCKWISE) if rotate else out


def panel_intake_result(half, cdata, cfg, side, hit, rotate):
    vis = cv2.resize(half, (PANEL_W, PANEL_H))
    sx = PANEL_W / half.shape[1]
    sy = PANEL_H / half.shape[0]

    det_cy = cfg["det_cy_l"] if side == "left" else cfg["det_cy_r"]
    zl = int((det_cy - cfg["det_thick"]) * sx)
    zr = int((det_cy + cfg["det_thick"]) * sx)
    overlay = vis.copy()
    cv2.rectangle(overlay, (zl, 0), (zr, PANEL_H), (0, 60, 0), -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (zl, 0), (zl, PANEL_H), (0, 220, 0), 1)
    cv2.line(vis, (zr, 0), (zr, PANEL_H), (0, 220, 0), 1)

    y1, y2 = int(cfg["trigger_x"] * sy), int(cfg["trig_y2"] * sy)
    ylo, yhi = min(y1, y2), max(y1, y2)
    edge = (0, 0, 255) if hit else (60, 60, 220)
    fill = (0, 0, 120) if hit else (0, 0, 60)
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, ylo), (PANEL_W, yhi), fill, -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (0, ylo), (PANEL_W, ylo), edge, 1)
    cv2.line(vis, (0, yhi), (PANEL_W, yhi), edge, 1)

    for c in cdata:
        dcx, dcy = int(c["cx"] * sx), int(c["cy"] * sy)
        dr = max(1, int(c["r"] * (sx + sy) / 2))
        if c["passed"]:
            cv2.circle(vis, (dcx, dcy), dr + 2, (0, 220, 0), 2)
            cv2.circle(vis, (dcx, dcy), 3, (255, 255, 255), -1)
            label = f"r:{c['r']} c:{c['circ']:.2f}"
            colour = (0, 220, 0)
        else:
            if not c["pz"]:
                label = "zone"
            elif not c["pr"]:
                label = f"r:{c['r']}"
            elif not c["pa"]:
                label = f"a:{int(c['area'])}"
            else:
                label = f"c:{c['circ']:.2f}"
            cv2.circle(vis, (dcx, dcy), dr, (60, 60, 180), 1)
            colour = (80, 80, 200)
        cv2.putText(vis, label, (max(0, dcx - dr), max(12, dcy - dr - 4)),
                    FONT, 0.32, colour, 1)

    cv2.putText(vis, side.upper(), (6, 16), FONT, 0.5, (220, 220, 220), 1)
    cv2.putText(vis, f"zone {det_cy}+/-{cfg['det_thick']}", (6, 32),
                FONT, 0.32, (0, 200, 0), 1)
    if hit:
        cv2.putText(vis, "HIT", (PANEL_W - 48, 20), FONT, 0.6, (0, 0, 255), 2)
    return cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE) if rotate else vis

#  Separator pipeline mirrors separator_vision.py
def clamp_roi(cfg, width, height):
    rx = max(0, min(cfg["roi_x"], width - 2))
    ry = max(0, min(cfg["roi_y"], height - 2))
    rw = max(1, min(cfg["roi_w"], width - rx))
    rh = max(1, min(cfg["roi_h"], height - ry))
    return rx, ry, rw, rh


def separator_verdict(masks, cfg, roi_pixels):
    # Percentages + the real classifier's verdict for this ROI
    pct = {name: _sep_pct(mask, roi_pixels) for name, mask in masks.items()}
    warm, cool, no_ball, reason = _sep_classify(pct, cfg)
    return dict(warm=warm, cool=cool, no_ball=no_ball, reason=reason,
                warm_pct=pct["warm"], cool_pct=pct["cool"],
                green_pct=pct["green"], white_pct=pct["white"])


def verdict_label(v):
    """What the DISPATCHER will do with this frame, not just what we saw.

    warm != cool is the dispatcher's own rule (see VisionState.feed): a frame
    claiming both is 'cannot tell' and holds the previous side.
    """
    if v["no_ball"]:
        return "NO BALL", (170, 170, 170)
    if v["warm"] and v["cool"]:
        return "BOTH -> HOLD", (0, 165, 255)
    if v["warm"]:
        reason = v.get("reason", "")
        if reason.startswith("green"):
            return "GREEN -> LEFT (mature)", (60, 220, 60)
        if "fallback" in reason:
            return "UNKNOWN -> LEFT (mature, fallback)", (60, 220, 200)
        return "WARM -> LEFT (mature)", (60, 200, 255)
    if v["cool"]:
        return "COOL -> RIGHT (immature)", (255, 160, 60)
    if "unknown" in v.get("reason", ""):
        # Matched no colour band.
        return "UNKNOWN -> NEUTRAL (background?)", (170, 170, 170)
    # 00:00 with a real ball present is the stall: the dispatcher holds,
    # the hold expires, the gate parks at NEUTRAL and nothing gets through.
    return "none -> NEUTRAL (GATE STALLS)", (0, 0, 255)


def view_separator(frame, cfg, roi_box, verdict, masks, fps, dragging):
    rx, ry, rw, rh = roi_box
    scale = SEP_VIEW_W / frame.shape[1]
    vis = cv2.resize(frame, (SEP_VIEW_W, int(frame.shape[0] * scale)))

    label, colour = verdict_label(verdict)
    p1 = (int(rx * scale), int(ry * scale))
    p2 = (int((rx + rw) * scale), int((ry + rh) * scale))
    cv2.rectangle(vis, p1, p2, colour, 2)
    cv2.putText(vis, label, (p1[0], max(16, p1[1] - 8)), FONT, 0.55, colour, 2)

    bar = (f"warm {verdict['warm_pct']:5.1f}/{cfg['warm_frac']}   "
           f"cool {verdict['cool_pct']:5.1f}/{cfg['cool_frac']}   "
           f"green {verdict['green_pct']:5.1f}/{cfg['green_frac']}   "
           f"white {verdict['white_pct']:5.1f}/{cfg['white_frac']}")
    cv2.rectangle(vis, (0, vis.shape[0] - 44), (vis.shape[1], vis.shape[0]),
                  (0, 0, 0), -1)
    cv2.putText(vis, bar, (8, vis.shape[0] - 26), FONT, 0.45, (230, 230, 230), 1)
    cv2.putText(vis, f"ROI {rw}x{rh} @ ({rx},{ry})   {fps:4.1f} fps"
                     + ("   [drag a box]" if dragging else ""),
                (8, vis.shape[0] - 8), FONT, 0.42, (160, 160, 160), 1)

    # The three masks, as they look inaide the ROI.
    tile_w = SEP_VIEW_W // 4
    tiles = []
    for name in ("warm", "cool", "green", "white"):
        tile = cv2.cvtColor(masks[name], cv2.COLOR_GRAY2BGR)
        tile = cv2.resize(tile, (tile_w, 140))
        cv2.putText(tile, name, (6, 18), FONT, 0.45, (0, 220, 0), 1)
        tiles.append(tile)
    strip = np.hstack(tiles)
    if strip.shape[1] != vis.shape[1]:
        strip = cv2.resize(strip, (vis.shape[1], 140))
    return np.vstack([vis, strip])


#  Trackbars
def build_trackbars(window, raw, spec):
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 420, min(760, 26 * len(spec) + 40))
    for name, hi in spec:
        value = int(max(0, min(raw[name], hi)))
        cv2.createTrackbar(name, window, value, hi, lambda _v: None)


def read_trackbars(window, raw, spec):
    for name, _hi in spec:
        try:
            raw[name] = cv2.getTrackbarPos(name, window)
        except cv2.error:
            pass  # window was closed
    return raw


def push_trackbars(window, raw, spec):
    for name, hi in spec:
        try:
            cv2.setTrackbarPos(name, window, int(max(0, min(raw[name], hi))))
        except cv2.error:
            pass


#  Main
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Live tuning for BOTH beans-phase cameras at once.")
    parser.add_argument("--intake-device",
                        help="open the intake camera by index/path instead of "
                             "resolving the role (needed on a Mac)")
    parser.add_argument("--separator-device",
                        help="same, for the separator camera")
    parser.add_argument("--no-intake", action="store_true",
                        help="separator only")
    parser.add_argument("--no-separator", action="store_true",
                        help="intake only")
    parser.add_argument("--no-rotate", action="store_true",
                        help="do not rotate the intake panels 90 deg")
    args = parser.parse_args()

    rotate = not args.no_rotate

    intake_raw = load_raw(INTAKE_CONFIG, INTAKE_DEFAULTS)
    separator_raw = load_raw(SEPARATOR_CONFIG, SEPARATOR_DEFAULTS)

    # Open both cameras
    print("[cam] opening — if this hangs or says 'Device or resource busy', "
          "the dispatcher still owns them:  sudo systemctl stop larc-dispatcher")

    intake = None if args.no_intake else CameraThread(
        "intake", device=args.intake_device)
    separator = None if args.no_separator else CameraThread(
        "separator", device=args.separator_device)

    for cam in (intake, separator):
        if cam is None:
            continue
        if cam.ok:
            print(f"[cam] {cam.role:<10} {cam.info.device}  "
                  f"{cam.width}x{cam.height}  ({cam.info.describe()})")
        else:
            print(f"[cam] {cam.role:<10} UNAVAILABLE — {cam.error}")

    if not any(c is not None and c.ok for c in (intake, separator)):
        print("[fatal] neither camera opened. Nothing to show.")
        return 1

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        ready = [c for c in (intake, separator)
                 if c is not None and c.ok and c.latest() is not None]
        if len(ready) == len([c for c in (intake, separator)
                              if c is not None and c.ok]):
            break
        time.sleep(0.05)

    intake_spec = []
    if intake is not None and intake.ok:
        half_w = intake.width // 2
        h = intake.height
        intake_spec = [
            ("s_min", 255), ("v_dark", 255),
            ("bg_h_lo", 179), ("bg_h_hi", 179),
            ("bg_s_min", 255), ("bg_v_min", 255),
            ("area_min", 4000), ("area_max", 30000),
            ("rad_min", 300), ("rad_max", 400),
            ("circ_min", 100), ("circ_max", 100),
            ("morph_k", 21), ("morph_iter", 6),
            ("det_cy_l", max(1, half_w)), ("det_cy_r", max(1, half_w)),
            ("det_thick", max(1, half_w)),
            ("trigger_x", max(1, h)), ("trig_y2", max(1, h)),
        ]
        build_trackbars(INTAKE_TUNE_WIN, intake_raw, intake_spec)
        cv2.namedWindow(INTAKE_WIN, cv2.WINDOW_NORMAL)
        cv2.namedWindow(INTAKE_FILTERS_WIN, cv2.WINDOW_NORMAL)

    separator_spec = []
    if separator is not None and separator.ok:
        w, h = separator.width, separator.height
        if (separator_raw["roi_x"] + separator_raw["roi_w"] > w or
                separator_raw["roi_y"] + separator_raw["roi_h"] > h):
            print(f"[WARN] separator ROI {separator_raw['roi_w']}x"
                  f"{separator_raw['roi_h']} @ ({separator_raw['roi_x']},"
                  f"{separator_raw['roi_y']}) does not fit this "
                  f"{w}x{h} frame.")
            print("[WARN] separator_vision.py is clamping it to "
                  f"{clamp_roi(separator_raw, w, h)} — almost certainly not "
                  "what you tuned. Drag a new box (press d) and save with 2.")
            separator_raw["roi_x"] = min(separator_raw["roi_x"], w // 4)
            separator_raw["roi_y"] = min(separator_raw["roi_y"], h // 4)
            separator_raw["roi_w"] = min(separator_raw["roi_w"], w // 2)
            separator_raw["roi_h"] = min(separator_raw["roi_h"], h // 2)

        separator_spec = [
            ("roi_x", max(1, w - 1)), ("roi_y", max(1, h - 1)),
            ("roi_w", w), ("roi_h", h),
            ("warm_h_lo", 179), ("warm_h_hi", 179),
            ("warm_s_min", 255), ("warm_v_min", 255),
            ("cool_h_lo", 179), ("cool_h_hi", 179),
            ("cool_s_min", 255), ("cool_v_min", 255),
            ("green_h_lo", 179), ("green_h_hi", 179),
            ("green_s_min", 255), ("green_v_min", 255),
            ("black_v_max", 255),
            ("white_s_max", 255), ("white_v_min", 255),
            ("warm_frac", 100), ("cool_frac", 100), ("green_frac", 100),
            ("white_frac", 100), ("unknown_to_mature", 1),
            ("unknown_cover_min", 100),
            ("morph_k", 21), ("morph_iter", 6),
        ]
        build_trackbars(SEPARATOR_TUNE_WIN, separator_raw, separator_spec)
        cv2.namedWindow(SEPARATOR_WIN, cv2.WINDOW_NORMAL)

    # Mouse ROI drag on the separator view
    drag = dict(active=False, start=None, box=None, enabled=False, scale=1.0)

    def on_mouse(event, x, y, _flags, _param):
        if not drag["enabled"]:
            return
        fx, fy = int(x / drag["scale"]), int(y / drag["scale"])
        if event == cv2.EVENT_LBUTTONDOWN:
            drag["active"] = True
            drag["start"] = (fx, fy)
        elif event == cv2.EVENT_LBUTTONUP and drag["active"]:
            drag["active"] = False
            x0, y0 = drag["start"]
            x1, y1 = fx, fy
            rx, ry = min(x0, x1), min(y0, y1)
            rw, rh = abs(x1 - x0), abs(y1 - y0)
            if rw > 8 and rh > 8:
                separator_raw.update(roi_x=rx, roi_y=ry, roi_w=rw, roi_h=rh)
                push_trackbars(SEPARATOR_TUNE_WIN, separator_raw, separator_spec)
                print(f"[roi] {rw}x{rh} @ ({rx},{ry})  — press 2 to save")

    if separator is not None and separator.ok:
        cv2.setMouseCallback(SEPARATOR_WIN, on_mouse)

    print("\n[keys] s save both   1 intake   2 separator   r reload   "
          "p print   d drag-ROI   h hide sliders   q quit\n")

    tune_hidden = False
    last_report = 0.0
    hits = dict(left=0, right=0, warm=0, cool=0)

    # Autocalibration
    from intake_autocalib import IntakeCalibrator
    from intake_autocalib import load_seed as load_intake_seed
    from separator_autocalib import SeparatorCalibrator
    from separator_autocalib import load_seed as load_separator_seed

    _i_seed, _i_note = load_intake_seed(derive_intake(intake_raw))
    _s_seed = load_separator_seed()
    _mode = "off" if "--no-autocalib" in sys.argv else "observe"
    intake_calib = DebugHarness(
        lambda manual: IntakeCalibrator(manual, seed=_i_seed), mode=_mode)
    separator_calib = DebugHarness(
        lambda manual: SeparatorCalibrator(manual, seed=_s_seed), mode=_mode)
    print(f"[Calib] intake: {_i_note}")
    print("[Calib] separator: " + ("seed loaded" if _s_seed else
          "no seed file — photometric only, hue locked"))
    print("[Calib] c = observe/apply/off   (observe only REPORTS)")

    try:
        while True:
            # Intake
            intake_hits = None
            if intake is not None and intake.ok:
                if not tune_hidden:
                    read_trackbars(INTAKE_TUNE_WIN, intake_raw, intake_spec)
                frame = intake.latest()
                if frame is not None:
                    intake_manual = derive_intake(intake_raw)
                    cfg = intake_calib.update(intake_manual, frame)
                    half_w = frame.shape[1] // 2
                    filters, results, intake_hits = [], [], {}
                    for side, half in (("left", frame[:, :half_w]),
                                       ("right", frame[:, half_w:])):
                        masks = intake_masks(half, cfg)
                        cdata = intake_detect(masks["morph"], cfg, side)
                        hit = intake_trigger(cdata, cfg)
                        intake_hits[side] = hit
                        if hit:
                            hits[side] += 1
                        filters.append(panel_intake_filters(half, masks, rotate))
                        results.append(panel_intake_result(
                            half, cdata, cfg, side, hit, rotate))
                    cv2.imshow(INTAKE_FILTERS_WIN, np.hstack(filters))
                    cv2.imshow(INTAKE_WIN, np.hstack(results))

            # Separator
            verdict = None
            if separator is not None and separator.ok:
                if not tune_hidden:
                    read_trackbars(SEPARATOR_TUNE_WIN, separator_raw, separator_spec)
                frame = separator.latest()
                if frame is not None:
                    sep_manual = derive_separator(separator_raw)
                    rx, ry, rw, rh = clamp_roi(sep_manual, frame.shape[1],
                                               frame.shape[0])
                    roi_bgr = frame[ry:ry + rh, rx:rx + rw]
                    cfg = separator_calib.update(sep_manual, roi_bgr)
                    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                    masks = separator_masks(roi_hsv, cfg)
                    verdict = separator_verdict(masks, cfg, max(1, rw * rh))
                    if verdict["warm"]:
                        hits["warm"] += 1
                    if verdict["cool"]:
                        hits["cool"] += 1
                    drag["scale"] = SEP_VIEW_W / frame.shape[1]
                    cv2.imshow(SEPARATOR_WIN,
                               view_separator(frame, cfg, (rx, ry, rw, rh),
                                              verdict, masks, separator.fps,
                                              drag["enabled"]))

            # One status line a second, so a log is readable later
            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                bits = []
                if intake_hits is not None:
                    payload = (int(intake_hits["right"]) |
                               (int(intake_hits["left"]) << 1))
                    bits.append(
                        f"intake {'L' if intake_hits['left'] else '-'}"
                        f"{'R' if intake_hits['right'] else '-'} "
                        f"VISION:{payload:02X} {intake.fps:4.1f}fps")
                if verdict is not None:
                    bits.append(f"sep {verdict_label(verdict)[0]:<24} "
                                f"VISION:FD:{int(verdict['warm']):02X}:"
                                f"{int(verdict['cool']):02X} "
                                f"{separator.fps:4.1f}fps")
                for cam in (intake, separator):
                    if cam is not None and not cam.ok:
                        bits.append(f"[{cam.role} DOWN]")
                if bits:
                    print("  |  ".join(bits))

            # Keys
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("s"):
                if intake_spec:
                    save_raw(INTAKE_CONFIG, intake_raw)
                if separator_spec:
                    save_raw(SEPARATOR_CONFIG, separator_raw)
            elif key == ord("1") and intake_spec:
                save_raw(INTAKE_CONFIG, intake_raw)
            elif key == ord("2") and separator_spec:
                save_raw(SEPARATOR_CONFIG, separator_raw)
            elif key == ord("r"):
                intake_raw = load_raw(INTAKE_CONFIG, INTAKE_DEFAULTS)
                separator_raw.update(load_raw(SEPARATOR_CONFIG, SEPARATOR_DEFAULTS))
                if intake_spec:
                    push_trackbars(INTAKE_TUNE_WIN, intake_raw, intake_spec)
                if separator_spec:
                    push_trackbars(SEPARATOR_TUNE_WIN, separator_raw, separator_spec)
            elif key == ord("p"):
                print("\n--- orin_config.json ---")
                print(json.dumps({k: int(v) for k, v in intake_raw.items()}, indent=2))
                print("--- separator_config.json ---")
                print(json.dumps({k: int(v) for k, v in separator_raw.items()}, indent=2))
                print()
            elif key == ord("c"):
                mode = intake_calib.cycle()
                separator_calib.cycle()
                print(f"[key] autocalib -> {mode.upper()} (both cameras)")
                for label, harness, manual in (
                        ("intake", intake_calib, derive_intake(intake_raw)),
                        ("separator", separator_calib,
                         derive_separator(separator_raw))):
                    for line in harness.lines(manual):
                        print(f"       {label}: {line}")
            elif key == ord("d"):
                drag["enabled"] = not drag["enabled"]
                print(f"[roi] drag {'ON — draw a box on SEPARATOR' if drag['enabled'] else 'off'}")
            elif key == ord("h"):
                tune_hidden = not tune_hidden
                for win, spec, raw in ((INTAKE_TUNE_WIN, intake_spec, intake_raw),
                                       (SEPARATOR_TUNE_WIN, separator_spec, separator_raw)):
                    if not spec:
                        continue
                    if tune_hidden:
                        cv2.destroyWindow(win)
                    else:
                        build_trackbars(win, raw, spec)

    except KeyboardInterrupt:
        pass
    finally:
        for cam in (intake, separator):
            if cam is not None:
                cam.release()
        cv2.destroyAllWindows()
        print(f"\n[stopped] intake hits L={hits['left']} R={hits['right']}   "
              f"separator warm={hits['warm']} cool={hits['cool']}")
        print("[reminder] sudo systemctl start larc-dispatcher")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
