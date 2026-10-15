#!/usr/bin/env python3
"""
intake_autocalib.py — continuous photometric autocalibration for the intake

Purpose : Track the illuminant and correct s_min, v_dark, bg_h_lo, bg_h_hi,
          bg_s_min, bg_v_min.
Usage
    python3 intake_autocalib.py --seed      guided, writes the seed file
    python3 intake_autocalib.py --report    watch it track, change nothing
"""

from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "link"))

from autocalib import Calibrator, Clamped          # noqa: E402

SEED_FILE = os.path.join(HERE, "intake_calib_seed.json")

HUE_V_FLOOR = 40.
BG_GATHER = 18
HUE_SEARCH = 30
MIN_BG_FRAC = 0.25
CALIB_SCALE = 0.25


# measurement
def background_stats(frame, seed_mode=None):

    small = cv2.resize(frame, None, fx=CALIB_SCALE, fy=CALIB_SCALE,
                       interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    lit = v >= HUE_V_FLOOR
    n_lit = int(np.count_nonzero(lit))
    if n_lit < 500:
        return None, "frame too dark to read"
    hist = cv2.calcHist([hsv], [0], lit.astype(np.uint8), [180], [0, 180])
    hist = cv2.GaussianBlur(hist, (1, 9), 0).ravel()
    mode = int(np.argmax(hist))

    if seed_mode is not None and abs(mode - seed_mode) > HUE_SEARCH:
        return None, f"background hue {mode} far from seed {seed_mode}"

    band = lit & (np.abs(h.astype(np.int16) - mode) <= BG_GATHER)
    n_bg = int(np.count_nonzero(band))
    if n_bg < MIN_BG_FRAC * n_lit:
        return None, f"background only {100.0 * n_bg / n_lit:.0f}% of frame"

    bg_s = s[band]
    bg_v = v[band]
    bg_h = h[band].astype(np.float32)

    return {
        "h_mode":  float(mode),
        "bg_s_hi": float(np.percentile(bg_s, 98)),
        "bg_v_lo": float(np.percentile(bg_v, 2)),
        "bg_h_lo": float(np.percentile(bg_h, 2)),
        "bg_h_hi": float(np.percentile(bg_h, 98)),
        "bg_frac": n_bg / n_lit,
    }, "ok"


def bean_stats(frame, bg_mode):
    """Measure the beans in one frame of the known-bean seed phase:
    everything that is NOT the background."""
    small = cv2.resize(frame, None, fx=CALIB_SCALE, fy=CALIB_SCALE,
                       interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    not_bg = np.abs(h.astype(np.int16) - bg_mode) > BG_GATHER
    if int(np.count_nonzero(not_bg)) < 200:
        return None
    return {
        "bean_s_lo": float(np.percentile(s[not_bg], 20)),
        "bean_v_hi": float(np.percentile(v[not_bg], 80)),
        "bean_frac": float(np.count_nonzero(not_bg)) / not_bg.size,
    }


# seed
def derive_seed(cfg, bg, bean=None):
    if bean is not None and bean["bean_s_lo"] > bg["bg_s_hi"]:
        s_min = 0.5 * (bg["bg_s_hi"] + bean["bean_s_lo"])
        source = "measured separation"
    else:
        s_min = float(cfg["s_min"])
        source = "hand-tuned s_min"

    return {
        "_source": source,
        "h_mode":     bg["h_mode"],
        "bg_s_hi":    bg["bg_s_hi"],
        "bg_v_lo":    bg["bg_v_lo"],
        "s_margin":      s_min - bg["bg_s_hi"],
        "v_margin":      float(cfg["v_dark"]) - bg["bg_v_lo"],
        "bg_h_lo_off":   float(cfg["bg_h_lo"]) - bg["h_mode"],
        "bg_h_hi_off":   float(cfg["bg_h_hi"]) - bg["h_mode"],
        "s_min":  s_min,
        "v_dark": float(cfg["v_dark"]),
    }


def load_seed(cfg):
    if os.path.exists(SEED_FILE):
        with open(SEED_FILE) as f:
            seed = json.load(f)
        return seed, f"seed file ({seed.get('_source', '?')})"
    return None, "no seed file — self-seeding from config on first frames"


# calibrator
class IntakeCalibrator(Calibrator):

    LIMITS = {
        "s_min":    (28, 0.05),
        "v_dark":   (22, 0.05),
        "bg_h_lo":  (12, 0.04),
        "bg_h_hi":  (12, 0.04),
        "bg_s_min": (10, 0.04),
        "bg_v_min": (10, 0.04),
    }

    def __init__(self, cfg, seed=None, hz=8.0, **kw):
        self.seed = seed
        self._pending_seed = seed is None
        super().__init__(cfg, hz=hz, name="intake-calib", **kw)

    def _build_params(self, cfg):
        params = {}
        for key, (max_delta, alpha) in self.LIMITS.items():
            if key not in cfg:
                continue
            hi = 180 if key.startswith("bg_h") else 255
            params[key] = Clamped(key, cfg[key], max_delta, alpha=alpha, hi=hi)
        return params

    def _estimate(self, frame):
        seed_mode = self.seed["h_mode"] if self.seed else None
        bg, reason = background_stats(frame, seed_mode)
        if bg is None:
            return None, reason

        if self._pending_seed:
            self.seed = derive_seed(self.seed_cfg, bg)
            self._pending_seed = False

        s = self.seed
        h_mode = bg["h_mode"]
        return {
            # Track the background, hold the learned margin.
            "s_min":   bg["bg_s_hi"] + s["s_margin"],
            "v_dark":  bg["bg_v_lo"] + s["v_margin"],
            # Hue band rides the mode.
            "bg_h_lo": h_mode + s["bg_h_lo_off"],
            "bg_h_hi": h_mode + s["bg_h_hi_off"],
            # Floors follow the background's own spread.
            "bg_s_min": max(0.0, bg["bg_s_hi"] * 0.05),
            "bg_v_min": max(0.0, bg["bg_v_lo"] * 0.5),
        }, "ok"

    def _postprocess(self, cfg):
        if cfg["bg_h_lo"] >= cfg["bg_h_hi"]:
            cfg["bg_h_lo"] = self.seed_cfg["bg_h_lo"]
            cfg["bg_h_hi"] = self.seed_cfg["bg_h_hi"]
        # s_min at 0 would make every pixel a candidate.
        cfg["s_min"] = max(8, cfg["s_min"])
        cfg["v_dark"] = max(1, cfg["v_dark"])
        return cfg


# guided seed routine
def run_seed(cfg_path="orin_config.json"):
    sys.path.insert(0, os.path.join(HERE, "..", "link"))
    from camera_select import open_role, CameraNotFound

    with open(cfg_path) as f:
        raw = json.load(f)
    cfg = {k: v for k, v in raw.items() if not k.startswith("_")}

    try:
        cap, cam = open_role("intake")
    except CameraNotFound as exc:
        sys.exit(f"[ERROR] intake camera: {exc}")
    print(f"[Camera] {cam.describe()}")

    def capture(prompt, n=40):
        input(f"\n{prompt}\n  press ENTER when ready... ")
        frames = []
        while len(frames) < n:
            ok, frame = cap.read()
            if ok and frame is not None:
                frames.append(frame)
        print(f"  captured {len(frames)} frames")
        return frames

    empty = capture("PHASE A — clear the intake completely (no beans).")
    bg_list = [b for b, _ in (background_stats(f) for f in empty) if b]
    if not bg_list:
        sys.exit("[ERROR] could not read a background from phase A")
    bg = {k: float(np.median([b[k] for b in bg_list])) for k in bg_list[0]}
    print(f"  background hue mode {bg['h_mode']:.0f}  "
          f"S98 {bg['bg_s_hi']:.0f}  V02 {bg['bg_v_lo']:.0f}")

    beans = capture("PHASE B — place several beans in the detection zone.")
    bn_list = [b for b in (bean_stats(f, bg["h_mode"]) for f in beans) if b]
    bean = None
    if bn_list:
        bean = {k: float(np.median([b[k] for b in bn_list])) for k in bn_list[0]}
        print(f"  beans S20 {bean['bean_s_lo']:.0f}  V80 {bean['bean_v_hi']:.0f}  "
              f"cover {100 * bean['bean_frac']:.1f}%")
    else:
        print("  no beans distinguishable — falling back to hand-tuned s_min")

    seed = derive_seed(cfg, bg, bean)
    with open(SEED_FILE, "w") as f:
        json.dump(seed, f, indent=2)
        f.write("\n")

    cap.release()
    print(f"\n[Seed] written to {SEED_FILE}  ({seed['_source']})")
    print(f"  s_min  {cfg['s_min']} -> {seed['s_min']:.0f}   "
          f"(margin {seed['s_margin']:+.0f} above background S98)")
    print(f"  v_dark {cfg['v_dark']} -> {seed['v_dark']:.0f}   "
          f"(margin {seed['v_margin']:+.0f} from background V02)")
    if bean and bean["bean_s_lo"] <= bg["bg_s_hi"]:
        print("  WARNING: beans are not more saturated than the background."
              "\n           Autocalibration cannot fix that — it is a lighting"
              "\n           or background-colour problem. Fix it optically.")


def main():
    if "--seed" in sys.argv:
        os.chdir(HERE)
        return run_seed()
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
