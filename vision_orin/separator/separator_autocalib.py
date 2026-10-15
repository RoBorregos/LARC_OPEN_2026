#!/usr/bin/env python3
"""
separator_autocalib.py — white reference autocalibration for the separator

Purpose : Adapt all photometric parameters, hue bands included, off the ROI
          background — which is WHITE, and so reports the illuminant
          directly. A far stronger signal than anything the intake has.
Cadence : updates ONLY on confident no-ball frames, when the ROI is a clean,
          uncontaminated look at that white reference.
Failure : freeze at the last good values. Never revert, never keep adapting
          on data we do not trust.
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

SEED_FILE = os.path.join(HERE, "separator_calib_seed.json")

IDLE_S_MAX = 90
IDLE_V_MIN = 45 
IDLE_S_FRAC = 0.70
HUE_MAX_SHIFT = 6 
GUARD_GAP = 1
# Green must stay wide enough to actually catch a green ball after any
# shift, not collapse into a slit.
MIN_GREEN_WIDTH = 20

CLASSES = ("warm", "cool", "green")


# measurement
def white_reference(roi_bgr):
    """Measure the illuminant from an idle ROI.

    Returns (stats, reason); stats is None when the ROI is not a
    trustworthy look at the white background.
    """
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]

    s_med = float(np.median(s))
    v_med = float(np.median(v))
    unsat = float(np.count_nonzero(s <= IDLE_S_MAX)) / s.size

    if v_med < IDLE_V_MIN:
        return None, f"ROI too dark (V med {v_med:.0f})"
    if s_med > IDLE_S_MAX or unsat < IDLE_S_FRAC:
        return None, f"ROI not idle (S med {s_med:.0f}, {100 * unsat:.0f}% unsat)"
    lo, hi = np.percentile(v, [10, 90])
    core = (v >= lo) & (v <= hi)
    if int(np.count_nonzero(core)) < 200:
        return None, "not enough background pixels"

    px = roi_bgr[core].reshape(-1, 3).astype(np.float32)
    return {
        "white_bgr": px.mean(axis=0).tolist(), # the illuminant
        "v_med": v_med,
        "s_med": s_med,
        # Where the white background actually sits, so the no-ball
        # thresholds can follow it.
        "white_v_p05": float(np.percentile(v, 5)),
        "white_s_p95": float(np.percentile(s, 95)),
    }, "ok"


def _hue_of(bgr):
    px = np.array([[list(bgr)]], dtype=np.float32).clip(0, 255).astype(np.uint8)
    return float(cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0, 0, 0])


def predicted_hue_shift(seed, white_now):
    ref_white = np.array(seed["white_bgr"], np.float32)
    now_white = np.array(white_now, np.float32)
    if float(np.min(ref_white)) < 1.0:
        return None, "seed white reference is degenerate"

    gain = now_white / np.maximum(ref_white, 1.0)
    shifts = {}
    for cls in CLASSES:
        ref = seed.get("balls", {}).get(cls)
        if ref is None:
            continue
        now_hue = _hue_of(np.array(ref, np.float32) * gain)
        delta = now_hue - _hue_of(ref)
        if delta > 90:
            delta -= 180
        elif delta < -90:
            delta += 180
        shifts[cls] = float(delta)
    if not shifts:
        return None, "no reference balls in seed"
    return shifts, "ok"


# calibrator
class SeparatorCalibrator(Calibrator):
    LIMITS = {
        "white_v_min": (45, 0.06),
        "white_s_max": (25, 0.06),
        "warm_s_min":  (25, 0.04),
        "warm_v_min":  (35, 0.04),
        "cool_s_min":  (25, 0.04),
        "cool_v_min":  (30, 0.04),
        "green_s_min": (25, 0.04),
        "green_v_min": (30, 0.04),
        "black_v_max": (25, 0.04),
        "warm_h_hi":   (HUE_MAX_SHIFT, 0.02),
        "cool_h_lo":   (HUE_MAX_SHIFT, 0.02),
        "cool_h_hi":   (HUE_MAX_SHIFT, 0.02),
        "green_h_lo":  (HUE_MAX_SHIFT, 0.02),
        "green_h_hi":  (HUE_MAX_SHIFT, 0.02),
    }
    HUE_KEYS = ("warm_h_hi", "cool_h_lo", "cool_h_hi", "green_h_lo", "green_h_hi")

    def __init__(self, cfg, seed=None, hz=8.0, **kw):
        self.seed = seed
        self.hue_enabled = bool(seed and seed.get("balls"))
        self.guard_trips = 0
        super().__init__(cfg, hz=hz, name="sep-calib", **kw)

    def _build_params(self, cfg):
        params = {}
        for key, (max_delta, alpha) in self.LIMITS.items():
            if key not in cfg:
                continue
            hi = 180 if key.endswith(("_h_lo", "_h_hi")) else 255
            params[key] = Clamped(key, cfg[key], max_delta, alpha=alpha, hi=hi)
        return params

    def _estimate(self, roi_bgr):
        ref, reason = white_reference(roi_bgr)
        if ref is None:
            return None, reason

        if self.seed is None:
            self.seed = {
                "white_bgr": ref["white_bgr"],
                "v_med": ref["v_med"],
                "white_v_p05": ref["white_v_p05"],
                "white_s_p95": ref["white_s_p95"],
                "v_margin": self.seed_cfg["white_v_min"] - ref["white_v_p05"],
                "s_margin": self.seed_cfg["white_s_max"] - ref["white_s_p95"],
            }

        s = self.seed
        est = {
            "white_v_min": ref["white_v_p05"] + s.get("v_margin", 0.0),
            "white_s_max": ref["white_s_p95"] + s.get("s_margin", 0.0),
        }
        ratio = ref["v_med"] / max(s.get("v_med", ref["v_med"]), 1.0)
        for key in ("warm_v_min", "cool_v_min", "green_v_min", "black_v_max"):
            if key in self.seed_cfg:
                est[key] = self.seed_cfg[key] * ratio
        s_bias = max(0.0, ref["white_s_p95"]
                     - s.get("white_s_p95", ref["white_s_p95"]))
        for key in ("warm_s_min", "cool_s_min", "green_s_min"):
            if key in self.seed_cfg:
                est[key] = self.seed_cfg[key] + s_bias
        if self.hue_enabled:
            shifts, why = predicted_hue_shift(s, ref["white_bgr"])
            if shifts is None:
                self.hue_enabled = False
                self.last_reason = f"hue locked — {why}"
            else:
                warm_d = shifts.get("warm", 0.0)
                cool_d = shifts.get("cool", 0.0)
                green_d = shifts.get("green", 0.0)
                b_warm_green = 0.5 * (warm_d + green_d)
                b_green_cool = 0.5 * (green_d + cool_d)
                for key, delta in (("warm_h_hi",  b_warm_green),
                                   ("green_h_lo", b_warm_green),
                                   ("green_h_hi", b_green_cool),
                                   ("cool_h_lo",  b_green_cool),
                                   ("cool_h_hi",  cool_d)):
                    if key in self.seed_cfg:
                        est[key] = self.seed_cfg[key] + delta

        return est, "ok"

    def _postprocess(self, cfg):
        """Hard ordering guard. Nothing published may let green fall back
        into the gap that stalled the gate."""
        ok = (cfg["warm_h_hi"] + GUARD_GAP <= cfg["green_h_lo"] and
              cfg["green_h_hi"] - cfg["green_h_lo"] >= MIN_GREEN_WIDTH and
              cfg["green_h_hi"] + GUARD_GAP <= cfg["cool_h_lo"] and
              cfg["cool_h_lo"] < cfg["cool_h_hi"])
        if not ok:
            for key in self.HUE_KEYS:
                if key in self.seed_cfg:
                    cfg[key] = self.seed_cfg[key]
                    param = self._params.get(key)
                    if param is not None:
                        param.reset()
            self.guard_trips += 1
            self.last_reason = "hue guard tripped — bands snapped to seed"
        cfg["white_v_min"] = max(60, cfg["white_v_min"])
        cfg["white_s_max"] = max(10, cfg["white_s_max"])
        return cfg

    def status(self):
        base = super().status()
        hue = "hue:on" if self.hue_enabled else "hue:LOCKED(no ball seed)"
        trips = f" guard-trips={self.guard_trips}" if self.guard_trips else ""
        return f"{base}  {hue}{trips}"


# guided seed routine
def run_seed():
    sys.path.insert(0, os.path.join(HERE, "..", "link"))
    from camera_select import open_role, CameraNotFound

    os.chdir(HERE)
    with open("separator_config.json") as f:
        raw = json.load(f)
    cfg = {k: v for k, v in raw.items() if not k.startswith("_")}

    try:
        cap, cam = open_role("separator")
    except CameraNotFound as exc:
        sys.exit(f"[ERROR] separator camera: {exc}")
    print(f"[Camera] {cam.describe()}")

    rx, ry = cfg["roi_x"], cfg["roi_y"]
    rw, rh = cfg["roi_w"], cfg["roi_h"]

    def grab(n=30):
        out = []
        while len(out) < n:
            ok, frame = cap.read()
            if ok and frame is not None:
                out.append(frame[ry:ry + rh, rx:rx + rw].copy())
        return out

    input("\nPHASE A — clear the separator gate (empty, white background)."
          "\n  press ENTER when ready... ")
    refs = [r for r, _ in (white_reference(roi) for roi in grab()) if r]
    if not refs:
        sys.exit("[ERROR] could not read the white background — check the ROI "
                 "and lighting")
    white = np.median([r["white_bgr"] for r in refs], axis=0).tolist()
    seed = {
        "white_bgr": white,
        "v_med": float(np.median([r["v_med"] for r in refs])),
        "white_v_p05": float(np.median([r["white_v_p05"] for r in refs])),
        "white_s_p95": float(np.median([r["white_s_p95"] for r in refs])),
        "balls": {},
    }
    seed["v_margin"] = cfg["white_v_min"] - seed["white_v_p05"]
    seed["s_margin"] = cfg["white_s_max"] - seed["white_s_p95"]
    print(f"  white reference BGR {[round(c) for c in white]}  "
          f"V med {seed['v_med']:.0f}")

    print("\nPHASE B — reference balls. These are what let hue bands adapt"
          "\n          at all; skip a class and hue stays locked.")
    for cls in CLASSES:
        answer = input(f"  place a {cls.upper()} ball on the gate "
                       f"(ENTER = capture, 's' = skip): ").strip().lower()
        if answer == "s":
            continue
        rois = grab(20)
        # Take the most saturated core of the ROI — that is the ball, not
        # the white surround.
        stack = np.concatenate([roi.reshape(-1, 3) for roi in rois])
        hsv = cv2.cvtColor(stack.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)
        sat = hsv[:, 0, 1]
        if cls == "cool":
            pick = (sat >= np.percentile(sat, 70)) | (hsv[:, 0, 2] <= 60)
        else:
            pick = sat >= np.percentile(sat, 85)
        if int(np.count_nonzero(pick)) < 100:
            print(f"    could not isolate the {cls} ball — skipped")
            continue
        med = stack[pick].astype(np.float32).mean(axis=0)
        seed["balls"][cls] = med.tolist()
        print(f"    {cls} reference BGR {[round(c) for c in med]}  "
              f"hue {_hue_of(med):.0f}")

    with open(SEED_FILE, "w") as f:
        json.dump(seed, f, indent=2)
        f.write("\n")
    cap.release()

    print(f"\n[Seed] written to {SEED_FILE}")
    if seed["balls"]:
        print(f"  hue adaptation ENABLED for: {', '.join(seed['balls'])}")
    else:
        print("  no reference balls captured — hue bands stay LOCKED to the "
              "config (photometric adaptation still runs)")


def load_seed():
    if os.path.exists(SEED_FILE):
        with open(SEED_FILE) as f:
            return json.load(f)
    return None


def main():
    if "--seed" in sys.argv:
        return run_seed()
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
