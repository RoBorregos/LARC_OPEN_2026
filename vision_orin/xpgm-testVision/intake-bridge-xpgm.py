#!/usr/bin/env python3
"""intake-bridge-xpgm.py -- feeds dispatcher.py's "intake" role with the
SAME decision logic dashboard-xpgm.py already visualizes live (median-
filtered color at the best candidate's point + the horizontal "gate"),
instead of the production orin_vision.py / orin_vision_model.py.

Only used when dispatcher.py's USE_XPGM_TEST flag is True (see
vision_orin/dispatcher/dispatcher.py). Default there is False, so this file
existing changes nothing on its own.

Contract dispatcher.py expects on stdout, one line per frame, unbuffered
(see dispatcher.py's own docstring -- "Script contract"):

    VISION:XX     bit 0 = intake upper, bit 1 = intake lower

Reuses detector-xpgm.py's detect_single_zone() / DEFAULT exactly like
dashboard-xpgm.py does (importlib, no live import of orin_vision.py --
keeps the xpgm-testVision isolation rule). best_candidate() / median_last_n()
/ reclassify_hue() / the GATE_X+GATE_Y_TOL check below are copied from
dashboard-xpgm.py's own update() -- same decision, not detector-xpgm.py's
own Confirmation class (that one tracks position across frames instead;
dashboard-xpgm.py never used it for the BAJAR verdict, so neither does
this). dashboard-xpgm.py itself is NOT imported here: it pulls in
matplotlib with a TkAgg backend at import time, which needs a desktop --
unsafe for a script dispatcher.py launches headless as a systemd service.

GATE_X widened from dashboard-xpgm.py's (0.35, 0.65) to (0.20, 0.80) per
request -- the ball has more horizontal room before BAJAR fires. Change the
two numbers below and restart if you want it narrower/wider again; nothing
else needs to change.

Camera is opened by role via link/camera_select.py (USB identity only, not
detection logic) instead of detector-xpgm.py's own --source/cv2.VideoCapture
argument handling.
"""
import importlib.util
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
LINK_DIR = HERE.parent / 'link'

# Same gate dashboard-xpgm.py draws as the yellow dashed horizontal line,
# widened -- see module docstring. GATE_Y_TOL widened too: real readings on
# "abajo" sat at fy=0.59, just outside the original 0.42-0.58 window.
# GATE_X shifted right to read before true center and buy lead time
# against latency. Pushed all the way to (0.40,1.00) (the hard edge) at one
# point, backed off one step to here.
GATE_X = (0.38, 0.98)
GATE_Y_TOL = 0.18
MEDIAN_N = 5

# dashboard-xpgm.py's own gate check has zero temporal hold -- fine for a
# ball held still in front of the camera, not for one crossing the gate in
# a couple of 15fps frames while the robot is moving: BAJAR blips true for
# 1-2 frames then drops, and the servo never commits. Same fix dispatcher.py
# already uses for the separator (SEPARATOR_HOLD_MS): once BAJAR fires,
# hold it true for HOLD_SEC even if the next frames lose the gate.
HOLD_SEC = 0.5

# CALIBRATION SAFE MODE: when True, every [CALIB]/[TRIGGER]/[DEBUG] line
# still prints normally (so the color calibration pass works exactly the
# same), but the actual VISION:XX bitfield sent to dispatcher.py is forced
# to 0x00 every frame -- the servo never gets a "bajar" command no matter
# what the gate/color logic decides internally. Set back to False once the
# calibration pass is done and you want the servo live again.
CALIB_SAFE_MODE = False


def load_detector_xpgm():
    spec = importlib.util.spec_from_file_location("detector_xpgm", HERE / "detector-xpgm.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def open_intake_camera():
    """The only piece of this file that depends on code outside
    xpgm-testVision: USB-identity camera resolution, not detection logic --
    see module docstring."""
    sys.path.insert(0, str(LINK_DIR))
    from camera_select import open_role, CameraNotFound
    try:
        return open_role("intake")
    except CameraNotFound as exc:
        raise SystemExit(f"[ERROR] intake camera: {exc}")


def best_candidate(found):
    """Copied from dashboard-xpgm.py."""
    return found[0] if found else None


def _valid(vals):
    return [v for v in vals if not (isinstance(v, float) and np.isnan(v))]


def median_last_n(dq, n):
    """Copied from dashboard-xpgm.py (the Dr. Bortoni median-of-n-frames
    suggestion, ignoring gaps from ball-less frames)."""
    window = _valid(list(dq)[-n:])
    return statistics.median(window) if window else float('nan')


def reclassify_hue(h, s, v, cfg):
    """Copied from dashboard-xpgm.py. (Tried splitting out a 'negro'/
    'incierto' distinction here -- made blue worse and green came back, so
    reverted. Back to dashboard-xpgm.py's original rule as-is.)"""
    if s < cfg['min_ball_saturation'] or v < cfg['min_ball_value']:
        return 'otro'
    for color_name, hue_ranges in cfg['ball_colors'].items():
        if any(lo <= h <= hi for lo, hi in hue_ranges):
            return color_name
    return 'otro'


def main():
    dx = load_detector_xpgm()
    # detector-xpgm.py's own DEFAULT -- max_radius lives there now (raised
    # to 160 for close range), so dashboard-xpgm.py sees the same value.
    cfg = dx.DEFAULT

    zone_left = cfg['eye_zones']['left']
    zone_right = cfg['eye_zones']['right']
    eye_of_zone = {zone_left: 'left', zone_right: 'right'}
    zones = [zone_left, zone_right]

    cap, cam = open_intake_camera()

    from collections import deque
    hsv_buf = {zone: {ch: deque(maxlen=MEDIAN_N * 4) for ch in ('H', 'S', 'V')} for zone in zones}
    held_until = {zone: 0.0 for zone in zones}
    # Tracks the last candidate's (x, y, r) per zone so a position/radius
    # jump -- a DIFFERENT physical ball taking over the same zone, not the
    # same one moving a little -- clears hsv_buf before the new ball's
    # first sample goes in. Without this, the median blends the tail of
    # one ball's H/S/V with the head of the next one's (e.g. a blue ball
    # leaving right as a green one arrives), producing a transitional
    # reading that belongs to neither -- confirmed in the 01:49:25-26 log:
    # h=121 (azul) directly followed by h=76 (verde) in the same zone with
    # no gap between them. Same position/radius tolerance as
    # detector-xpgm.py's own Confirmation class (0.6*r, 0.3*r).
    last_pos = {zone: None for zone in zones}

    last_debug = 0.0
    frame_count = 0
    try:
        while True:
            loop_t0 = time.monotonic()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            w = frame.shape[1]
            halves = {'left': frame[:, :w // 2], 'right': frame[:, w // 2:]}

            now = time.monotonic()
            bajar = {}
            debug_info = {}
            for zone in zones:
                crop = halves[eye_of_zone[zone]]
                img, found = dx.detect_single_zone(crop, cfg)
                c = best_candidate(found)

                if c is None:
                    held = now < held_until[zone]
                    bajar[zone] = held
                    debug_info[zone] = "sin_pelota" + (":HOLD" if held else "")
                    last_pos[zone] = None
                    continue

                prev = last_pos[zone]
                is_new_ball = (
                    prev is None
                    or ((c['x'] - prev[0]) ** 2 + (c['y'] - prev[1]) ** 2) ** 0.5 > 0.6 * prev[2]
                    or abs(c['r'] - prev[2]) > 0.3 * prev[2]
                )
                if is_new_ball:
                    hsv_buf[zone]['H'].clear()
                    hsv_buf[zone]['S'].clear()
                    hsv_buf[zone]['V'].clear()
                last_pos[zone] = (c['x'], c['y'], c['r'])

                x, y = round(c['x']), round(c['y'])
                x = min(max(x, 0), img.shape[1] - 1)
                y = min(max(y, 0), img.shape[0] - 1)
                bgr = img[y, x]
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[y, x]
                hsv_buf[zone]['H'].append(int(hsv[0]))
                hsv_buf[zone]['S'].append(int(hsv[1]))
                hsv_buf[zone]['V'].append(int(hsv[2]))

                med_h = median_last_n(hsv_buf[zone]['H'], MEDIAN_N)
                med_s = median_last_n(hsv_buf[zone]['S'], MEDIAN_N)
                med_v = median_last_n(hsv_buf[zone]['V'], MEDIAN_N)
                color = reclassify_hue(med_h, med_s, med_v, cfg)

                fx = c['x'] / img.shape[1]
                fy = c['y'] / img.shape[0]
                # CALIB (temporary): every candidate, every frame, whatever
                # its color -- for sampling real H/S/V while driving past
                # known balls to calibrate ball_colors with real numbers.
                # Purely additive: doesn't touch bajar/gate/servo logic at
                # all, just logs alongside it.
                print(f"[CALIB] {zone} color={color} h={med_h:.0f} s={med_s:.0f} v={med_v:.0f} "
                      f"r={c['r']:.0f} fx={fx:.2f} fy={fy:.2f}", flush=True)

                if color == 'verde':
                    # Absolute veto: verde means no, full stop -- even if a
                    # PREVIOUS non-green trigger left this zone inside its
                    # HOLD_SEC window right now. Confirmed in the 01:40:01-02
                    # log: a red ball triggered "abajo", then a green ball
                    # immediately after inherited that hold and read
                    # "lower=1" while h=77-79 (clean, unambiguous green) --
                    # not a color-calibration problem, this exact logic bug.
                    held_until[zone] = 0.0
                    bajar[zone] = False
                    debug_info[zone] = "VERDE"
                else:
                    in_gate = GATE_X[0] <= fx <= GATE_X[1] and abs(fy - 0.5) <= GATE_Y_TOL
                    if in_gate:
                        held_until[zone] = now + HOLD_SEC
                        # Logged immediately, not throttled to 1/sec like the
                        # rest of [DEBUG] -- this is the one line that answers
                        # "what exactly triggered BAJAR", so a fast trigger
                        # between two throttled samples can't go unseen.
                        print(f"[TRIGGER] {zone} {color}:h={med_h:.0f}:s={med_s:.0f}:v={med_v:.0f}:"
                              f"r={c['r']:.0f}:fx={fx:.2f}:fy={fy:.2f}",
                              flush=True)
                    held = now < held_until[zone]
                    bajar[zone] = in_gate or held
                    tag = 'BAJAR' if in_gate else ('HOLD' if held else 'esperando')
                    debug_info[zone] = f"{color}:r={c['r']:.0f}:fx={fx:.2f}:fy={fy:.2f}:{tag}"

            upper = int(bajar.get('arriba', False))
            lower = int(bajar.get('abajo', False))
            bitfield = upper | (lower << 1)
            if CALIB_SAFE_MODE:
                bitfield = 0

            print(f"VISION:{bitfield:02x}", flush=True)

            frame_count += 1
            now = time.monotonic()
            if now - last_debug >= 1.0:
                last_debug = now
                loop_ms = (time.monotonic() - loop_t0) * 1000
                print(f"[DEBUG] fps~{frame_count} arriba={debug_info.get('arriba')} "
                      f"abajo={debug_info.get('abajo')} loop_ms={loop_ms:.0f}", flush=True)
                frame_count = 0
    finally:
        cap.release()


if __name__ == '__main__':
    main()
