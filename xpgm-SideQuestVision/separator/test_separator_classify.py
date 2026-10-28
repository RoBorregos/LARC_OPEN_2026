#!/usr/bin/env python3
"""
test_separator_classify.py — regression test for the separator classifier

Purpose : Run the REAL masks_for() / classify() from separator_vision.py
          against synthetic ROIs (a coloured ball on a white background) and
          assert the bin each colour lands in.
Why     : a GREEN ball used to match neither the warm nor the cool hue band,
          so the script emitted VISION:FD:00:00. The dispatcher reads 00:00
          as "no clean hit", lets SEPARATOR_HOLD_MS expire and parks the
          servo at SEP_NEUTRAL — the ball stays on the gate and the pipeline
          stops.
Needs   : no camera, no Teensy
Usage   : python3 test_separator_classify.py
"""
import os, sys, json
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

from separator_vision import masks_for, classify, _pct, DEFAULT_CFG

ROI = 200
RADIUS = 80          # ~50% of the ROI -> below white_frac, so "ball present"


def cfg_from_file():
    with open("separator_config.json") as f:
        saved = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    return {**DEFAULT_CFG, **saved}


def ball(bgr):
    """White background with a coloured ball in the middle -> HSV ROI."""
    img = np.full((ROI, ROI, 3), 255, np.uint8)
    cv2.circle(img, (ROI // 2, ROI // 2), RADIUS, bgr, -1)
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def empty():
    return cv2.cvtColor(np.full((ROI, ROI, 3), 255, np.uint8), cv2.COLOR_BGR2HSV)


# (label, roi, expected warm bit, expected cool bit, expected no_ball)
CASES = [
    ("no ball (white)",      empty(),                False, False, True),
    ("red   -> mature",      ball((0, 0, 255)),      True,  False, False),
    ("orange-> mature",      ball((0, 140, 255)),    True,  False, False),
    ("yellow-> mature",      ball((0, 255, 255)),    True,  False, False),
    ("GREEN -> mature",      ball((0, 255, 0)),      True,  False, False),
    ("dark green -> mature", ball((40, 120, 40)),    True,  False, False),
    ("lime  -> mature",      ball((60, 220, 140)),   True,  False, False),
    ("blue  -> immature",    ball((255, 0, 0)),      False, True,  False),
    ("cyan-blue -> immature", ball((200, 120, 0)),   False, True,  False),
    ("black -> immature",    ball((15, 15, 15)),     False, True,  False),
    ("magenta -> NEUTRAL",   ball((255, 0, 255)),    False, False, False),
]


def main():
    cfg = cfg_from_file()
    print(f"green band  H {cfg['green_h_lo']}-{cfg['green_h_hi']}  "
          f"S>={cfg['green_s_min']}  V>={cfg['green_v_min']}  "
          f"frac>={cfg['green_frac']}%\n")

    failures = 0
    for label, roi, exp_w, exp_c, exp_none in CASES:
        pct = {n: _pct(m, ROI * ROI) for n, m in masks_for(roi, cfg).items()}
        warm, cool, no_ball, reason = classify(pct, cfg)

        ok = (warm, cool, no_ball) == (exp_w, exp_c, exp_none)
        failures += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {label:24s} "
              f"-> {int(warm)}:{int(cool)}  {reason:28s} "
              f"[w{pct['warm']:5.1f} c{pct['cool']:5.1f} "
              f"g{pct['green']:5.1f} wh{pct['white']:5.1f}]")
        if not ok:
            print(f"       expected {int(exp_w)}:{int(exp_c)} no_ball={exp_none}")

    # Every ball of a KNOWN colour must set a bit. A colour we recognise
    # emitting 00:00 is the original stall bug.
    print("\n  stall check — every known-colour ball must set a bit:")
    stalls = 0
    for label, roi, exp_w, exp_c, exp_none in CASES:
        if exp_none or not (exp_w or exp_c):
            continue                      # skip empty ROI and the unknown case
        pct = {n: _pct(m, ROI * ROI) for n, m in masks_for(roi, cfg).items()}
        warm, cool, no_ball, _ = classify(pct, cfg)
        if not warm and not cool and not no_ball:
            print(f"    FAIL {label} emits 00:00 -> SEP_NEUTRAL -> stall")
            stalls += 1
    failures += stalls
    if not stalls:
        print("    ok   every known-colour ball sets a bit")

    # unknown_to_mature is the switch between the two readings of "nothing
    # matched". Default 0 assumes the background; 1 assumes an odd ball.
    print("\n  unknown routing:")
    unknown = ball((255, 0, 255))
    pct = {n: _pct(m, ROI * ROI) for n, m in masks_for(unknown, cfg).items()}

    neutral = dict(cfg, unknown_to_mature=0)
    w, c, nb, reason = classify(pct, neutral)
    ok = (w, c, nb) == (False, False, False)
    print(f"  {'ok  ' if ok else 'FAIL'} unknown_to_mature=0 -> NEUTRAL   {reason}")
    failures += not ok

    mature = dict(cfg, unknown_to_mature=1)
    w, c, nb, reason = classify(pct, mature)
    ok = (w, c, nb) == (True, False, False)
    print(f"  {'ok  ' if ok else 'FAIL'} unknown_to_mature=1 -> MATURE    {reason}")
    failures += not ok

    # A cream/beige gate is the real-world case that made NEUTRAL the
    # default: it fails the white test, so it matches nothing at all.
    cream = np.full((ROI, ROI, 3), 0, np.uint8)
    cream[:, :] = (196, 214, 226)          # BGR, a warm off-white surface
    cream_hsv = cv2.cvtColor(cream, cv2.COLOR_BGR2HSV)
    pct = {n: _pct(m, ROI * ROI) for n, m in masks_for(cream_hsv, cfg).items()}
    w, c, nb, reason = classify(pct, cfg)
    ok = not w and not c
    print(f"  {'ok  ' if ok else 'FAIL'} cream background never routes to a bin   "
          f"{reason}  [white {pct['white']:.0f}%]")
    failures += not ok

    print("\n" + ("ALL PASS" if not failures else f"{failures} FAILURE(S)"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
