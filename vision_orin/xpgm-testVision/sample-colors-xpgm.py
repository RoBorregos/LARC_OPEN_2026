import importlib.util
import json

import cv2

spec = importlib.util.spec_from_file_location("detector_xpgm", "detector-xpgm.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

cfg = m.DEFAULT
frame = cv2.imread("live-frame-xpgm.png")
w = frame.shape[1]
img_left, found_left = m.detect_single_zone(frame[:, :w // 2], cfg)
img_right, found_right = m.detect_single_zone(frame[:, w // 2:], cfg)

zone_left = cfg['eye_zones']['left']
zone_right = cfg['eye_zones']['right']

results = []
for zone, img, found in [(zone_left, img_left, found_left), (zone_right, img_right, found_right)]:
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for c in found:
        x, y = round(c['x']), round(c['y'])
        bgr = [int(v) for v in img[y, x]]
        hsv = [int(v) for v in hsv_img[y, x]]
        results.append({"zone": zone, "x": c['x'], "y": c['y'], "r": c['r'],
                         "support": c['support'], "color_clasificado": c['color'],
                         "bgr": bgr, "hsv": hsv})

print(json.dumps(results, indent=2))
