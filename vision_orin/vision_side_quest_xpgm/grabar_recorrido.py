#!/usr/bin/env python3
import cv2, time, sys, datetime

duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
name = sys.argv[2] if len(sys.argv) > 2 else datetime.datetime.now().strftime("recorrido_%Y%m%d_%H%M%S")
outpath = f"{name}.avi"

cap = cv2.VideoCapture('/dev/video_zed')
if not cap.isOpened():
    print("ERROR: no se pudo abrir /dev/video_zed"); sys.exit(1)

ok, frame = cap.read()
if not ok:
    print("ERROR: no se pudo leer el primer frame"); sys.exit(1)
h, w = frame.shape[:2]
fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter(outpath, fourcc, 15.0, (w, h))

print(f"GRABANDO {outpath} ({w}x{h}) por {duration_s:.0f}s ...", flush=True)
t0 = time.monotonic()
frames = 0
while time.monotonic() - t0 < duration_s:
    ok, frame = cap.read()
    if not ok:
        continue
    writer.write(frame)
    frames += 1
writer.release()
cap.release()
elapsed = time.monotonic() - t0
print(f"LISTO: {outpath} | {frames} frames | {elapsed:.1f}s | ~{frames/elapsed:.1f} fps reales")
