#!/usr/bin/env python3
"""
camera_test_open.py — show every working camera feed in one window

Purpose : Find connected cameras and display all working feeds in a single
          grid. Works on macOS, Windows and Linux through OpenCV.
Controls: q or Esc quit, s save a snapshot of the complete grid

"""

from __future__ import annotations

import argparse
import math
import platform
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("This script needs OpenCV and NumPy.")
    print("Install them with:  python -m pip install opencv-python numpy")
    raise SystemExit(1)


WINDOW_NAME = "Connected cameras - q/Esc: quit, s: snapshot"
TILE_WIDTH = 640
TILE_HEIGHT = 360


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan for cameras and display all working feeds in a grid."
    )
    parser.add_argument(
        "indexes",
        metavar="INDEX",
        type=int,
        nargs="*",
        help="camera indexes to test (default: scan 0 through --max-index)",
    )
    parser.add_argument(
        "--max-index",
        type=int,
        default=9,
        help="highest camera index to scan when no indexes are given (default: 9)",
    )
    parser.add_argument("--width", type=int, help="requested capture width")
    parser.add_argument("--height", type=int, help="requested capture height")
    parser.add_argument(
        "--backend",
        choices=("auto", "avfoundation", "dshow", "msmf", "v4l2"),
        default="auto",
        help="OpenCV capture backend (default: best choice for this OS)",
    )
    return parser.parse_args()


def backend_for(name: str) -> tuple[int, str]:
    """Return the OpenCV backend constant and a readable name."""
    if name == "auto":
        system = platform.system()
        name = {
            "Darwin": "avfoundation",
            "Windows": "dshow",
            "Linux": "v4l2",
        }.get(system, "opencv-default")

    backends = {
        "avfoundation": (cv2.CAP_AVFOUNDATION, "AVFoundation"),
        "dshow": (cv2.CAP_DSHOW, "DirectShow"),
        "msmf": (cv2.CAP_MSMF, "Media Foundation"),
        "v4l2": (cv2.CAP_V4L2, "Video4Linux2"),
        "opencv-default": (cv2.CAP_ANY, "OpenCV default"),
    }
    return backends[name]


def open_camera(
    index: int, backend: int, width: int | None, height: int | None
) -> tuple[object, np.ndarray] | None:
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None

    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    frame = None
    for _ in range(15):
        ok, candidate = cap.read()
        if ok and candidate is not None and candidate.size:
            frame = candidate
            break
        time.sleep(0.03)

    if frame is None:
        cap.release()
        return None
    return cap, frame


def letterbox(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    source_h, source_w = frame.shape[:2]
    scale = min(width / source_w, height / source_h)
    resized_w = max(1, round(source_w * scale))
    resized_h = max(1, round(source_h * scale))
    resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_AREA)

    tile = np.zeros((height, width, 3), dtype=np.uint8)
    x = (width - resized_w) // 2
    y = (height - resized_h) // 2
    tile[y : y + resized_h, x : x + resized_w] = resized
    return tile


def add_label(tile: np.ndarray, label: str, healthy: bool = True) -> None:
    color = (40, 210, 80) if healthy else (50, 50, 230)
    cv2.rectangle(tile, (0, 0), (tile.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(
        tile,
        label,
        (12, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )


def make_grid(tiles: list[np.ndarray]) -> np.ndarray:
    columns = max(1, math.ceil(math.sqrt(len(tiles))))
    rows = math.ceil(len(tiles) / columns)
    blank = np.zeros_like(tiles[0])
    padded = tiles + [blank] * (rows * columns - len(tiles))
    return cv2.vconcat(
        [cv2.hconcat(padded[row * columns : (row + 1) * columns]) for row in range(rows)]
    )


def main() -> int:
    args = parse_args()
    if args.max_index < 0 or any(index < 0 for index in args.indexes):
        print("Camera indexes must be zero or greater.", file=sys.stderr)
        return 2

    backend, backend_name = backend_for(args.backend)
    indexes = args.indexes or list(range(args.max_index + 1))
    cameras: list[dict[str, object]] = []

    print(f"System: {platform.system()} {platform.release()}")
    print(f"Capture backend: {backend_name}")
    print(f"Testing camera indexes: {', '.join(map(str, indexes))}")
    print("On first run, your system may ask for camera permission.\n")

    try:
        for index in indexes:
            print(f"Camera {index}: testing...", end=" ", flush=True)
            opened = open_camera(index, backend, args.width, args.height)
            if opened is None:
                print("not available")
                continue

            cap, frame = opened
            height, width = frame.shape[:2]
            reported_fps = float(cap.get(cv2.CAP_PROP_FPS))
            fps_text = f"{reported_fps:.1f} FPS" if reported_fps > 0 else "FPS unknown"
            print(f"working - {width}x{height}, {fps_text}")
            cameras.append(
                {
                    "index": index,
                    "cap": cap,
                    "frame": frame,
                    "last_ok": time.monotonic(),
                    "frames": 0,
                    "measured_fps": 0.0,
                    "fps_started": time.monotonic(),
                }
            )

        if not cameras:
            print("\nNo working cameras were found.")
            print("Check camera permission, close other apps using the camera, or scan more indexes.")
            return 1

        print(f"\nFound {len(cameras)} working camera(s). Opening preview...")
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while True:
            tiles: list[np.ndarray] = []
            now = time.monotonic()

            for camera in cameras:
                cap = camera["cap"]
                ok, frame = cap.read()
                if ok and frame is not None and frame.size:
                    camera["frame"] = frame
                    camera["last_ok"] = now
                    camera["frames"] += 1

                elapsed = now - camera["fps_started"]
                if elapsed >= 1.0:
                    camera["measured_fps"] = camera["frames"] / elapsed
                    camera["frames"] = 0
                    camera["fps_started"] = now

                healthy = now - camera["last_ok"] < 1.0
                tile = letterbox(camera["frame"], TILE_WIDTH, TILE_HEIGHT)
                state = f"{camera['measured_fps']:.1f} FPS" if healthy else "NO SIGNAL"
                add_label(tile, f"Camera {camera['index']}  |  {state}", healthy)
                tiles.append(tile)

            grid = make_grid(tiles)
            cv2.imshow(WINDOW_NAME, grid)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                filename = Path.cwd() / f"camera_snapshot_{datetime.now():%Y%m%d_%H%M%S}.jpg"
                if cv2.imwrite(str(filename), grid):
                    print(f"Snapshot saved: {filename}")
                else:
                    print("Could not save snapshot.", file=sys.stderr)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for camera in cameras:
            camera["cap"].release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

