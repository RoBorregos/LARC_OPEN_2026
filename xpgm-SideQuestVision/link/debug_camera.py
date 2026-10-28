#!/usr/bin/env python3
"""
debug_camera.py — open the camera meant for the job, on any machine

Purpose : The debug tools run in two very different places. On the Orin,
          cameras.json matches a role to one physical unit by VID:PID and
          serial; on a Mac none of that exists, so this resolves the role by
          CAMERA NAME instead, using the same ordered device list
          AVFoundation gives OpenCV.
Used by : separator_debug.py, benefits_debug.py
Not used: by the production scripts. They run only on the Orin and go
          straight through camera_select, which fails loudly rather than
          guessing. Guessing is fine at a bench and wrong on a robot
          mid-match.

Resolution order — index order is not a promise, since a Continuity iPhone,
a Desk View camera or a Studio Display can shuffle everything after index 0
    1. --device N            you said exactly which one
    2. --name "C920"         you said what it is called
    3. name_contains in cameras.json for this role
    4. the first camera that is NOT a known Apple built-in
    5. the first camera that works, with a warning

Steps 2-4 need the device names in the order AVFoundation hands them to
OpenCV. Three ways to get that, best first:
    pyobjc  AVCaptureDevice, the exact list OpenCV walks
            pip3 install pyobjc-framework-AVFoundation
    ffmpeg  -f avfoundation -list_devices true, prints [N] Name
    system_profiler  names only, order is a guess, so it is used for
            labelling and never for selection
With none of them you still get the index scan, and --device.
"""

from __future__ import annotations

import json
import os
import platform
import re
import subprocess
import sys
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_select import CameraInfo, CameraNotFound, load_config, open_role  # noqa: E402

SYS_V4L = "/sys/class/video4linux"
SCAN_LIMIT = 6
WARMUP_FRAMES = 12

# Cameras that are part of the Mac rather than something you plugged in for
# this. Matched case-insensitively as substrings of the device name.
BUILTIN_PATTERNS = (
    "facetime", "built-in", "builtin", "isight",
    "iphone", "ipad", "continuity", "desk view",
    "studio display", "screen capture", "obs virtual",
)


def is_builtin(name: str) -> bool:
    lowered = (name or "").lower()
    return any(pattern in lowered for pattern in BUILTIN_PATTERNS)


# the ordered device list
def _names_via_pyobjc() -> list:
    try:
        import AVFoundation  # type: ignore
    except ImportError:
        return []
    try:
        devices = AVFoundation.AVCaptureDevice.devicesWithMediaType_(
            AVFoundation.AVMediaTypeVideo)
        return [str(d.localizedName()) for d in devices]
    except Exception:
        return []


def _names_via_ffmpeg() -> list:
    try:
        proc = subprocess.run(
            ["ffmpeg", "-hide_banner", "-f", "avfoundation",
             "-list_devices", "true", "-i", ""],
            capture_output=True, text=True, timeout=12)
    except Exception:
        return []

    names, seen_video = {}, False
    for line in (proc.stderr or "").splitlines():
        if "AVFoundation video devices" in line:
            seen_video = True
            continue
        if "AVFoundation audio devices" in line:
            break
        if not seen_video:
            continue
        match = re.search(r"\[(\d+)\]\s+(.+?)\s*$", line)
        if match:
            names[int(match.group(1))] = match.group(2)
    return [names[i] for i in sorted(names)] if names else []


def _names_via_system_profiler() -> list:
    try:
        out = subprocess.run(["system_profiler", "-json", "SPCameraDataType"],
                             capture_output=True, text=True, timeout=10).stdout
        return [e.get("_name", "?")
                for e in json.loads(out).get("SPCameraDataType", [])]
    except Exception:
        return []


def device_names() -> tuple:
    """(names, source). Empty list when nothing could enumerate them."""
    if platform.system() != "Darwin":
        return [], "n/a"
    for getter, source in ((_names_via_pyobjc, "AVFoundation"),
                           (_names_via_ffmpeg, "ffmpeg"),
                           (_names_via_system_profiler, "system_profiler")):
        names = getter()
        if names:
            return names, source
    return [], "none"


# opening
def _backend():
    import cv2
    return {"Darwin": cv2.CAP_AVFOUNDATION,
            "Windows": cv2.CAP_DSHOW}.get(platform.system(), cv2.CAP_V4L2)


def _try_index(index: int, width=None, height=None, fourcc=None):
    """Open one index and prove it delivers a frame. (cap, w, h) or None."""
    import cv2

    for backend in (_backend(), cv2.CAP_ANY):
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            cap.release()
            continue

        if fourcc:
            pack = getattr(cv2, "VideoWriter_fourcc", None) or cv2.VideoWriter.fourcc
            cap.set(cv2.CAP_PROP_FOURCC, pack(*fourcc))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        for _ in range(WARMUP_FRAMES):
            ok, frame = cap.read()
            if ok and frame is not None and frame.size:
                h, w = frame.shape[:2]
                return cap, w, h
        cap.release()
    return None


def scan(width=None, height=None, verbose: bool = True) -> list:
    """Every index that produces an image, as [(index, w, h, name, builtin)].
    Cameras are released again."""
    names, source = device_names()
    found = []

    if verbose:
        print(f"[Camera] scanning indices 0..{SCAN_LIMIT - 1} ({platform.system()})"
              + (f", names from {source}" if names else ""))
        if platform.system() == "Darwin" and not names:
            print("         (no name source — install pyobjc-framework-AVFoundation")
            print("          or ffmpeg to select cameras by name instead of index)")

    for index in range(SCAN_LIMIT):
        opened = _try_index(index, width, height)
        if opened is None:
            continue
        cap, w, h = opened
        cap.release()
        name = names[index] if index < len(names) else ""
        found.append((index, w, h, name, is_builtin(name)))
        if verbose:
            tag = "  [built-in]" if is_builtin(name) else ""
            print(f"  index {index}: {w}x{h}   {name or '(name unknown)'}{tag}")

    if verbose and not found:
        print("[Camera] nothing responded.")
        if platform.system() == "Darwin":
            print("         macOS needs permission on first access: System Settings")
            print("         > Privacy & Security > Camera, and allow Terminal (or")
            print("         whatever you launched this from).")
    return found


def _role_name_hint(role: str, config_path: Optional[str]) -> str:
    """The name_contains this role declares in cameras.json, if any."""
    try:
        return str(load_config(config_path).get(role, {}).get("name_contains", ""))
    except Exception:
        return ""


def open_for_debug(role: str, device=None, name: Optional[str] = None,
                   width=None, height=None, fourcc=None,
                   config_path: Optional[str] = None):
    """Return (cap, CameraInfo) on any machine. Raises CameraNotFound."""
    import cv2

    # 1. an explicit device always wins
    if device is not None:
        try:
            index = int(device)
        except (TypeError, ValueError):
            cap = cv2.VideoCapture(str(device), cv2.CAP_ANY)
            if not cap.isOpened():
                cap.release()
                raise CameraNotFound(f"could not open {device}")
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap, CameraInfo(device=str(device), card="explicit path")

        opened = _try_index(index, width, height, fourcc)
        if opened is None:
            raise CameraNotFound(f"index {index} did not deliver a frame. "
                                 f"Run with --list to see what does.")
        cap, w, h = opened
        names, _ = device_names()
        label = names[index] if index < len(names) else f"index {index}"
        print(f"[Camera] index {index} — {label}  ({w}x{h})")
        return cap, CameraInfo(device=f"index {index}", card=label)

    # 2. on the Orin, the role from cameras.json — the production path
    if os.path.isdir(SYS_V4L):
        return open_role(role, config_path, width=width, height=height,
                         fourcc=fourcc)

    # 3. everywhere else: resolve by NAME
    wanted = name or _role_name_hint(role, config_path)
    print(f"[Camera] no {SYS_V4L} here ({platform.system()}). Resolving the "
          f"'{role}' role by name" + (f": looking for '{wanted}'" if wanted
                                       else " (no name hint given)"))

    found = scan(width, height)
    if not found:
        raise CameraNotFound("no working camera found. Plug one in, or pass "
                             "--device N.")

    chosen = None
    if wanted:
        matches = [entry for entry in found
                   if wanted.lower() in (entry[3] or "").lower()]
        if len(matches) == 1:
            chosen = matches[0]
        elif len(matches) > 1:
            listing = "\n".join(f"      index {i}: {n}" for i, _, _, n, _ in matches)
            raise CameraNotFound(
                f"'{wanted}' matched {len(matches)} cameras:\n{listing}\n"
                f"    Pick one with --device N.")
        else:
            named = [entry for entry in found if entry[3]]
            if named:
                listing = "\n".join(f"      index {i}: {n}"
                                    for i, _, _, n, _ in found)
                print(f"[Camera] nothing matched '{wanted}'. Present:\n{listing}")
            # fall through to the built-in filter

    if chosen is None:
        external = [entry for entry in found if not entry[4]]
        if external:
            chosen = external[0]
            if len(found) > len(external):
                skipped = ", ".join(f"{i} ({n})" for i, _, _, n, b in found if b)
                print(f"[Camera] skipping built-in: {skipped}")
        else:
            chosen = found[0]
            if chosen[4]:
                print(f"[Camera] ! only a built-in camera responded "
                      f"({chosen[3]}). Is the USB camera plugged in and awake? "
                      f"Pass --device N to force one.")

    index, w, h, cam_name, builtin = chosen
    opened = _try_index(index, width, height, fourcc)
    if opened is None:
        raise CameraNotFound(f"index {index} worked during the scan and then "
                             f"would not reopen — is something else using it?")
    cap, w, h = opened

    print(f"[Camera] using index {index} — {cam_name or 'name unknown'}  ({w}x{h})")
    if not cam_name:
        print("         Name unknown, so this was chosen by position. If it is "
              "the wrong camera, use --device N or --name.")
    return cap, CameraInfo(device=f"index {index}",
                           card=cam_name or f"index {index}")


if __name__ == "__main__":
    names, source = device_names()
    if names:
        print(f"AVFoundation devices (via {source}):")
        for i, n in enumerate(names):
            print(f"  [{i}] {n}{'   [built-in]' if is_builtin(n) else ''}")
        print()
    scan()
