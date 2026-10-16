#!/usr/bin/env python3
"""
camera_select.py — open a camera by what it IS, not by /dev/video<N>

Purpose : Turn a role name ("intake", "separator", "benefits") into an open
          cv2.VideoCapture by matching the camera's manufacturer, product
          name, serial number or physical USB port. /dev/video numbers are
          handed out in enumeration order and change between boots; none of
          these do.
Config  : ../cameras.json — roles and match rules
Runs on : the Orin (Linux/V4L2). Import it from anywhere.

Match rules, all optional, ANDed together
    device        exact path, e.g. "/dev/video_zed". Skips all matching.
    name_contains substring of the V4L2 card name, case-insensitive
    manufacturer_contains   substring of the USB manufacturer string
    vid, pid      USB IDs as 4-hex-digit strings ("2b03", "f582")
    serial        exact USB serial number — the strongest match there is
    usb_port      physical port, e.g. "1-2.3". Survives reboots, not
                  re-plugging into a different socket.
    by_id_contains  substring of a /dev/v4l/by-id/... symlink
    nth           tie-break when a rule matches several, 0-based after
                  sorting by usb_port. Prefer serial or usb_port.
"""

from __future__ import annotations

import glob
import json
import os
import re
from dataclasses import dataclass, field
from typing import Optional

SYS_V4L = "/sys/class/video4linux"

VIDIOC_QUERYCAP = 0x80685600
CAP_STRUCT_SIZE = 104
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_DEVICE_CAPS = 0x80000000

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cameras.json")


class CameraNotFound(RuntimeError):
    """No camera matched, or too many did. Both are fatal by design."""


@dataclass
class CameraInfo:
    device: str
    card: str = ""
    manufacturer: str = ""
    product: str = ""
    vid: str = ""
    pid: str = ""
    serial: str = ""
    usb_port: str = ""
    by_id: list = field(default_factory=list)
    by_path: list = field(default_factory=list)

    def describe(self) -> str:
        bits = [self.card or self.product or "?"]
        if self.vid:
            bits.append(f"{self.vid}:{self.pid}")
        if self.serial:
            bits.append(f"serial={self.serial}")
        if self.usb_port:
            bits.append(f"port={self.usb_port}")
        return "  ".join(bits)

    def stable_path(self) -> str:
        """The path worth putting in a log or a config: a by-id symlink if
        there is one, otherwise by-path, otherwise the raw node."""
        return (self.by_id or self.by_path or [self.device])[0]


# sysfs
def _read(path: str) -> str:
    try:
        with open(path, "r", errors="replace") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def _usb_dir(node: str) -> str:
    current = os.path.realpath(os.path.join(SYS_V4L, node, "device"))
    for _ in range(8):
        if os.path.exists(os.path.join(current, "idVendor")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return ""


def _links(device: str, directory: str) -> list:
    if not os.path.isdir(directory):
        return []
    return [os.path.join(directory, name)
            for name in sorted(os.listdir(directory))
            if os.path.realpath(os.path.join(directory, name)) == device]


def _is_capture(device: str) -> bool:
    """True only for nodes that can actually hand over an image. Cameras
    routinely create metadata nodes that open fine and never give a frame —
    this is what keeps us from picking one of those."""
    try:
        import ctypes
        import fcntl
    except ImportError:
        return True

    buffer = ctypes.create_string_buffer(CAP_STRUCT_SIZE)
    try:
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, buffer)
    except OSError:
        return False
    finally:
        os.close(fd)

    raw = buffer.raw
    caps = int.from_bytes(raw[84:88], "little")
    device_caps = int.from_bytes(raw[88:92], "little")
    effective = device_caps if (caps & V4L2_CAP_DEVICE_CAPS) else caps
    return bool(effective & V4L2_CAP_VIDEO_CAPTURE)


def list_cameras(capture_only: bool = True) -> list:
    """Every camera on this machine, with the fields you can match on."""
    if not os.path.isdir(SYS_V4L):
        return []

    cameras = []
    nodes = sorted(os.listdir(SYS_V4L),
                   key=lambda n: int(re.sub(r"\D", "", n) or 0))
    for node in nodes:
        device = f"/dev/{node}"
        if capture_only and not _is_capture(device):
            continue
        usb = _usb_dir(node)
        cameras.append(CameraInfo(
            device=device,
            card=_read(os.path.join(SYS_V4L, node, "name")),
            manufacturer=_read(os.path.join(usb, "manufacturer")) if usb else "",
            product=_read(os.path.join(usb, "product")) if usb else "",
            vid=_read(os.path.join(usb, "idVendor")) if usb else "",
            pid=_read(os.path.join(usb, "idProduct")) if usb else "",
            serial=_read(os.path.join(usb, "serial")) if usb else "",
            usb_port=os.path.basename(usb) if usb else "",
            by_id=_links(device, "/dev/v4l/by-id"),
            by_path=_links(device, "/dev/v4l/by-path"),
        ))
    return cameras


# matching
def _matches(cam: CameraInfo, rule: dict) -> bool:
    def contains(needle: str, haystack: str) -> bool:
        return needle.lower() in (haystack or "").lower()

    if "name_contains" in rule and not contains(rule["name_contains"], cam.card):
        # A camera's product string is often more descriptive than its card
        # name, so accept a hit on either.
        if not contains(rule["name_contains"], cam.product):
            return False
    if "manufacturer_contains" in rule and \
            not contains(rule["manufacturer_contains"], cam.manufacturer):
        return False
    if "vid" in rule and cam.vid.lower() != str(rule["vid"]).lower():
        return False
    if "pid" in rule and cam.pid.lower() != str(rule["pid"]).lower():
        return False
    if "serial" in rule and cam.serial != rule["serial"]:
        return False
    if "usb_port" in rule and cam.usb_port != rule["usb_port"]:
        return False
    if "by_id_contains" in rule and \
            not any(contains(rule["by_id_contains"], link) for link in cam.by_id):
        return False
    return True


def find_camera(rule: dict, cameras: Optional[list] = None) -> CameraInfo:
    """Resolve one match rule to exactly one camera, or raise."""
    if "device" in rule:
        path = rule["device"]
        if not os.path.exists(path):
            raise CameraNotFound(f"{path} does not exist")
        real = os.path.realpath(path)
        for cam in (cameras if cameras is not None else list_cameras()):
            if cam.device == real:
                return cam
        return CameraInfo(device=path, card="(explicit path)")

    pool = cameras if cameras is not None else list_cameras()
    matched = [cam for cam in pool if _matches(cam, rule)]

    if not matched:
        seen = "\n".join(f"      {c.device}  {c.describe()}" for c in pool) or \
               "      (no cameras at all)"
        raise CameraNotFound(
            f"no camera matched {rule}.\n    Cameras present:\n{seen}\n"
            f"    Run tools/camera_probe.py and fix cameras.json.")

    if len(matched) > 1:
        if "nth" in rule:
            ordered = sorted(matched, key=lambda c: (c.usb_port, c.device))
            index = int(rule["nth"])
            if index >= len(ordered):
                raise CameraNotFound(
                    f"rule {rule} matched {len(ordered)} cameras, "
                    f"nth={index} is out of range")
            return ordered[index]
        listing = "\n".join(f"      {c.device}  {c.describe()}" for c in matched)
        raise CameraNotFound(
            f"rule {rule} is ambiguous — it matched {len(matched)} cameras:\n"
            f"{listing}\n    Narrow it with serial= or usb_port=, or add nth=.")

    return matched[0]


# config
def load_config(path: Optional[str] = None) -> dict:
    path = path or os.environ.get("LARC_CAMERAS_JSON") or DEFAULT_CONFIG
    if not os.path.exists(path):
        raise CameraNotFound(
            f"camera config not found: {path}\n"
            f"    Run tools/camera_probe.py and fill it in.")
    with open(path) as handle:
        config = json.load(handle)
    return {k: v for k, v in config.items() if not k.startswith("_")}


def find_role(role: str, config_path: Optional[str] = None) -> CameraInfo:
    config = load_config(config_path)
    if role not in config:
        raise CameraNotFound(
            f"role '{role}' is not in the camera config. "
            f"Known roles: {', '.join(sorted(config)) or '(none)'}")
    return find_camera(config[role])


# opening
def open_camera(cam: CameraInfo, width: Optional[int] = None,
                height: Optional[int] = None, fourcc: Optional[str] = None,
                fps: Optional[float] = None, warmup_frames: int = 15):
    """Open a resolved camera and prove it delivers a frame before handing
    it back. Returns the cv2.VideoCapture; raises if it never produces one."""
    import cv2

    cap = cv2.VideoCapture(cam.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise CameraNotFound(f"{cam.device} ({cam.describe()}) would not open")

    # Order matters on V4L2: pixel format, then size, then rate.
    if fourcc:
        # cv2.VideoWriter_fourcc on OpenCV <= 4.9, VideoWriter.fourcc after.
        pack = getattr(cv2, "VideoWriter_fourcc", None) or cv2.VideoWriter.fourcc
        cap.set(cv2.CAP_PROP_FOURCC, pack(*fourcc))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)

    # Never hand back a stale frame: on a sorter, a 3-frame queue is 100 ms
    # of lag between the bean being there and the servo hearing about it.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(max(1, warmup_frames)):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            return cap

    cap.release()
    raise CameraNotFound(
        f"{cam.device} ({cam.describe()}) opened but never delivered a frame")


def open_role(role: str, config_path: Optional[str] = None, **kwargs):
    """The one call the vision scripts make. Returns (cap, CameraInfo)."""
    cam = find_role(role, config_path)
    settings = dict(load_config(config_path).get(role, {}))
    for key in ("width", "height", "fourcc", "fps"):
        if key in settings and key not in kwargs:
            kwargs[key] = settings[key]
    return open_camera(cam, **kwargs), cam


def open_bench(role: Optional[str] = None, device=None,
               config_path: Optional[str] = None, **kwargs):
    """Cross-platform open for the debug tools.

    On the Orin, pass a role and it resolves through cameras.json exactly
    like the production scripts. On a Mac there is no /sys/class/video4linux
    to match against, so pass `device` — an index (0, 1, ...) or a path —
    and it opens directly with the right backend for the OS.

    Returns (cap, CameraInfo). The CameraInfo is a stub when opening by
    index, so the caller can always print `cam.device`.
    """
    import platform

    import cv2

    if device is None and role and os.path.isdir(SYS_V4L):
        return open_role(role, config_path, **kwargs)

    if device is None:
        raise CameraNotFound(
            f"no /sys/class/video4linux here (this looks like "
            f"{platform.system()}), so role '{role}' cannot be resolved.\n"
            f"    Pass --device 0 (or 1, 2, ...) to pick a camera by index.")

    # Explicit macOS camera indices still need the role's calibrated size.
    if role:
        settings = load_config(config_path).get(role, {})
        for key in ("width", "height", "fourcc", "fps"):
            if key in settings and key not in kwargs:
                kwargs[key] = settings[key]

    # Accept "0" as an index and "/dev/video0" or a name as a path.
    try:
        target = int(device)
        backend = {"Darwin": cv2.CAP_AVFOUNDATION,
                   "Windows": cv2.CAP_DSHOW}.get(platform.system(), cv2.CAP_V4L2)
        label = f"index {target}"
    except (TypeError, ValueError):
        target, backend, label = str(device), cv2.CAP_ANY, str(device)

    cap = cv2.VideoCapture(target, backend)
    if not cap.isOpened():
        cap.release()
        raise CameraNotFound(f"could not open camera at {label}")

    if kwargs.get("fourcc"):
        pack = getattr(cv2, "VideoWriter_fourcc", None) or cv2.VideoWriter.fourcc
        cap.set(cv2.CAP_PROP_FOURCC, pack(*kwargs["fourcc"]))
    if kwargs.get("width"):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, kwargs["width"])
    if kwargs.get("height"):
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, kwargs["height"])
    if kwargs.get("fps"):
        cap.set(cv2.CAP_PROP_FPS, kwargs["fps"])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(15):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            return cap, CameraInfo(device=str(device), card=f"camera at {label}")

    cap.release()
    raise CameraNotFound(f"camera at {label} opened but never delivered a frame")


if __name__ == "__main__":
    for camera in list_cameras():
        print(f"{camera.device:<16} {camera.describe()}")
        print(f"{'':<16} {camera.stable_path()}")
