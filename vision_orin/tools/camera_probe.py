#!/usr/bin/env python3
"""
camera_probe.py — everything the Orin knows about every camera

Purpose : Print the full identity of every /dev/video* node — driver name,
USB vendor/product, serial number, physical USB port, the stable
/dev/v4l/by-id and by-path symlinks, capabilities and supported
formats. Run this first and send back what it prints: whatever is
in here is what a camera can be matched on, so we never have to
trust /dev/video0 keeping its number across a reboot.

Usage
    python3 camera_probe.py                 table + detail for every node
    python3 camera_probe.py --open          also try to grab one frame each
    python3 camera_probe.py --json out.json machine-readable dump as well
    python3 camera_probe.py --capture-only  hide metadata/output-only nodes

"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys

SYS_V4L = "/sys/class/video4linux"

# VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability)  — 104-byte struct.
# Done by hand so this script needs no v4l2 python bindings.
VIDIOC_QUERYCAP = 0x80685600
CAP_STRUCT_SIZE = 104

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_OUTPUT = 0x00000002
V4L2_CAP_META_CAPTURE = 0x00800000
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000

CAP_FLAGS = [
    (V4L2_CAP_VIDEO_CAPTURE, "CAPTURE"),
    (V4L2_CAP_VIDEO_OUTPUT, "OUTPUT"),
    (V4L2_CAP_META_CAPTURE, "META"),
    (V4L2_CAP_STREAMING, "STREAMING"),
]


# sysfs helpers
def read_attr(path: str) -> str:
    """Read a one-line sysfs attribute, or '' if it is not there."""
    try:
        with open(path, "r", errors="replace") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def usb_device_dir(node: str) -> str:
    """Walk up from a video node's sysfs entry to the USB device that owns
    it — the directory holding idVendor / idProduct / serial."""
    start = os.path.realpath(os.path.join(SYS_V4L, node, "device"))
    current = start
    for _ in range(8):
        if os.path.exists(os.path.join(current, "idVendor")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return ""


def usb_interface_number(node: str) -> str:
    """Which USB interface of the device this node came from. Cameras that
    expose several nodes differ here, so it is worth showing."""
    iface = os.path.realpath(os.path.join(SYS_V4L, node, "device"))
    match = re.search(r":(\d+)\.(\d+)$", os.path.basename(iface))
    return match.group(2) if match else ""


def symlinks_for(device: str, directory: str) -> list:
    """Every /dev/v4l/by-{id,path} symlink pointing at this device node."""
    found = []
    if not os.path.isdir(directory):
        return found
    for name in sorted(os.listdir(directory)):
        link = os.path.join(directory, name)
        if os.path.islink(link) and os.path.realpath(link) == device:
            found.append(link)
    return found


def extra_symlinks_for(device: str) -> list:
    """Custom udev symlinks straight in /dev, e.g. /dev/video_zed."""
    found = []
    for link in sorted(glob.glob("/dev/video_*")) + sorted(glob.glob("/dev/cam_*")):
        if os.path.islink(link) and os.path.realpath(link) == device:
            found.append(link)
    return found


# V4L2 capability query, without any bindings
def query_capabilities(device: str) -> dict:
    try:
        import ctypes
        import fcntl
    except ImportError:
        return {}

    buffer = ctypes.create_string_buffer(CAP_STRUCT_SIZE)
    try:
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    except OSError as exc:
        return {"error": f"cannot open: {exc.strerror}"}

    try:
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, buffer)
    except OSError as exc:
        os.close(fd)
        return {"error": f"QUERYCAP failed: {exc.strerror}"}
    os.close(fd)

    raw = buffer.raw
    driver = raw[0:16].split(b"\x00")[0].decode(errors="replace")
    card = raw[16:48].split(b"\x00")[0].decode(errors="replace")
    bus_info = raw[48:80].split(b"\x00")[0].decode(errors="replace")
    caps = int.from_bytes(raw[84:88], "little")
    device_caps = int.from_bytes(raw[88:92], "little")

    effective = device_caps if (caps & V4L2_CAP_DEVICE_CAPS) else caps
    names = [name for bit, name in CAP_FLAGS if effective & bit]

    return {
        "driver": driver,
        "card": card,
        "bus_info": bus_info,
        "caps_hex": f"0x{effective:08X}",
        "caps": names,
        "is_capture": bool(effective & V4L2_CAP_VIDEO_CAPTURE),
    }


# optional extras
def list_formats(device: str) -> list:
    """Resolutions and frame rates, via v4l2-ctl when it is installed."""
    if not shutil.which("v4l2-ctl"):
        return []
    try:
        out = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats-ext"],
            capture_output=True, text=True, timeout=8,
        ).stdout
    except Exception:
        return []

    entries, fourcc = [], ""
    for line in out.splitlines():
        line = line.strip()
        fmt = re.match(r"\[\d+\]:\s+'(\w+)'\s+\((.+)\)", line)
        if fmt:
            fourcc = f"{fmt.group(1)} ({fmt.group(2)})"
            continue
        size = re.match(r"Size:\s+\S+\s+(\d+x\d+)", line)
        if size:
            entries.append({"format": fourcc, "size": size.group(1), "fps": []})
            continue
        rate = re.search(r"\(([\d.]+)\s*fps\)", line)
        if rate and entries:
            entries[-1]["fps"].append(float(rate.group(1)))
    return entries


def try_open(device: str) -> dict:
    """Prove the node really hands over a frame. Needs OpenCV."""
    try:
        import cv2
    except ImportError:
        return {"tested": False, "reason": "opencv not installed"}

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return {"tested": True, "ok": False, "reason": "VideoCapture would not open"}

    frame = None
    for _ in range(15):
        ok, candidate = cap.read()
        if ok and candidate is not None and candidate.size:
            frame = candidate
            break

    result = {"tested": True, "ok": frame is not None}
    if frame is not None:
        height, width = frame.shape[:2]
        result["frame"] = f"{width}x{height}"
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        result["reported_fps"] = round(fps, 1) if fps > 0 else None
    cap.release()
    return result


# the probe
def probe_node(node: str, do_open: bool) -> dict:
    device = f"/dev/{node}"
    usb_dir = usb_device_dir(node)

    info = {
        "node": node,
        "device": device,
        "v4l_name": read_attr(os.path.join(SYS_V4L, node, "name")),
        "v4l_index": read_attr(os.path.join(SYS_V4L, node, "index")),
        "usb_interface": usb_interface_number(node),
        "vid": read_attr(os.path.join(usb_dir, "idVendor")) if usb_dir else "",
        "pid": read_attr(os.path.join(usb_dir, "idProduct")) if usb_dir else "",
        "manufacturer": read_attr(os.path.join(usb_dir, "manufacturer")) if usb_dir else "",
        "product": read_attr(os.path.join(usb_dir, "product")) if usb_dir else "",
        "serial": read_attr(os.path.join(usb_dir, "serial")) if usb_dir else "",
        # Basename of the USB device dir, e.g. "1-2.3" — the physical port.
        "usb_port": os.path.basename(usb_dir) if usb_dir else "",
        "usb_speed_mbps": read_attr(os.path.join(usb_dir, "speed")) if usb_dir else "",
        "by_id": symlinks_for(device, "/dev/v4l/by-id"),
        "by_path": symlinks_for(device, "/dev/v4l/by-path"),
        "udev_symlinks": extra_symlinks_for(device),
    }
    info.update(query_capabilities(device))
    info["formats"] = list_formats(device)
    if do_open:
        info["open_test"] = try_open(device)
    return info


def print_report(cameras: list) -> None:
    print("=" * 78)
    print("SUMMARY — one line per /dev/video node")
    print("=" * 78)
    print(f"{'node':<9} {'kind':<10} {'name':<26} {'VID:PID':<10} {'port':<9} serial")
    print("-" * 78)
    for cam in cameras:
        kind = "CAPTURE" if cam.get("is_capture") else ",".join(cam.get("caps", [])) or "?"
        name = (cam.get("card") or cam.get("v4l_name") or "?")[:26]
        ids = f"{cam['vid']}:{cam['pid']}" if cam["vid"] else "-"
        print(f"{cam['node']:<9} {kind:<10} {name:<26} {ids:<10} "
              f"{cam['usb_port'] or '-':<9} {cam['serial'] or '(none)'}")

    for cam in cameras:
        print()
        print("=" * 78)
        print(f"{cam['device']}   {cam.get('card') or cam.get('v4l_name')}")
        print("=" * 78)
        if cam.get("error"):
            print(f"  !! {cam['error']}")
        print(f"  driver        {cam.get('driver', '?')}")
        print(f"  bus_info      {cam.get('bus_info', '?')}")
        print(f"  capabilities  {', '.join(cam.get('caps', [])) or '?'}  "
              f"{cam.get('caps_hex', '')}")
        print(f"  sysfs name    {cam['v4l_name']}   (index {cam['v4l_index']}, "
              f"usb interface {cam['usb_interface'] or '?'})")
        print(f"  manufacturer  {cam['manufacturer'] or '(not reported)'}")
        print(f"  product       {cam['product'] or '(not reported)'}")
        print(f"  VID:PID       {cam['vid']}:{cam['pid']}" if cam["vid"] else
              "  VID:PID       (not a USB device)")
        print(f"  serial        {cam['serial'] or '(none — cannot match on this)'}")
        print(f"  usb port      {cam['usb_port'] or '-'}   "
              f"({cam['usb_speed_mbps'] or '?'} Mbps)")

        for label, key in (("by-id", "by_id"), ("by-path", "by_path"),
                           ("udev link", "udev_symlinks")):
            for link in cam[key]:
                print(f"  {label:<13} {link}")
        if not cam["by_id"] and not cam["by_path"]:
            print("  by-id/by-path (none — is /dev/v4l present?)")

        if cam.get("formats"):
            print("  formats:")
            for entry in cam["formats"][:24]:
                rates = ", ".join(f"{f:g}" for f in entry["fps"][:6]) or "?"
                print(f"      {entry['format']:<22} {entry['size']:<12} {rates} fps")
            if len(cam["formats"]) > 24:
                print(f"      ... and {len(cam['formats']) - 24} more")
        elif shutil.which("v4l2-ctl") is None:
            print("  formats:      install v4l-utils to list them "
                  "(sudo apt install v4l-utils)")

        test = cam.get("open_test")
        if test:
            if not test.get("tested"):
                print(f"  open test     skipped ({test.get('reason')})")
            elif test.get("ok"):
                print(f"  open test     OK — {test.get('frame')} "
                      f"@ {test.get('reported_fps') or '?'} fps reported")
            else:
                print(f"  open test     FAILED — {test.get('reason')}")


def print_advice(cameras: list) -> None:
    capture = [c for c in cameras if c.get("is_capture")]
    print()
    print("=" * 78)
    print("HOW TO NAME THESE CAMERAS")
    print("=" * 78)

    if not capture:
        print("  No capture-capable nodes found. Is anything plugged in?")
        return

    by_model = {}
    for cam in capture:
        by_model.setdefault((cam["vid"], cam["pid"]), []).append(cam)

    for (vid, pid), group in by_model.items():
        label = group[0].get("product") or group[0].get("card") or "?"
        print(f"\n  {label}  ({vid}:{pid}) — {len(group)} capture node(s)")
        serials = [c["serial"] for c in group if c["serial"]]
        if len(group) == 1:
            print("    Only one. Match it on name or VID:PID and you are done.")
        elif len(set(serials)) == len(group) and all(serials):
            print("    Every unit has its OWN serial. Match on serial — this is")
            print("    the good case: cables can move and nothing breaks.")
            for cam in group:
                print(f"      {cam['device']}  serial={cam['serial']}")
        else:
            print("    Same model, no usable serials — they are indistinguishable")
            print("    by identity. Match on the physical USB port instead:")
            for cam in group:
                path = cam["by_path"][0] if cam["by_path"] else "(no by-path link)"
                print(f"      {cam['device']}  port={cam['usb_port']}  {path}")
            print("    Decide which socket is which role, label the cables, and")
            print("    never swap them.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Dump the full identity of every camera on this machine.")
    parser.add_argument("--open", action="store_true",
                        help="also try to grab one frame from each node (needs OpenCV)")
    parser.add_argument("--capture-only", action="store_true",
                        help="hide metadata and output-only nodes")
    parser.add_argument("--json", metavar="FILE",
                        help="also write the whole probe as JSON")
    args = parser.parse_args()

    if not os.path.isdir(SYS_V4L):
        print("No /sys/class/video4linux — this is not a Linux box with V4L2.",
              file=sys.stderr)
        return 1

    nodes = sorted(os.listdir(SYS_V4L),
                   key=lambda n: int(re.sub(r"\D", "", n) or 0))
    if not nodes:
        print("No video devices found at all.")
        return 1

    cameras = [probe_node(node, args.open) for node in nodes]
    if args.capture_only:
        cameras = [c for c in cameras if c.get("is_capture")]

    print_report(cameras)
    print_advice(cameras)

    if args.json:
        with open(args.json, "w") as handle:
            json.dump(cameras, handle, indent=2)
        print(f"\nJSON written to {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
