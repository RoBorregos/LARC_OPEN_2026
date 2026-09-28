#!/usr/bin/env python3
"""
record_dataset.py — record video and take photos for model training

Purpose : Capture CLEAN training data from the robot's cameras — the exact
          frames the vision scripts see, with no overlays, no masks, no
          rotation — from a browser UI. Photos and videos, one camera or all
          of them at once, each tagged with a label and the camera settings
          that were live when it was taken.

Usage (Orin)
    sudo systemctl stop larc-dispatcher         it owns the cameras
    cd ~/LARC_OPEN_2026/vision_orin
    python3 tools/record_dataset.py             then open http://localhost:8090
    python3 tools/record_dataset.py --roles separator,benefits
    sudo systemctl start larc-dispatcher        when you are done

Usage (pc/bench)
    python3 tools/record_dataset.py --list                 what cameras exist
    python3 tools/record_dataset.py --roles separator      resolved by name
    python3 tools/record_dataset.py --device separator=1   or force an index

Keys in the page (not while typing in a box)
    Space  photo from every selected camera
    R      start / stop recording on every selected camera
    A      auto-photo on / off (every N seconds)

Output — one folder per session, one per camera, one per label
    dataset/<session>/<role>/<label>/
        photo_<role>_<label>_<HHMMSS_mmm>.png
        video_<role>_<label>_<HHMMSS>[_partNN].avi
        video_...json       camera, settings, frame count, drops, timing
        video_...csv        per-frame timestamp — true timing, for syncing
    dataset/<session>/manifest.jsonl  one line per photo / video, append-only

Camera opening is the same as everything else in vision_orin:
    Orin  cameras.json role -> camera_select.open_role (VID:PID / serial),
          with that role's fourcc / size / fps
    Mac   debug_camera.open_for_debug, which resolves the role by NAME
    any   --device role=N or role=/dev/videoX overrides both
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(REPO / "link"))

from camera_select import CameraNotFound, load_config, open_role  # noqa: E402

SYS_V4L = "/sys/class/video4linux"
DEFAULT_OUT = REPO / "dataset"
DEFAULT_ROLES = ("intake", "separator", "benefits")

PREVIEW_MAX_W = 640
PREVIEW_JPEG_Q = 70
PREVIEW_FPS = 15
PHOTO_JPEG_Q = 95
VIDEO_QUALITY = 95          # MJPG only
QUEUE_FRAMES = 120          # ~4 s at 30 fps before frames are dropped
MIN_FREE_BYTES = 500 * 1024 * 1024
SETTINGS_TTL_S = 10.0

CAP_PROPS = {
    "auto_exposure": "CAP_PROP_AUTO_EXPOSURE", "exposure": "CAP_PROP_EXPOSURE",
    "gain": "CAP_PROP_GAIN", "auto_wb": "CAP_PROP_AUTO_WB",
    "wb_temperature": "CAP_PROP_WB_TEMPERATURE",
    "brightness": "CAP_PROP_BRIGHTNESS", "contrast": "CAP_PROP_CONTRAST",
    "saturation": "CAP_PROP_SATURATION", "sharpness": "CAP_PROP_SHARPNESS",
    "autofocus": "CAP_PROP_AUTOFOCUS", "focus": "CAP_PROP_FOCUS",
    "fps_reported": "CAP_PROP_FPS",
}


def now_stamp(ms: bool = False) -> str:
    t = dt.datetime.now()
    return t.strftime("%H%M%S") + (f"_{t.microsecond // 1000:03d}" if ms else "")


def clean_label(text: str) -> str:
    text = re.sub(r"[^a-z0-9_-]+", "_", (text or "").strip().lower()).strip("_")
    return text[:48] or "unlabeled"


def fourcc_str(value: float) -> str:
    code = int(value)
    chars = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
    return chars if chars.isprintable() and chars.strip() else ""


def dispatcher_running() -> bool:
    if platform.system() != "Linux" or not shutil.which("systemctl"):
        return False
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", "larc-dispatcher"],
                              capture_output=True, timeout=3).returncode == 0
    except Exception:
        return False


#  Opening
def open_any(role: str, device=None, config_path=None):
    """(cap, CameraInfo, settings). settings = this role's cameras.json entry."""
    try:
        settings = dict(load_config(config_path).get(role, {}))
    except CameraNotFound:
        settings = {}
    settings = {k: v for k, v in settings.items() if not k.startswith("_")}

    # The Orin, no override: exactly what production does, settings included.
    if device is None and os.path.isdir(SYS_V4L):
        cap, info = open_role(role, config_path)
        return cap, info, settings

    # Everywhere else, the debug path — pass the role's settings explicitly.
    from debug_camera import open_for_debug
    cap, info = open_for_debug(role, device=device,
                               width=settings.get("width"),
                               height=settings.get("height"),
                               fourcc=settings.get("fourcc"),
                               config_path=config_path)
    return cap, info, settings


#  One camera
class Camera:
    """Owns one capture. The capture thread is the only reader; every frame
    it reads goes to the recorder (if any) exactly once, so videos are
    frame-complete and never contain duplicated preview frames."""

    def __init__(self, role, cap, info, settings):
        self.role, self.cap, self.info, self.settings = role, cap, info, settings
        self.error = None
        self.lock = threading.Lock()
        self.frame = None
        self.frame_t = 0.0
        self.seq = 0
        self.fps = 0.0
        self.recorder: "Recorder | None" = None
        self.photos = 0
        self._stop = threading.Event()
        self._settings_cache = (0.0, {})
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.thread = threading.Thread(target=self._run, name=f"cam-{role}", daemon=True)
        self.thread.start()

    def _run(self):
        misses, count, t0 = 0, 0, time.monotonic()
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            stamp = time.time()
            if not ok or frame is None:
                misses += 1
                if misses > 300:
                    self.error = "stopped delivering frames (unplugged? bus starved?)"
                    rec = self.recorder
                    if rec:
                        rec.stop("camera lost")
                    return
                time.sleep(0.005)
                continue
            misses = 0
            with self.lock:
                self.frame, self.frame_t = frame, stamp
                self.seq += 1
                rec = self.recorder
            if rec is not None:
                rec.offer(frame, stamp)
            count += 1
            elapsed = time.monotonic() - t0
            if elapsed >= 1.0:
                self.fps, count, t0 = count / elapsed, 0, time.monotonic()

    def latest(self):
        with self.lock:
            return self.frame, self.frame_t, self.seq

    def camera_settings(self, fresh: bool = False) -> dict:
        """What the camera is ACTUALLY doing, read back — not what we asked."""
        cached_t, cached = self._settings_cache
        if not fresh and time.monotonic() - cached_t < SETTINGS_TTL_S:
            return cached
        out = {"fourcc": fourcc_str(self.cap.get(cv2.CAP_PROP_FOURCC)),
               "width": self.width, "height": self.height,
               "fps_measured": round(self.fps, 2)}
        for key, name in CAP_PROPS.items():
            prop = getattr(cv2, name, None)
            if prop is None:
                continue
            try:
                value = self.cap.get(prop)
            except Exception:
                continue
            if value not in (-1, None):
                out[key] = value
        dev = str(getattr(self.info, "device", ""))
        if dev.startswith("/dev/") and shutil.which("v4l2-ctl"):
            try:
                ctrls = subprocess.run(["v4l2-ctl", "-d", dev, "--get-ctrl",
                                        "auto_exposure,exposure_time_absolute,"
                                        "gain,white_balance_automatic,"
                                        "white_balance_temperature,focus_automatic_continuous,"
                                        "focus_absolute"],
                                       capture_output=True, text=True, timeout=2).stdout
                out["v4l2"] = dict(line.split(": ", 1) for line in ctrls.splitlines()
                                   if ": " in line)
            except Exception:
                pass
        self._settings_cache = (time.monotonic(), out)
        return out

    def identity(self) -> dict:
        info = self.info
        ident = {"role": self.role, "device": str(getattr(info, "device", ""))}
        describe = getattr(info, "describe", None)
        ident["camera"] = describe() if callable(describe) else str(info)
        stable = getattr(info, "stable_path", None)
        if callable(stable):
            ident["stable_path"] = stable()
        for key in ("vid", "pid", "serial", "usb_port"):
            value = getattr(info, key, "")
            if value:
                ident[key] = value
        return ident

    def release(self):
        self._stop.set()
        rec = self.recorder
        if rec is not None:
            rec.stop("shutdown")
            rec.join()
        self.thread.join(timeout=2)
        self.cap.release()


#  Recording
class Recorder:
    """Encodes on its own thread so a slow disk never stalls capture. If the
    queue fills anyway, frames are DROPPED and counted — never silently."""

    def __init__(self, cam: Camera, folder: Path, base: str, codec: str,
                 fps: float, segment_s: float, session: "Session", label: str):
        self.cam, self.folder, self.base = cam, folder, base
        self.codec, self.fps, self.segment_s = codec, fps, segment_s
        self.session, self.label = session, label
        self.q: queue.Queue = queue.Queue(maxsize=QUEUE_FRAMES)
        self.frames = 0
        self.dropped = 0
        self.part = 0
        self.started = time.time()
        self.reason = ""
        self.files: list = []
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"rec-{cam.role}", daemon=True)
        self.thread.start()

    def offer(self, frame, stamp):
        if self._stop.is_set():
            return
        try:
            self.q.put_nowait((frame, stamp))
        except queue.Full:
            self.dropped += 1

    def stop(self, reason: str = "stopped"):
        if not self._stop.is_set():
            self.reason = reason
            self._stop.set()

    def join(self):
        self.thread.join(timeout=15)

    @property
    def elapsed(self) -> float:
        return time.time() - self.started

    def _open_part(self, shape):
        h, w = shape[:2]
        suffix = f"_part{self.part:02d}" if self.segment_s > 0 else ""
        ext, tag = (".avi", "MJPG") if self.codec == "mjpg" else (".mp4", "mp4v")
        path = self.folder / f"{self.base}{suffix}{ext}"
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*tag),
                                 self.fps, (w, h))
        if self.codec == "mjpg" and hasattr(cv2, "VIDEOWRITER_PROP_QUALITY"):
            writer.set(cv2.VIDEOWRITER_PROP_QUALITY, VIDEO_QUALITY)
        if not writer.isOpened():
            raise RuntimeError(f"VideoWriter would not open {path}")
        csv = open(path.with_suffix(".csv"), "w")
        csv.write("frame,unix_time,t_rel\n")
        return path, writer, csv

    def _close_part(self, path, writer, csv, part_frames, part_t0, part_t1, drops0):
        writer.release()
        csv.close()
        span = max(part_t1 - part_t0, 1e-6)
        meta = {
            "type": "video", "file": path.name, "label": self.label,
            **self.cam.identity(),
            "frames": part_frames, "dropped": self.dropped - drops0,
            "start": dt.datetime.fromtimestamp(part_t0).isoformat(timespec="milliseconds"),
            "duration_s": round(part_t1 - part_t0, 3),
            "fps_container": self.fps,
            "fps_actual": round((part_frames - 1) / span, 2) if part_frames > 1 else 0,
            "codec": self.codec, "part": self.part if self.segment_s > 0 else None,
            "stop_reason": self.reason or "segment",
            "camera_settings": self.cam.camera_settings(),
            "host": socket.gethostname(),
        }
        path.with_suffix(".json").write_text(json.dumps(meta, indent=2))
        self.session.log(meta, self.folder / path.name)
        self.files.append(path.name)

    def _run(self):
        path = writer = csv = None
        part_frames, part_t0, t_last, drops0 = 0, 0.0, 0.0, 0
        try:
            while True:
                try:
                    frame, stamp = self.q.get(timeout=0.2)
                except queue.Empty:
                    if self._stop.is_set():
                        break
                    continue
                if writer is not None and self.segment_s > 0 and \
                        stamp - part_t0 >= self.segment_s:
                    self._close_part(path, writer, csv, part_frames, part_t0, t_last, drops0)
                    writer = None
                    self.part += 1
                if writer is None:
                    path, writer, csv = self._open_part(frame.shape)
                    part_frames, part_t0, drops0 = 0, stamp, self.dropped
                writer.write(frame)
                csv.write(f"{part_frames},{stamp:.6f},{stamp - part_t0:.6f}\n")
                part_frames += 1
                self.frames += 1
                t_last = stamp
                if self.frames % 60 == 0 and self.session.free_bytes() < MIN_FREE_BYTES:
                    self.stop("disk almost full")
        except Exception as exc:                        # noqa: BLE001
            self.reason = f"error: {exc}"
            print(f"[rec] {self.cam.role}: {exc}")
        finally:
            if writer is not None:
                self._close_part(path, writer, csv, part_frames, part_t0, t_last, drops0)
            with self.cam.lock:
                if self.cam.recorder is self:
                    self.cam.recorder = None
            print(f"[rec] {self.cam.role}: {self.frames} frames, {self.dropped} dropped "
                  f"— {self.reason or 'stopped'}")


#  Session: where things go, and the running tally
class Session:

    def __init__(self, root: Path, name: str, photo_fmt: str, codec: str, segment_s: float):
        self.root = root
        self.dir = root / name
        self.name = name
        self.photo_fmt = photo_fmt
        self.codec = codec
        self.segment_s = segment_s
        self.dir.mkdir(parents=True, exist_ok=True)
        self.manifest = self.dir / "manifest.jsonl"
        self._lock = threading.Lock()
        self.recent: list = []
        self.counts: dict = {}           # (role, label) -> {"photos": n, "videos": n}
        self._scan_existing()

    def _scan_existing(self):
        if not self.manifest.exists():
            return
        for line in self.manifest.read_text().splitlines():
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            self._count(entry)

    def _count(self, entry):
        key = (entry.get("role", "?"), entry.get("label", "?"))
        bucket = self.counts.setdefault(key, {"photos": 0, "videos": 0, "frames": 0})
        if entry.get("type") == "photo":
            bucket["photos"] += 1
        else:
            bucket["videos"] += 1
            bucket["frames"] += int(entry.get("frames", 0))

    def folder(self, role: str, label: str) -> Path:
        path = self.dir / role / label
        path.mkdir(parents=True, exist_ok=True)
        return path

    def log(self, entry: dict, path: Path):
        entry = dict(entry, path=str(path.relative_to(self.dir)))
        with self._lock:
            with open(self.manifest, "a") as handle:
                handle.write(json.dumps(entry) + "\n")
            self._count(entry)
            self.recent.insert(0, {"type": entry["type"], "role": entry.get("role"),
                                   "label": entry.get("label"), "path": entry["path"],
                                   "time": time.strftime("%H:%M:%S"),
                                   "frames": entry.get("frames")})
            del self.recent[30:]

    def free_bytes(self) -> int:
        try:
            return shutil.disk_usage(self.dir).free
        except OSError:
            return 0

    def tally(self) -> list:
        with self._lock:
            return [{"role": r, "label": l, **v}
                    for (r, l), v in sorted(self.counts.items())]


#  The controller the HTTP handler talks to
class Studio:

    def __init__(self, cams: dict, session: Session):
        self.cams = cams
        self.session = session
        self.label = "unlabeled"
        self.selected = set(cams)
        self.auto_every = 0.0
        self.auto_on = False
        self.flash: dict = {}              # role -> time of last photo
        self.message = ""
        self.warn_dispatcher = dispatcher_running()
        self._lock = threading.Lock()
        self._auto = threading.Thread(target=self._auto_loop, name="autosnap", daemon=True)
        self._auto.start()

    def _targets(self, role):
        if role and role != "all":
            return [self.cams[role]] if role in self.cams else []
        return [self.cams[r] for r in sorted(self.selected) if r in self.cams]

    # photos
    def snap(self, role=None) -> list:
        saved = []
        if self.session.free_bytes() < MIN_FREE_BYTES:
            self.message = "disk almost full — not saving"
            return saved
        label = self.label
        for cam in self._targets(role):
            frame, stamp, _ = cam.latest()
            if frame is None or cam.error:
                continue
            folder = self.session.folder(cam.role, label)
            ext = "png" if self.session.photo_fmt == "png" else "jpg"
            name = f"photo_{cam.role}_{label}_{now_stamp(ms=True)}.{ext}"
            path = folder / name
            params = [int(cv2.IMWRITE_JPEG_QUALITY), PHOTO_JPEG_Q] if ext == "jpg" else \
                     [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
            if not cv2.imwrite(str(path), frame, params):
                self.message = f"could not write {path}"
                continue
            cam.photos += 1
            self.flash[cam.role] = time.time()
            self.session.log({
                "type": "photo", "file": name, "label": label, **cam.identity(),
                "time": dt.datetime.fromtimestamp(stamp).isoformat(timespec="milliseconds"),
                "width": frame.shape[1], "height": frame.shape[0],
                "camera_settings": cam.camera_settings(),
            }, path)
            saved.append(str(path.relative_to(self.session.root)))
        if saved:
            self.message = f"saved {len(saved)} photo(s)"
        return saved

    def _auto_loop(self):
        next_t = 0.0
        while True:
            time.sleep(0.05)
            if not (self.auto_on and self.auto_every > 0):
                next_t = 0.0
                continue
            now = time.monotonic()
            if now >= next_t:
                self.snap()
                next_t = now + self.auto_every

    # video
    def recording(self, cam) -> bool:
        return cam.recorder is not None

    def start(self, role=None) -> list:
        started = []
        if self.session.free_bytes() < MIN_FREE_BYTES:
            self.message = "disk almost full — not recording"
            return started
        label = self.label
        for cam in self._targets(role):
            if cam.recorder is not None or cam.error:
                continue
            fps = cam.fps if cam.fps > 1 else float(cam.settings.get("fps") or 30)
            fps = round(fps)
            folder = self.session.folder(cam.role, label)
            base = f"video_{cam.role}_{label}_{now_stamp()}"
            cam.camera_settings(fresh=True)
            rec = Recorder(cam, folder, base, self.session.codec, fps,
                           self.session.segment_s, self.session, label)
            with cam.lock:
                cam.recorder = rec
            started.append(cam.role)
            print(f"[rec] {cam.role}: -> {folder / base}  @ {fps} fps")
        if started:
            self.message = "recording " + ", ".join(started)
        return started

    def stop(self, role=None) -> list:
        stopped = []
        for cam in self._targets(role) if role not in (None, "all") else self.cams.values():
            rec = cam.recorder
            if rec is not None:
                rec.stop("stopped by user")
                stopped.append(cam.role)
        if stopped:
            self.message = "stopped " + ", ".join(stopped)
        return stopped

    def toggle(self, role=None):
        targets = self._targets(role)
        if any(c.recorder is not None for c in targets):
            for cam in targets:
                if cam.recorder is not None:
                    cam.recorder.stop("stopped by user")
            self.message = "stopped"
        else:
            self.start(role)

    def state(self) -> dict:
        cams = []
        for role, cam in self.cams.items():
            rec = cam.recorder
            cams.append({
                "role": role, "device": cam.identity().get("device"),
                "camera": cam.identity().get("camera"),
                "width": cam.width, "height": cam.height,
                "fps": round(cam.fps, 1), "error": cam.error,
                "selected": role in self.selected, "photos": cam.photos,
                "flash": time.time() - self.flash.get(role, 0) < 0.35,
                "rec": None if rec is None else {
                    "elapsed": round(rec.elapsed, 1), "frames": rec.frames,
                    "dropped": rec.dropped, "queue": rec.q.qsize(),
                    "part": rec.part, "fps": rec.fps},
            })
        return {
            "cams": cams, "label": self.label,
            "session": self.session.name, "out": str(self.session.dir),
            "free_gb": round(self.session.free_bytes() / 1e9, 1),
            "auto_on": self.auto_on, "auto_every": self.auto_every,
            "photo_fmt": self.session.photo_fmt, "codec": self.session.codec,
            "tally": self.session.tally(), "recent": self.session.recent[:12],
            "message": self.message, "dispatcher": self.warn_dispatcher,
        }


STUDIO: "Studio | None" = None


def preview_jpeg(frame, recording: bool):
    """Preview only — the REC border is drawn on a copy and never saved."""
    h, w = frame.shape[:2]
    if w > PREVIEW_MAX_W:
        frame = cv2.resize(frame, (PREVIEW_MAX_W, int(h * PREVIEW_MAX_W / w)),
                           interpolation=cv2.INTER_AREA)
    elif recording:
        frame = frame.copy()
    if recording:
        cv2.rectangle(frame, (1, 1), (frame.shape[1] - 2, frame.shape[0] - 2),
                      (40, 40, 230), 4)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), PREVIEW_JPEG_Q])
    return buf.tobytes() if ok else None


#  HTTP
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    def _send(self, code, body: bytes, ctype="text/plain; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload, code=200):
        self._send(code, json.dumps(payload).encode(), "application/json")

    def do_GET(self):
        route = urlparse(self.path)
        if route.path == "/":
            return self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        if route.path == "/state":
            return self._json(STUDIO.state())
        if route.path.startswith("/stream/"):
            return self._stream(route.path.rsplit("/", 1)[-1])
        return self._send(404, b"not found")

    def do_POST(self):
        route = urlparse(self.path)
        query = parse_qs(route.query)

        def one(name, default=None):
            return query.get(name, [default])[0]

        studio, path, role = STUDIO, route.path, one("role")
        if path == "/label":
            studio.label = clean_label(one("value", ""))
            return self._json({"ok": True, "label": studio.label})
        if path == "/select":
            if role in studio.cams:
                if one("on") == "1":
                    studio.selected.add(role)
                else:
                    studio.selected.discard(role)
            return self._json({"ok": True})
        if path == "/snap":
            return self._json({"ok": True, "saved": studio.snap(role)})
        if path == "/record":
            action = one("action", "toggle")
            if action == "start":
                studio.start(role)
            elif action == "stop":
                studio.stop(role)
            else:
                studio.toggle(role)
            return self._json({"ok": True})
        if path == "/auto":
            try:
                studio.auto_every = max(0.0, float(one("every", studio.auto_every)))
            except ValueError:
                pass
            if one("on") is not None:
                studio.auto_on = one("on") == "1"
            return self._json({"ok": True})
        return self._send(404, b"not found")

    def _stream(self, role):
        cam = STUDIO.cams.get(role)
        if cam is None:
            return self._send(404, b"no such camera")
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seen, period = -1, 1.0 / PREVIEW_FPS
        try:
            while True:
                frame, _, seq = cam.latest()
                if frame is None or seq == seen:
                    time.sleep(0.02)
                    continue
                seen = seq
                jpeg = preview_jpeg(frame, cam.recorder is not None)
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpeg)).encode() +
                                     b"\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dataset recorder</title>
<style>
:root{--bg:#111418;--panel:#1a1f25;--line:#2b323b;--text:#e6e9ee;--dim:#8b95a3;
--accent:#3d8bfd;--rec:#e5484d;--ok:#46a758;--warn:#f5a524}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,-apple-system,Segoe UI,sans-serif}
header{display:flex;flex-wrap:wrap;gap:12px 20px;align-items:center;padding:12px 16px;
border-bottom:1px solid var(--line);position:sticky;top:0;background:var(--bg);z-index:2}
h1{font-size:16px;margin:0;font-weight:600}
.meta{color:var(--dim);font-size:12px}
.banner{background:#3a2a0a;color:var(--warn);padding:8px 16px;border-bottom:1px solid #5a4214}
main{padding:16px;min-width:0;display:grid;gap:16px;grid-template-columns:minmax(0,1fr) 300px}
@media(max-width:900px){main{grid-template-columns:minmax(0,1fr)}}
section{min-width:0}
.bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;background:var(--panel);
border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:16px}
.bar label{color:var(--dim);font-size:12px}
input[type=text],input[type=number]{background:#0d1014;color:var(--text);border:1px solid var(--line);
border-radius:6px;padding:7px 9px;font:inherit}
input[type=text]{width:190px;max-width:100%}input[type=number]{width:64px}
button{background:#232a33;color:var(--text);border:1px solid var(--line);border-radius:6px;
padding:7px 12px;font:inherit;cursor:pointer}
button:hover{border-color:#44505e}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
button.rec{background:var(--rec);border-color:var(--rec);color:#fff}
button.on{background:var(--ok);border-color:var(--ok);color:#fff}
kbd{font-size:11px;color:inherit;opacity:.75;border:1px solid currentColor;border-radius:4px;padding:0 4px;margin-left:4px}
.chips{display:flex;gap:6px;flex-wrap:wrap}
.chip{font-size:12px;padding:3px 9px;border-radius:99px;background:#0d1014;border:1px solid var(--line);cursor:pointer}
.chip.cur{border-color:var(--accent);color:var(--accent)}
.grid{display:grid;gap:16px;grid-template-columns:repeat(auto-fill,minmax(min(340px,100%),1fr))}
.cam{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden}
.cam.recording{border-color:var(--rec)}
.cam .view{position:relative;background:#000;display:flex;align-items:center;justify-content:center}
.cam img{max-width:100%;max-height:100%;display:block}
.cam .flash{position:absolute;inset:0;background:#fff;opacity:0;pointer-events:none;transition:opacity .25s}
.cam .flash.go{opacity:.55;transition:none}
.cam .badge{position:absolute;top:8px;left:8px;background:var(--rec);color:#fff;font-weight:600;
font-size:12px;padding:2px 8px;border-radius:4px;display:none}
.cam.recording .badge{display:block}
.cam .body{padding:10px 12px;display:grid;gap:8px}
.cam .top{display:flex;justify-content:space-between;align-items:center;gap:8px}
.cam .name{font-weight:600;text-transform:capitalize}
.cam .info{color:var(--dim);font-size:12px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.cam .info.stats{white-space:normal}
.cam .err{color:var(--rec);font-size:12px}
.cam .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
aside .box{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:16px}
aside h2{font-size:13px;margin:0 0 8px;color:var(--dim);font-weight:600;text-transform:uppercase;letter-spacing:.04em}
table{width:100%;border-collapse:collapse;font-size:12px}
td,th{padding:4px 2px;text-align:left;border-bottom:1px solid var(--line)}
th{color:var(--dim);font-weight:500}td.n{text-align:right;font-variant-numeric:tabular-nums}
ul{list-style:none;margin:0;padding:0;font-size:12px}
li{padding:4px 0;border-bottom:1px solid var(--line);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
li .t{color:var(--dim);margin-right:6px}
#msg{color:var(--dim);font-size:12px;min-height:1em}
</style></head><body>
<header>
  <h1>Dataset recorder</h1>
  <span class="meta" id="where"></span>
  <span class="meta" id="free"></span>
</header>
<div class="banner" id="disp" hidden>larc-dispatcher is running and probably owns the cameras.
Stop it first: <code>sudo systemctl stop larc-dispatcher</code></div>
<main>
<section>
  <div class="bar">
    <label for="label">Label</label>
    <input type="text" id="label" placeholder="e.g. mature_red, empty, venue_light">
    <button id="setlabel">Set</button>
    <div class="chips" id="chips"></div>
  </div>
  <div class="bar">
    <button class="primary" id="snapall">Photo, all selected<kbd>Space</kbd></button>
    <button id="recall">Record, all selected<kbd>R</kbd></button>
    <span style="flex:1"></span>
    <label for="every">Auto-photo every</label>
    <input type="number" id="every" min="0.2" step="0.1" value="1"> <span class="meta">s</span>
    <button id="auto">Auto off<kbd>A</kbd></button>
  </div>
  <div id="msg"></div>
  <div class="grid" id="cams"></div>
</section>
<aside>
  <div class="box"><h2>This session</h2><table id="tally"></table></div>
  <div class="box"><h2>Recent</h2><ul id="recent"></ul></div>
</aside>
</main>
<script>
const $ = s => document.querySelector(s);
const post = (p) => fetch(p, {method: 'POST'}).then(r => r.json()).catch(() => ({}));
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
let S = null, built = false, labels = [];
try { labels = JSON.parse(localStorage.getItem('rd_labels') || '[]'); } catch (e) {}

function remember(l) {
  labels = [l, ...labels.filter(x => x !== l)].slice(0, 10);
  try { localStorage.setItem('rd_labels', JSON.stringify(labels)); } catch (e) {}
}
async function setLabel(v) {
  const r = await post('/label?value=' + encodeURIComponent(v));
  if (r.label) { remember(r.label); $('#label').value = r.label; }
  refresh();
}
function fmt(t) { const m = Math.floor(t / 60), s = Math.floor(t % 60); return m + ':' + String(s).padStart(2, '0'); }

function build(cams) {
  $('#cams').innerHTML = cams.map(c => `
  <div class="cam" id="cam-${c.role}">
    <div class="view" style="aspect-ratio:${c.width}/${c.height}"><img src="/stream/${c.role}" alt="${c.role} preview">
      <div class="flash"></div><div class="badge">REC</div></div>
    <div class="body">
      <div class="top"><span class="name">${esc(c.role)}</span>
        <label class="meta"><input type="checkbox" data-sel="${c.role}"> selected</label></div>
      <div class="info" title="${esc(c.camera)}">${esc(c.device)} · ${esc(c.camera)}</div>
      <div class="info stats"></div>
      <div class="err"></div>
      <div class="row">
        <button data-snap="${c.role}">Photo</button>
        <button data-rec="${c.role}">Record</button>
      </div>
    </div>
  </div>`).join('');
  document.querySelectorAll('[data-snap]').forEach(b => b.onclick = () => post('/snap?role=' + b.dataset.snap).then(refresh));
  document.querySelectorAll('[data-rec]').forEach(b => b.onclick = () => post('/record?role=' + b.dataset.rec).then(refresh));
  document.querySelectorAll('[data-sel]').forEach(b => b.onchange = () => post(`/select?role=${b.dataset.sel}&on=${b.checked ? 1 : 0}`));
  built = true;
}

function render(s) {
  S = s;
  if (!built) { build(s.cams); $('#label').value = s.label; $('#every').value = s.auto_every || 1; }
  $('#where').textContent = `session ${s.session} → ${s.out}`;
  $('#free').textContent = `${s.free_gb} GB free · photos ${s.photo_fmt} · video ${s.codec}`;
  $('#disp').hidden = !s.dispatcher;
  $('#msg').textContent = s.message || '';
  const all = [s.label, ...labels.filter(l => l !== s.label)];
  $('#chips').innerHTML = all.map(l => `<span class="chip ${l === s.label ? 'cur' : ''}" data-l="${esc(l)}">${esc(l)}</span>`).join('');
  document.querySelectorAll('.chip').forEach(c => c.onclick = () => setLabel(c.dataset.l));
  let anyRec = false;
  for (const c of s.cams) {
    const el = $('#cam-' + c.role); if (!el) continue;
    el.classList.toggle('recording', !!c.rec);
    if (c.rec) anyRec = anyRec || c.selected;
    const sel = el.querySelector('[data-sel]'); if (document.activeElement !== sel) sel.checked = c.selected;
    const stats = `${c.width}×${c.height} · ${c.fps} fps · ${c.photos} photos` +
      (c.rec ? ` · REC ${fmt(c.rec.elapsed)} · ${c.rec.frames} frames` +
        (c.rec.dropped ? ` · <b style="color:var(--rec)">${c.rec.dropped} dropped</b>` : '') +
        (c.rec.part ? ` · part ${c.rec.part}` : '') : '');
    el.querySelector('.stats').innerHTML = stats;
    el.querySelector('.err').textContent = c.error || '';
    const rb = el.querySelector('[data-rec]');
    rb.textContent = c.rec ? 'Stop' : 'Record'; rb.className = c.rec ? 'rec' : '';
    if (c.flash) { const f = el.querySelector('.flash'); f.classList.add('go'); setTimeout(() => f.classList.remove('go'), 60); }
  }
  const ra = $('#recall'); ra.className = anyRec ? 'rec' : '';
  ra.firstChild.textContent = anyRec ? 'Stop recording' : 'Record, all selected';
  const au = $('#auto'); au.className = s.auto_on ? 'on' : '';
  au.firstChild.textContent = s.auto_on ? 'Auto on' : 'Auto off';
  $('#tally').innerHTML = '<tr><th>camera</th><th>label</th><th class="n">photos</th><th class="n">videos</th><th class="n">frames</th></tr>' +
    (s.tally.length ? s.tally.map(t => `<tr><td>${esc(t.role)}</td><td>${esc(t.label)}</td><td class="n">${t.photos}</td><td class="n">${t.videos}</td><td class="n">${t.frames}</td></tr>`).join('')
      : '<tr><td colspan="5" class="meta">nothing yet</td></tr>');
  $('#recent').innerHTML = s.recent.map(r => `<li title="${esc(r.path)}"><span class="t">${r.time}</span>${r.type === 'photo' ? '📷' : '🎞'} ${esc(r.path)}</li>`).join('') || '<li class="meta">nothing yet</li>';
}

async function refresh() {
  try { render(await (await fetch('/state')).json()); } catch (e) { $('#msg').textContent = 'lost connection to the recorder'; }
}

$('#setlabel').onclick = () => setLabel($('#label').value);
$('#label').onkeydown = e => { if (e.key === 'Enter') { setLabel(e.target.value); e.target.blur(); } };
$('#snapall').onclick = () => post('/snap?role=all').then(refresh);
$('#recall').onclick = () => post('/record?role=all').then(refresh);
$('#auto').onclick = () => post(`/auto?every=${$('#every').value}&on=${S && S.auto_on ? 0 : 1}`).then(refresh);
$('#every').onchange = () => post('/auto?every=' + $('#every').value);
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'checkbox') return;
  if (e.repeat || e.metaKey || e.ctrlKey || e.altKey) return;
  if (document.activeElement && document.activeElement.tagName === 'BUTTON') document.activeElement.blur();
  if (e.code === 'Space') { e.preventDefault(); $('#snapall').click(); }
  else if (e.key === 'r' || e.key === 'R') $('#recall').click();
  else if (e.key === 'a' || e.key === 'A') $('#auto').click();
});
refresh(); setInterval(refresh, 400);
</script></body></html>
"""


#  Main
def parse_devices(items) -> dict:
    out = {}
    for item in items or []:
        if "=" not in item:
            raise SystemExit(f"--device wants role=N or role=/dev/videoX, got '{item}'")
        role, dev = item.split("=", 1)
        out[role.strip()] = dev.strip()
    return out


def main() -> int:
    global STUDIO

    parser = argparse.ArgumentParser(
        description="Record clean video and photos from the robot cameras for "
                    "model training, from a browser.")
    parser.add_argument("--roles", help="comma list; default: every role in "
                        "cameras.json that opens (intake, separator, benefits)")
    parser.add_argument("--device", action="append", metavar="ROLE=DEV",
                        help="open a role by index or path instead of by "
                             "identity, e.g. separator=1. Repeatable. The role "
                             "need not be in cameras.json.")
    parser.add_argument("--out", default=str(DEFAULT_OUT),
                        help=f"dataset root (default {DEFAULT_OUT})")
    parser.add_argument("--session", default=dt.date.today().isoformat(),
                        help="session folder name (default: today's date)")
    parser.add_argument("--label", default="unlabeled", help="starting label")
    parser.add_argument("--photo-format", choices=("png", "jpg"), default="png",
                        help="png = lossless (default), jpg = smaller")
    parser.add_argument("--codec", choices=("mjpg", "mp4v"), default="mjpg",
                        help="mjpg .avi = near-lossless per frame, easy to "
                             "extract (default); mp4v .mp4 = much smaller")
    parser.add_argument("--segment-min", type=float, default=5.0,
                        help="split videos every N minutes so a crash loses "
                             "one part, not the take (0 = never)")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--bind", default="127.0.0.1",
                        help="0.0.0.0 to reach it from other machines")
    parser.add_argument("--list", action="store_true", help="list cameras and exit")
    args = parser.parse_args()

    if args.list:
        if os.path.isdir(SYS_V4L):
            from camera_select import list_cameras
            for cam in list_cameras():
                print(f"{cam.device:<14} {cam.describe()}\n{'':<14} {cam.stable_path()}")
        else:
            import debug_camera
            names, source = debug_camera.device_names()
            for i, n in enumerate(names):
                print(f"  [{i}] {n}   (via {source})")
            debug_camera.scan()
        return 0

    devices = parse_devices(args.device)
    if args.roles:
        roles = [r.strip() for r in args.roles.split(",") if r.strip()]
    else:
        try:
            roles = [r for r in load_config() if r in DEFAULT_ROLES] or list(DEFAULT_ROLES)
        except CameraNotFound:
            roles = list(DEFAULT_ROLES)
        roles += [r for r in devices if r not in roles]

    if dispatcher_running():
        print("[warn] larc-dispatcher is running and owns the cameras. Stop it first:\n"
              "       sudo systemctl stop larc-dispatcher")

    cams = {}
    for role in roles:
        try:
            cap, info, settings = open_any(role, devices.get(role))
        except CameraNotFound as exc:
            print(f"[cam] {role:<10} UNAVAILABLE — {exc}")
            continue
        except Exception as exc:                        # noqa: BLE001
            print(f"[cam] {role:<10} UNAVAILABLE — {type(exc).__name__}: {exc}")
            continue
        cam = Camera(role, cap, info, settings)
        cams[role] = cam
        print(f"[cam] {role:<10} {cam.identity()['device']}  {cam.width}x{cam.height}  "
              f"({cam.identity()['camera']})")

    if not cams:
        print("[fatal] no camera opened. Try --list, then --device role=N.")
        return 1

    # Two roles that landed on the same device would record one camera twice.
    seen = {}
    for role, cam in cams.items():
        dev = cam.identity()["device"]
        if dev in seen:
            print(f"[warn] '{role}' and '{seen[dev]}' opened the SAME device {dev}. "
                  f"Pass --device to separate them, or --roles to drop one.")
        seen[dev] = role

    session = Session(Path(args.out).expanduser().resolve(), args.session,
                      args.photo_format, args.codec, args.segment_min * 60)
    STUDIO = Studio(cams, session)
    STUDIO.label = clean_label(args.label)

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    print(f"\n  open  http://localhost:{args.port}   (VS Code forwards this over SSH)")
    if args.bind != "127.0.0.1":
        print(f"  also  http://{socket.gethostname()}:{args.port}")
    print(f"  saving to {session.dir}")
    print("  Ctrl+C to stop — open recordings are closed cleanly\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[stopping]")
    finally:
        STUDIO.auto_on = False
        for cam in cams.values():
            cam.release()
        server.server_close()
        if platform.system() == "Linux" and shutil.which("systemctl"):
            print("[reminder] sudo systemctl start larc-dispatcher")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
