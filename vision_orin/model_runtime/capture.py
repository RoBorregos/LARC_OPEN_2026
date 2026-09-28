"""Drain each camera continuously; inference reads only the latest frame."""
import contextlib
import os
from pathlib import Path
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'link'))

class Camera:
    def __init__(self, role, device=None):
        from camera_select import open_bench
        with contextlib.redirect_stdout(sys.stderr):
            self.cap, _ = open_bench(role, device=device, config_path=os.environ.get('LARC_CAMERAS_JSON'))
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.frame = None
        self.stamp = 0.
        self.sequence = 0
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    with self.lock:
                        self.error = 'Camera not delivering frames'
                    time.sleep(.02)
                    continue
                with self.lock:
                    self.frame, self.stamp = frame, time.monotonic()
                    self.sequence += 1
                    self.error = None
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            self.cap.release()

    def latest(self):
        with self.lock:
            return self.frame, self.stamp, self.sequence, self.error

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
