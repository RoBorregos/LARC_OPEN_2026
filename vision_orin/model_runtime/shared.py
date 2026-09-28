"""One persistent detector, owned exclusively by one inference thread."""
import threading
import time
import traceback


class SharedModelWorker:
    def __init__(self, roles, publish, invalidate, log):
        self.roles = tuple(roles)
        self.publish = publish
        self.invalidate = invalidate
        self.log = log
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.closed = threading.Event()
        self.active = False
        self.generation = 0
        self.activated_at = 0.
        self.error = None
        self.thread = None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, name='shared-model', daemon=True)
            self.thread.start()

    def activate(self):
        with self.lock:
            self.generation += 1
            self.activated_at = time.monotonic()
            self.active = True
        self.wake.set()

    def deactivate(self):
        # Wait only for publication, never for an in-flight GPU call.
        with self.lock:
            self.active = False
            self.generation += 1
        self.wake.set()

    def alive(self):
        return self.thread is not None and self.thread.is_alive() and self.error is None

    def close(self):
        self.deactivate()
        self.closed.set()
        self.wake.set()
        if self.thread:
            self.thread.join(timeout=3)

    def deliver(self, generation, role, out, captured_at):
        with self.lock:
            if (not self.active or generation != self.generation or
                    captured_at < self.activated_at):
                return
            if out.get('error') or time.monotonic() - captured_at > self.max_age:
                self.invalidate(role)
            else:
                self.publish(role, out['message'].removeprefix('VISION:'))

    def _run(self):
        cameras = {}
        try:
            from model_runtime.core import Detector, load_roi, settings
            from model_runtime.capture import Camera
            started = time.monotonic()
            options = settings()
            self.max_age = options['max_frame_age_ms'] / 1000
            self.log('[model] STARTUP: loading one shared model for ' + ', '.join(self.roles))
            detector = Detector(options)
            if self.closed.is_set():
                return
            self.log(f'[model] model loaded in {time.monotonic()-started:.1f}s; warming camera shapes')
            detector.warmup(self.roles)
            self.log(f'[model] WARM: one model ready in {time.monotonic()-started:.1f}s; fresh camera reports still required')
            seen, configs, roi_checked, last_log, retry = {}, {}, {}, {}, {}
            camera_generation = None
            while not self.closed.is_set():
                self.wake.clear()
                with self.lock:
                    active, generation = self.active, self.generation
                if generation != camera_generation or not active:
                    for camera in cameras.values():
                        camera.close()
                    cameras.clear()
                    seen.clear()
                    configs.clear()
                    roi_checked.clear()
                    retry.clear()
                    camera_generation = generation
                if not active:
                    self.wake.wait(.1)
                    continue
                worked = False
                for role in self.roles:
                    with self.lock:
                        if not self.active or generation != self.generation:
                            break
                    try:
                        now = time.monotonic()
                        if role not in cameras:
                            if now < retry.get(role, 0):
                                continue
                            retry[role] = now + 1
                            cameras[role] = Camera(role)
                        frame, stamp, seq, error = cameras[role].latest()
                        if error or frame is None or now-stamp > self.max_age:
                            self.deliver(generation, role, {'error': error or 'No fresh frame'}, now)
                            continue
                        if seq == seen.get(role):
                            continue
                        seen[role] = seq
                        worked = True
                        if role not in configs or now-roi_checked.get(role, 0) >= .5:
                            configs[role] = load_roi(role)
                            roi_checked[role] = now
                        out, _ = detector.run(frame, role, configs[role], stamp)
                        self.deliver(generation, role, out, stamp)
                        if time.monotonic()-last_log.get(role, 0) >= 2:
                            self.log(f"[{role}] shared model infer={out['inference_ms']}ms frame_age={out['frame_age_ms']}ms {out.get('error') or 'OK'}")
                            last_log[role] = time.monotonic()
                    except Exception as exc:
                        self.deliver(generation, role, {'error': str(exc)}, time.monotonic())
                        if time.monotonic()-last_log.get(role, 0) >= 2:
                            self.log(f'[{role}] ERROR: {exc}')
                            last_log[role] = time.monotonic()
                if not worked:
                    self.wake.wait(.02)
        except Exception as exc:
            self.error = str(exc)
            self.log(f'[model] FATAL: {type(exc).__name__}: {exc}')
            for line in traceback.format_exc().splitlines():
                self.log(f'[model] {line}')
        finally:
            for camera in cameras.values():
                camera.close()
