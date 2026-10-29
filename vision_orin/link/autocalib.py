#!/usr/bin/env python3
"""
autocalib.py — shared autocalibration primitives (intake + separator)

Purpose : Track venue lighting without letting a calibrator wander. Venue
          light is not the light you tuned under and you cannot stop the
          robot to retune, but a calibrator that drifts is worse than one
          that never runs. So every tracked value is a BOUNDED OFFSET FROM A
          SEED, never a free parameter:
              seed:the hand-tuned value. The anchor. Never lost.
              live: what the current frame says it should be
              published  = clamp(EMA(live), seed - max_delta, seed + max_delta)
          A publish happens only once a ConfidenceGate agrees. If confidence
          drops the calibrator FREEZES at its last published values — it does
          not revert, and it does not keep updating on bad data.
Threads : the detection loop must never block. Calibrators run on their own
          thread and publish by rebinding one dict reference, atomic in
          CPython, so the hot path reads `cfg = calib.current` once and a
          frame always sees an internally consistent config, never a
          half-applied update.
"""

from __future__ import annotations

import threading
import time


# one tracked parameter
class Clamped:

    __slots__ = ("name", "seed", "max_delta", "alpha", "lo", "hi",
                 "ema", "int_result")

    def __init__(self, name, seed, max_delta, alpha=0.05,
                 lo=0, hi=255, int_result=True):
        self.name = name
        self.seed = float(seed)
        self.max_delta = float(max_delta)
        self.alpha = float(alpha)
        self.lo = lo
        self.hi = hi
        self.int_result = int_result
        self.ema = float(seed)

    def observe(self, value):
        self.ema += self.alpha * (float(value) - self.ema)
        return self.value()

    def value(self):
        v = self.ema
        v = max(self.seed - self.max_delta, min(self.seed + self.max_delta, v))
        v = max(self.lo, min(self.hi, v))
        return int(round(v)) if self.int_result else v

    def reset(self):
        self.ema = self.seed

    def drift(self):
        return self.value() - self.seed

    def at_limit(self):
        return abs(self.ema - self.seed) >= self.max_delta


# confidence
class ConfidenceGate:

    __slots__ = ("need", "tolerate", "_good", "_bad", "ok", "reason")

    def __init__(self, need=5, tolerate=2):
        self.need = need
        self.tolerate = tolerate
        self._good = 0
        self._bad = 0
        self.ok = False
        self.reason = "starting up"

    def update(self, good, reason=""):
        if good:
            self._bad = 0
            self._good += 1
            if self._good >= self.need:
                self.ok = True
                self.reason = "ok"
        else:
            self._good = 0
            self._bad += 1
            if self._bad >= self.tolerate:
                self.ok = False
                self.reason = reason or "low confidence"
        return self.ok


# base calibrator
class Calibrator:

    def __init__(self, seed_cfg, hz=8.0, need=5, tolerate=2, name="calib"):
        self.name = name
        self.seed_cfg = dict(seed_cfg)
        self._params = self._build_params(self.seed_cfg)
        self.gate = ConfidenceGate(need=need, tolerate=tolerate)

        # The published config. Rebound wholesale; never mutated in place.
        self.current = dict(seed_cfg)

        self._period = 1.0 / float(hz)
        self._frame = None
        self._frame_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self.ticks = 0
        self.commits = 0
        self.frozen_since = None
        self.last_reason = "starting up"

    # -- subclass hooks
    def _build_params(self, seed_cfg):
        raise NotImplementedError

    def _estimate(self, frame):
        raise NotImplementedError

    def _postprocess(self, cfg):
        return cfg

    # -- producer side: called from the detection loop, must be cheap
    def offer(self, frame):

        if self._frame_lock.acquire(blocking=False):
            try:
                self._frame = frame
            finally:
                self._frame_lock.release()

    def _take(self):
        with self._frame_lock:
            frame = self._frame
            self._frame = None
        return frame
    def tick(self, frame=None):
        if frame is None:
            frame = self._take()
        if frame is None:
            return False

        self.ticks += 1
        estimates, reason = self._estimate(frame)

        if estimates is None:
            self.gate.update(False, reason)
            self.last_reason = reason
            if not self.gate.ok and self.frozen_since is None:
                self.frozen_since = time.time()
            return False

        self.gate.update(True)
        self.last_reason = "ok"

        for key, value in estimates.items():
            param = self._params.get(key)
            if param is not None:
                param.observe(value)

        if not self.gate.ok:
            if self.frozen_since is None:
                self.frozen_since = time.time()
            return False

        self.frozen_since = None
        cfg = dict(self.current)
        for key, param in self._params.items():
            cfg[key] = param.value()
        cfg = self._postprocess(cfg)

        if cfg != self.current:
            self.current = cfg
            self.commits += 1
            return True
        return False

    # thread control
    def start(self):
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, name=self.name,
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self):
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception as exc: # never kill the run
                self.last_reason = f"error: {exc}"
                self.gate.update(False, self.last_reason)
            self._stop.wait(self._period)

    # telemetry
    def status(self):
        drifts = " ".join(
            f"{p.name}{p.drift():+.0f}{'!' if p.at_limit() else ''}"
            for p in self._params.values() if abs(p.drift()) >= 1)
        state = "OK    " if self.gate.ok else "FROZEN"
        held = ""
        if self.frozen_since is not None:
            held = f" for {time.time() - self.frozen_since:.0f}s"
        return (f"[{self.name}] {state}{held}  ticks={self.ticks} "
                f"commits={self.commits}  {self.last_reason}"
                + (f"  drift: {drifts}" if drifts else "  drift: none"))


# autocalibration inside a tuning tool
class DebugHarness:
    """Runs a calibrator inside a DEBUG/TUNING tool.

        observe  (default) the calibrator runs and reports what it WOULD
                 publish, but the pipeline still uses the original values. This is
                 how we check whether the calibrator agrees with our
                 hand tuning, and whether the seed is any good, without
                 it touching anything.
        apply    the calibrator drives the pipeline and you watch it adapt
                 under changing light. the trackbars become the seed it
                 works from.
        off      the calibrator does not run at all.
    """

    MODES = ("observe", "apply", "off")

    def __init__(self, factory, mode="observe", every=3):
        self.factory = factory
        self.mode = mode if mode in self.MODES else "observe"
        self.every = max(1, int(every))
        self.calib = None
        self.error = None
        self._seed = None
        self._n = 0

    def cycle(self):
        self.mode = self.MODES[(self.MODES.index(self.mode) + 1) % len(self.MODES)]
        if self.mode == "off":
            self.calib = None
            self._seed = None
        return self.mode

    def update(self, manual_cfg, frame):
        """Feed one frame. Returns the config the pipeline should use."""
        if self.mode == "off":
            return manual_cfg
        if self.calib is None or manual_cfg != self._seed:
            try:
                self.calib = self.factory(manual_cfg)
                self._seed = dict(manual_cfg)
                self.error = None
            except Exception as exc:
                self.error = str(exc)
                self.calib = None
                return manual_cfg

        self._n += 1
        if frame is not None and self._n % self.every == 0:
            try:
                self.calib.tick(frame)
            except Exception as exc:
                self.error = str(exc)

        if self.mode == "apply" and self.calib is not None:
            return self.calib.current
        return manual_cfg

    def deltas(self, manual_cfg):
        """{key: (yours, calibrated)} for every value that differs."""
        if self.calib is None:
            return {}
        out = {}
        for key in self.calib._params:
            mine = manual_cfg.get(key)
            theirs = self.calib.current.get(key)
            if mine is not None and theirs is not None and mine != theirs:
                out[key] = (mine, theirs)
        return out

    def lines(self, manual_cfg):
        """Short status for an overlay or a console line."""
        if self.mode == "off":
            return ["autocalib OFF  (c to cycle)"]
        if self.error:
            return [f"autocalib ERROR  {self.error}"]
        if self.calib is None:
            return [f"autocalib {self.mode.upper()}  starting"]

        head = (f"autocalib {self.mode.upper()}  "
                + ("locked on" if self.calib.gate.ok else
                   f"settling ({self.calib.last_reason})"))
        if self.mode == "observe":
            head += "  — showing what it WOULD do, using YOUR values"
        else:
            head += "  — DRIVING the pipeline"

        diffs = self.deltas(manual_cfg)
        if not diffs:
            return [head, "no difference from your values"]
        body = "  ".join(f"{k} {a}->{b}" for k, (a, b) in sorted(diffs.items()))
        return [head, body]
