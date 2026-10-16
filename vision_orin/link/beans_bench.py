"""BEANS bench controller using the existing wire protocol and serial library.
Freshness is checked independently of vision, since TeensyLink streams the last
command continuously. The hold-test sketch requests START_BEANS after boot.
"""
import threading
import time
from collections import deque
import vision_protocol as vp

class BeansBench:
    def __init__(self, device=None, link=None, start_thread=True, clock=time.monotonic):
        if link is None:
            from teensy_link import TeensyLink
            link = TeensyLink(device=device)
        self.link = link
        self.device = link.device
        self.clock = clock
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.active = False
        self.error = None
        self.samples = {}
        self.messages = deque(maxlen=20)
        self.last_separator_stamp = None
        self.side = vp.SEP_NEUTRAL
        self.until = 0.
        self.command = (False, False, vp.SEP_NEUTRAL)
        self.link.set_ready(True)
        self.link.set_idle()
        self.thread = None
        if start_thread:
            self.thread = threading.Thread(target=self._run, daemon=True, name='beans-freshness')
            self.thread.start()

    def publish(self, role, result, captured_at):
        with self.lock:
            self.samples[role] = (dict(result), captured_at)

    def push(self, intake_hits, verdict, intake_stamp=None, separator_stamp=None):
        # Adapter for the existing OpenCV tuner. Reprocessed frames retain
        # their original timestamp and cannot extend a separator hold.
        if intake_hits is not None:
            self.publish('intake', intake_hits, intake_stamp or 0.)
        if verdict is not None:
            self.publish('separator', verdict, separator_stamp or 0.)
        state = self.snapshot()
        return state['upper'], state['lower'], state['separator']

    def _fresh(self, role, now):
        result, stamp = self.samples.get(role, ({}, 0.))
        if result.get('error') or not 0 <= now-stamp < .4:
            return {}, stamp
        return result, stamp

    def tick(self):
        now = self.clock()
        with self.lock:
            for request in self.link.take_requests():
                self.messages.append('Request: ' + vp.CMD_NAMES.get(request, hex(request)))
                if request == vp.CMD_START_BEANS:
                    self.active = True
                elif request in (vp.CMD_STOP, vp.CMD_START_BENEFITS):
                    self.active = False
                    self.samples.clear()
                    self.side, self.until = vp.SEP_NEUTRAL, 0.
            self.messages.extend(self.link.take_messages())
            if not self.active:
                self.command = (False, False, vp.SEP_NEUTRAL)
                self.link.set_beans_running(False)
                self.link.set_idle()
                return
            intake, _ = self._fresh('intake', now)
            separator, stamp = self._fresh('separator', now)
            if not separator:
                self.side, self.until = vp.SEP_NEUTRAL, 0.
            elif stamp != self.last_separator_stamp:
                self.last_separator_stamp = stamp
                warm, cool = bool(separator.get('warm')), bool(separator.get('cool'))
                if warm != cool:
                    self.side = vp.SEP_LEFT if warm else vp.SEP_RIGHT
                    self.until = now + .250
            side = self.side if now < self.until else vp.SEP_NEUTRAL
            self.command = (bool(intake.get('right')), bool(intake.get('left')), side)
            self.link.set_beans_running(True)
            self.link.set_beans(*self.command)

    def _run(self):
        try:
            while not self.stop.is_set():
                self.tick()
                self.stop.wait(.02)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.command = (False, False, vp.SEP_NEUTRAL)
            self.link.halt()

    def snapshot(self):
        with self.lock:
            upper, lower, side = self.command
            return dict(device=self.device, healthy=self.link.healthy and self.error is None,
                        active=self.active, upper=upper, lower=lower,
                        separator=vp.SEP_NAMES[side], error=self.error or str(self.link.error or ''),
                        messages=list(self.messages))

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1)
        self.link.set_beans_running(False)
        self.link.set_idle()
        self.link.close()
