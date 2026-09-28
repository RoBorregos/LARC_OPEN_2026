#!/usr/bin/env python3
"""
dispatcher.py — the Orin's one long running process (protocol v2)

Purpose : Own the serial link to the Teensy, start and stop the vision
          scripts for the phase the Teensy asks for, turn what they print
          into protocol-v2 command frames, and keep that stream alive so the
          Teensy's watchdog stays happy. No servo logic lives here.
Link    : link/teensy_link.py (the only serial owner)
Runs on : the Orin, normally as a systemd service
Logs    : stdout -> journald.  journalctl -u larc-dispatcher -f
Stop    : Ctrl+C or systemctl stop — both park the robot in IDLE first

Script contract
    intake     VISION:XX        bit 0 = intake upper, bit 1 = intake lower
    separator  VISION:FD:WW:CC  WW = warm hit, CC = cool hit (00 or 01)
    benefits   VISION:FE:XX     00 none, 01 red, 02 blue (only when centred)
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

# vision_orin/link is a sibling of this directory.
REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "link"))
sys.path.insert(0, str(REPO))
from model_runtime.shared import SharedModelWorker

import vision_protocol as vp
from teensy_link import TeensyLink, find_teensy

SCRIPTS = {
    "intake":    Path(os.environ.get("LARC_INTAKE_PY",
                                     REPO / "main_vision" / "orin_vision_model.py")),
    "separator": Path(os.environ.get("LARC_SEPARATOR_PY",
                                     REPO / "separator" / "separator_model.py")),
    "benefits":  Path(os.environ.get("LARC_BENEFITS_PY",
                                     REPO / "benefits" / "benefits.py")),
}

PHASES = {
    "beans":    ["intake", "separator"],
    "benefits": ["benefits"],
}

SERIAL_DEVICE = os.environ.get("LARC_TEENSY_DEV") or None

AUTOSTART_PHASE = os.environ.get("LARC_AUTOSTART_PHASE", "").strip().lower() or None

# Tuning
LOOP_HZ = 100 # how often we recompute the command
# Only affects the wait for the first report; normal stale limits stay unchanged.
STARTUP_GRACE_SEC = float(os.environ.get("LARC_STARTUP_GRACE_SEC", "120"))
if not 0 < STARTUP_GRACE_SEC <= 180:
    raise ValueError("LARC_STARTUP_GRACE_SEC must be greater than 0 and at most 180")
HEALTH_CHECK_SEC = 2.0
LINK_RETRY_SEC = 2.0

# A source that has not printed for this long stops counting.
STALE_MS = {"intake": 500, "separator": 500, "benefits": 800}

# How long the separator holds a side after the last hit for that side.
SEPARATOR_HOLD_MS = 250

# Semantic mapping — the only place these meanings are written down
INTAKE_UPPER_BIT = 0x01
INTAKE_LOWER_BIT = 0x02

WARM_IS = vp.SEP_LEFT # warm ball - separator LEFT  (mature)
COOL_IS = vp.SEP_RIGHT # cool ball - separator RIGHT (overmature)

BOX_NONE, BOX_RED, BOX_BLUE = 0, 1, 2
BOX_DOOR = {BOX_RED: 0, BOX_BLUE: 1}   # box type - which door opens
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}

FAULT_BIT = {
    "intake":    vp.STATUS_MAIN_FAULT, # critical on the Teensy
    "separator": vp.STATUS_SEPARATOR_FAULT,
    "benefits":  vp.STATUS_BENEFITS_FAULT,
}

VISION_LINE = re.compile(r"^VISION:([0-9A-Fa-f:]+)\s*$")
MAX_KEPT_LINES = 40


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def now_ms() -> int:
    return int(time.monotonic() * 1000)


# What the vision scripts are currently telling us
class VisionState:

    def __init__(self) -> None:
        self._lock = threading.Lock()

        self.intake_upper = False
        self.intake_lower = False
        self.intake_ms = 0

        self.sep_side = vp.SEP_NEUTRAL
        self.sep_until_ms = 0
        self.separator_ms = 0

        self.box = BOX_NONE
        self.box_ms = 0

        self.reports = {} # actual reports only; startup grace is not readiness
        self.stale = set() # sources that have gone quiet

    def feed(self, source: str, payload: str) -> bool:
        try:
            fields = [int(part, 16) for part in payload.split(":")]
        except ValueError:
            return False
        if not fields:
            return False

        stamp = now_ms()
        with self._lock:
            if source == "intake" and len(fields) == 1:
                self.intake_upper = bool(fields[0] & INTAKE_UPPER_BIT)
                self.intake_lower = bool(fields[0] & INTAKE_LOWER_BIT)
                self.intake_ms = stamp
                self.reports[source] = stamp
                return True

            if source == "separator" and len(fields) == 3 and fields[0] == 0xFD:
                warm, cool = bool(fields[1]), bool(fields[2])
                if warm != cool:
                    self.sep_side = WARM_IS if warm else COOL_IS
                    self.sep_until_ms = stamp + SEPARATOR_HOLD_MS
                self.separator_ms = stamp
                self.reports[source] = stamp
                return True

            if source == "benefits" and len(fields) == 2 and fields[0] == 0xFE:
                self.box = fields[1]
                self.box_ms = stamp
                self.reports[source] = stamp
                return True

        return False

    def invalidate(self, source):
        with self._lock:
            had_report = source in self.reports
            self.reports.pop(source, None)
            # Before the first valid report, keep the launch deadline intact.
            # Errors neither grant readiness nor extend that deadline. Once a
            # source has reported, invalidate it immediately with no new grace.
            if source == 'intake':
                self.intake_upper = self.intake_lower = False
                if had_report:
                    self.intake_ms = 0
            elif source == 'separator':
                self.sep_side = vp.SEP_NEUTRAL
                self.sep_until_ms = 0
                if had_report:
                    self.separator_ms = 0

    def note_launched(self, sources) -> None:
        stamp = now_ms()
        with self._lock:
            for source in sources:
                offset = int(STARTUP_GRACE_SEC * 1000)
                if source == "intake":
                    self.intake_ms = stamp + offset
                elif source == "separator":
                    self.separator_ms = stamp + offset
                elif source == "benefits":
                    self.box_ms = stamp + offset
            self.stale -= set(sources)

    def reset(self) -> None:
        with self._lock:
            self.intake_upper = self.intake_lower = False
            self.sep_side = vp.SEP_NEUTRAL
            self.sep_until_ms = 0
            self.box = BOX_NONE
            self.stale.clear()
            self.reports.clear()

    def ready(self, sources):
        stamp = now_ms()
        with self._lock:
            return all(source in self.reports and
                       stamp - self.reports[source] <= STALE_MS[source]
                       for source in sources)

    # called from the main loop
    def beans_command(self):
        stamp = now_ms()
        with self._lock:
            stale = set()

            if stamp - self.intake_ms > STALE_MS["intake"]:
                stale.add("intake")
                upper = lower = False
            else:
                upper, lower = self.intake_upper, self.intake_lower

            if stamp - self.separator_ms > STALE_MS["separator"]:
                stale.add("separator")
                separator = vp.SEP_NEUTRAL
            else:
                separator = (self.sep_side if stamp < self.sep_until_ms
                             else vp.SEP_NEUTRAL)

            self.stale = stale
            return upper, lower, separator, stale

    def benefits_command(self):
        stamp = now_ms()
        with self._lock:
            stale = set()
            if stamp - self.box_ms > STALE_MS["benefits"]:
                stale.add("benefits")
                self.stale = stale
                return False, False, stale

            # Held while benefits.py sees the box centred. The Teensy decides
            # when to open (openBenefit()); timed doors re-arm on their own.
            door = BOX_DOOR.get(self.box)
            self.stale = stale
            return door == 0, door == 1, stale


# Child processes
class ScriptRunner:

    def __init__(self, state: VisionState) -> None:
        self.state = state
        self.procs = {}
        self.tails = {}
        self.phase = None
        self._lock = threading.Lock()
        self.model_roles = [name for name in ('intake', 'separator')
                            if SCRIPTS[name].name in ('orin_vision_model.py', 'separator_model.py')]
        self.model = (SharedModelWorker(self.model_roles, state.feed, state.invalidate, log)
                      if self.model_roles else None)
        self._model_death_reported = False

    def prepare(self):
        if self.model:
            self.model.start()

    def close(self):
        self.stop(quiet=True)
        if self.model:
            self.model.close()

    def _reader(self, proc, name: str) -> None:
        for raw in proc.stdout:
            text = raw.rstrip()
            if not text:
                continue
            match = VISION_LINE.match(text)
            if match:
                if not self.state.feed(name, match.group(1)):
                    log(f"[{name}] unparsable vision line: {text}")
                continue
            with self._lock:
                tail = self.tails.setdefault(name, [])
                tail.append(text)
                del tail[:-MAX_KEPT_LINES]
            log(f"[{name}] {text}")

    def tail(self, name: str, count: int = 8):
        with self._lock:
            return list(self.tails.get(name, []))[-count:]

    def start(self, phase: str) -> bool:
        self.stop(quiet=True)

        names = PHASES.get(phase)
        if not names:
            log(f"[ERROR] unknown phase '{phase}'")
            return False

        missing = [n for n in names if n not in self.model_roles and not SCRIPTS[n].exists()]
        if missing:
            for name in missing:
                log(f"[ERROR] script missing: {name} -> {SCRIPTS[name]}")
            return False

        self.state.note_launched(names)
        log(f"launching phase '{phase}': {names} (first-report grace {STARTUP_GRACE_SEC:g}s)")
        for name in names:
            if name in self.model_roles:
                continue
            path = SCRIPTS[name]
            try:
                proc = subprocess.Popen(
                    [sys.executable, "-u", str(path)],
                    cwd=str(path.parent), # so it finds its *_config.json
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except Exception as exc:
                log(f"[ERROR] could not launch {name}: {exc}")
                self.stop(quiet=True)
                return False

            self.procs[name] = proc
            self.tails[name] = []
            threading.Thread(target=self._reader, args=(proc, name),
                             name=f"read-{name}", daemon=True).start()
            log(f"  {name} pid={proc.pid}  ({path})")

        self.phase = phase
        if phase == 'beans' and self.model:
            self.prepare()
            if not self.model.alive():
                log(f'[ERROR] shared model worker unavailable: {self.model.error or "worker exited without an error message"}')
                self.stop(quiet=True)
                return False
            self.model.activate()
        return True

    def stop(self, quiet: bool = False) -> None:
        if self.model:
            self.model.deactivate()
        for name, proc in list(self.procs.items()):
            if proc and proc.poll() is None:
                if not quiet:
                    log(f"  stopping {name} pid={proc.pid}")
                try:
                    proc.terminate()
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        self.procs.clear()
        self.phase = None
        self.state.reset()

    def alive(self):
        names = [n for n, p in self.procs.items() if p and p.poll() is None]
        if self.phase == 'beans' and self.model and self.model.alive():
            names.extend(self.model_roles)
        return names

    def reap(self):
        dead = []
        if (self.phase == 'beans' and self.model and not self.model.alive()
                and not self._model_death_reported):
            self._model_death_reported = True
            dead.extend(self.model_roles)
            for role in self.model_roles:
                self.state.invalidate(role)
            log(f'[HEALTH] shared model worker exited: {self.model.error or "no error message"}')
        for name, proc in list(self.procs.items()):
            if proc and proc.poll() is not None:
                log(f"[HEALTH] {name} exited (rc={proc.returncode})")
                for line in self.tail(name, 10):
                    log(f"    | {line}")
                dead.append(name)
                del self.procs[name]
        return dead


# Main
class Dispatcher:
    def __init__(self, args) -> None:
        self.args = args
        self.state = VisionState()
        self.runner = ScriptRunner(self.state)
        self.link = None
        self.running = True
        self._last_reported = None
        self._stale_reported = set()
        self._warned_missing = False
        self._ready_reported = None
        self._last_wait_log = 0.0

    def stop(self, *_signal) -> None:
        self.running = False

    # -- link
    def connect(self) -> None:
        while self.running and self.link is None:
            try:
                self.link = TeensyLink(device=SERIAL_DEVICE)
                self._warned_missing = False
                log(f"serial open: {self.link.device} @ {vp.BAUD}")
                self.link.set_ready(True)
                log("dispatcher ready — waiting for the Teensy to ask for a phase")
            except Exception as exc:
                if not self._warned_missing:
                    log(f"serial not available ({exc})")
                    log(f"waiting for a Teensy — retrying every {LINK_RETRY_SEC}s, "
                        f"quietly from here")
                    self._warned_missing = True
                time.sleep(LINK_RETRY_SEC)

    def drop_link(self, reason: str) -> None:
        log(f"[serial] lost: {reason}")
        self.runner.stop(quiet=True)
        try:
            if self.link:
                self.link.close()
        except Exception:
            pass
        self.link = None

    def handle_messages(self) -> None:
        for line in self.link.take_messages():
            log(f"[teensy] {line}")

    # -- requests from the Teensy
    def handle_requests(self) -> None:
        for request in self.link.take_requests():
            name = vp.CMD_NAMES.get(request, f"0x{request:02X}")
            log(f">> Teensy requests {name}")
            if request == vp.CMD_START_BEANS:
                self.enter("beans")
            elif request == vp.CMD_START_BENEFITS:
                self.enter("benefits")
            elif request == vp.CMD_STOP:
                self.enter(None)
            elif request == vp.CMD_STATUS:
                self.report_status()

    def enter(self, phase) -> None:
        if phase == self.runner.phase:
            log(f"already in phase '{phase}' — nothing to do")
            return

        # Park the link before the scripts change under it.
        self.link.set_idle()
        self.link.set_beans_running(False)
        self.link.set_benefits_running(False)

        self._stale_reported.clear()
        self._last_reported = None
        self._ready_reported = None
        self._last_wait_log = 0.0
        for mask in FAULT_BIT.values():
            self.link.set_fault(mask, False)

        if phase is None:
            self.runner.stop()
            log("phase -> IDLE")
            return

        if not self.runner.start(phase):
            self.link.set_fault(FAULT_BIT.get(phase, vp.STATUS_MAIN_FAULT), True)
            return

        if phase == "beans":
            self.link.set_phase_beans()
            # RUNNING is asserted only after fresh reports from both workers.
        else:
            self.link.set_phase_benefits()
            # RUNNING is asserted only after the first benefits report.
        log(f"phase -> {phase.upper()}")

    # pushing the command
    def push_command(self) -> None:
        phase = self.runner.phase
        if phase == "beans":
            upper, lower, separator, stale = self.state.beans_command()
            self.link.set_beans(upper, lower, separator)
            summary = (f"BEANS upper={int(upper)} lower={int(lower)} "
                       f"sep={vp.SEP_NAMES[separator]}")
        elif phase == "benefits":
            door1, door2, stale = self.state.benefits_command()
            self.link.set_benefits(door1, door2)
            summary = f"BENEFITS door1={int(door1)} door2={int(door2)}"
        else:
            return

        ready = (self.state.ready(PHASES[phase]) and
                 all(name in self.runner.alive() for name in PHASES[phase]))
        self.link.set_beans_running(phase == "beans" and ready)
        self.link.set_benefits_running(phase == "benefits" and ready)
        if ready != self._ready_reported:
            log(f"[VISION READY] {phase.upper()}={int(ready)} (fresh reports required)")
            self._ready_reported = ready
        if not ready and time.monotonic() - self._last_wait_log >= 2:
            waiting = [n for n in PHASES[phase] if not self.state.ready([n])]
            log(f"[WAITING] {phase}: no fresh report from {waiting}")
            self._last_wait_log = time.monotonic()

        for source in stale - self._stale_reported:
            log(f"[STALE] {source} stopped reporting — its outputs are now safe")
            self.link.set_fault(FAULT_BIT[source], True)
        for source in self._stale_reported - stale:
            log(f"[STALE] {source} is reporting again")
            self.link.set_fault(FAULT_BIT[source], False)
        self._stale_reported = set(stale)

        if summary != self._last_reported:
            log(summary)
            self._last_reported = summary

    # health
    def check_health(self) -> None:
        if not self.runner.phase:
            return
        dead = self.runner.reap()
        if not dead:
            return
        for name in dead:
            self.link.set_fault(FAULT_BIT[name], True)
        if not self.runner.alive():
            log("[HEALTH] every script for this phase is dead — going IDLE")
            self.enter(None)

    def report_status(self) -> None:
        log("== STATUS ==")
        log(f"  phase        {self.runner.phase or 'IDLE'}")
        log(f"  serial       {self.link.device}")
        log(f"  status byte  0x{self.link.status:02X}")
        if self.runner.model:
            log(f"  model error  {self.runner.model.error or 'none'}")
        if self.runner.phase:
            for name in PHASES[self.runner.phase]:
                proc = self.runner.procs.get(name)
                alive = name in self.runner.alive()
                log(f"  {name:<10}  {'RUNNING' if alive else 'DEAD'}"
                    f"  worker={'shared-model' if name in self.runner.model_roles else (proc.pid if proc else '-')}")
                for line in self.runner.tail(name, 3):
                    log(f"      | {line}")
        else:
            import glob
            log(f"  video nodes  {sorted(glob.glob('/dev/video*'))}")
        log("== END STATUS ==")

    # -- run --
    def run(self) -> int:
        log("LARC vision dispatcher (protocol v2) starting")
        log(f"  repo         {REPO}")
        for name, path in SCRIPTS.items():
            if name in self.runner.model_roles:
                log(f"  {name:<10}  shared persistent model (no camera subprocess)")
            else:
                log(f"  {name:<10}  {path}  [{'OK' if path.exists() else 'MISSING'}]")
        log(f"  teensy       {SERIAL_DEVICE or find_teensy() or 'not found yet'}")

        signal.signal(signal.SIGTERM, self.stop)
        signal.signal(signal.SIGINT, self.stop)

        period = 1.0 / LOOP_HZ
        last_health = time.monotonic()

        try:
            self.runner.prepare()
            while self.running:
                if self.link is None:
                    self.connect()
                    if self.link is None:
                        break
                    phase = self.args.phase or AUTOSTART_PHASE
                    if phase:
                        source = "--phase" if self.args.phase else "LARC_AUTOSTART_PHASE"
                        log(f"{source}={phase}: starting it now without waiting "
                            f"for the Teensy to ask")
                        self.enter(phase)
                if not self.link.healthy:
                    self.drop_link(str(self.link.error or "transmit thread stopped"))
                    time.sleep(LINK_RETRY_SEC)
                    continue

                try:
                    self.handle_messages()
                    self.handle_requests()
                    self.push_command()

                    now = time.monotonic()
                    if now - last_health >= HEALTH_CHECK_SEC:
                        self.check_health()
                        last_health = now
                except OSError as exc:
                    self.drop_link(str(exc))
                    time.sleep(LINK_RETRY_SEC)
                    continue

                time.sleep(period)

        finally:
            log("shutting down — parking the robot")
            self.runner.stop(quiet=True)
            if self.link:
                try:
                    self.link.set_idle()
                    self.link.close() # sends IDLE frames, then closes
                except Exception:
                    pass
            self.runner.close()
            log("dispatcher exited")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="LARC Orin vision dispatcher (protocol v2).")
    parser.add_argument("--phase", choices=sorted(PHASES),
                        help="start this phase immediately instead of waiting "
                             "for the Teensy to ask (bench testing)")
    return Dispatcher(parser.parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
