#!/usr/bin/env python3

"""
teensy_link.py — the Orin's end of the Teensy link

Purpose : Own the serial port to the Teensy and keep a command frame stream
          flowing on it. Vision code calls a setter when it decides
          something; this class handles sequence numbers, CRC, the transmit
          cadence that feeds the Teensy's watchdog, and the request bytes the
          Teensy sends back.
Spec    : lib/Vision/PROTOCOL.md
Requires: pyserial
Threads : the Teensy goes safe after LINK_TIMEOUT_MS without a valid frame
          and needs REQUIRED_CONFIRMATION_FRAMES identical frames in a row
          before it acts. Both want a steady stream, not one frame per vision
          decision, so this transmits at a fixed rate from its own thread,
          always sending the latest state. A slow vision frame can never
          stall the link.

Usage
    with TeensyLink() as link:
        link.set_ready(True)
        link.set_phase_beans()
        while running:
            link.set_beans(*run_vision())
            for request in link.take_requests():
                if request == vp.CMD_START_BENEFITS:
                    link.set_phase_benefits()
"""

from __future__ import annotations

import glob
import threading
import time
from typing import List, Optional

try:
    import serial as pyserial
except ImportError:
    raise SystemExit("pyserial is required:  pip install pyserial")

import vision_protocol as vp

STREAM_HZ = 50
MAX_TEXT_LINE = 240
MAX_TEXT_LINES = 200

DEVICE_CANDIDATES = (
    "/dev/ttyTeensy",
    "/dev/tty.usbmodem*",
    "/dev/ttyACM*",
)


def find_teensy() -> Optional[str]:
    for pattern in DEVICE_CANDIDATES:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


class TeensyLink:
    """Streams protocol v2 command frames to the Teensy.

    Every setter is thread-safe and returns immediately: it only updates
    the state that the transmit thread will send on its next tick.
    """

    def __init__(self, device: Optional[str] = None,
                 baud: int = vp.BAUD, stream_hz: int = STREAM_HZ) -> None:
        self.device = device or find_teensy()
        if not self.device:
            raise RuntimeError(
                "no Teensy found — looked for: " + ", ".join(DEVICE_CANDIDATES)
            )

        self._ser = pyserial.Serial(self.device, baud, timeout=0, write_timeout=1)
        self._period = 1.0 / max(1, stream_hz)
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._phase = vp.PHASE_IDLE
        self._payload = 0x00
        self._status = 0x00
        self._seq = 0
        self._requests: List[int] = []

        self._messages: List[str] = []
        self._partial = bytearray()

        self._error: Optional[BaseException] = None

        self._tx = threading.Thread(target=self._run, name="teensy-link",
                                    daemon=True)
        self._tx.start()

    # Health
    @property
    def healthy(self) -> bool:
        return self._error is None and self._tx.is_alive()

    @property
    def error(self) -> Optional[BaseException]:
        return self._error

    # Context manager
    def __enter__(self) -> "TeensyLink":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def close(self) -> None:
        if self.healthy:
            self.set_idle()
            time.sleep(self._period * 3) 
        self._stop.set()
        self._tx.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass

    # Phase + command setters
    def set_idle(self) -> None:
        #Nothing running. Every actuator goes home, doors shut.
        self._set(vp.PHASE_IDLE, 0x00)

    def halt(self) -> None:
        #Emergency stop. The Teensy acts on the first frame, with no confirmation delay.
        self._set(vp.PHASE_HALT, 0x00)

    def set_phase_beans(self) -> None:
        #Enter the beans stage with everything at rest.
        self._set(vp.PHASE_BEANS, vp.make_beans_payload(False, False, vp.SEP_NEUTRAL))

    def set_phase_benefits(self) -> None:
        # Enter the benefits stage with both doors shut.
        self._set(vp.PHASE_BENEFITS, vp.make_benefits_payload(False, False))

    def set_beans(self, upper: bool, lower: bool, separator: int) -> None:
        # The beans-stage command. Also switches into BEANS if needed, so
        # vision code never has to remember the phase separately.
        self._set(vp.PHASE_BEANS, vp.make_beans_payload(upper, lower, separator))

    def set_benefits(self, door1_open: bool, door2_open: bool) -> None:
        #The benefits-stage command. Also switches into BENEFITS.
        
        self._set(vp.PHASE_BENEFITS,
                  vp.make_benefits_payload(door1_open, door2_open))

    #Health reporting (the STATUS byte in every frame)
    def set_ready(self, ready: bool) -> None:
        self._set_status_bit(vp.STATUS_ORIN_READY, ready)

    def set_beans_running(self, running: bool) -> None:
        self._set_status_bit(vp.STATUS_BEANS_RUNNING, running)

    def set_benefits_running(self, running: bool) -> None:
        self._set_status_bit(vp.STATUS_BENEFITS_RUNNING, running)

    def set_fault(self, mask: int, faulted: bool = True) -> None:
        self._set_status_bit(mask, faulted)

    @property
    def status(self) -> int:
        with self._lock:
            return self._status

    # Requests coming back from the Teensy
    def take_messages(self) -> List[str]:
        with self._lock:
            pending, self._messages = self._messages, []
        return pending

    def take_requests(self) -> List[int]:
        #Pop every request byte received since the last call.
        #Values are vp.CMD_START_BEANS / CMD_STOP / CMD_STATUS /
        #CMD_START_BENEFITS. They are requests, not orders — the Orin stays
        #the authority on which phase is actually running.
        
        with self._lock:
            pending, self._requests = self._requests, []
        return pending

    # Internals
    def _flush_text(self) -> None:
        if not self._partial:
            return
        line = self._partial.decode("ascii", "replace").rstrip()
        self._partial = bytearray()
        if not line:
            return
        with self._lock:
            self._messages.append(line)
            # Never let a chatty sketch grow this without bound.
            del self._messages[:-MAX_TEXT_LINES]

    def _set(self, phase: int, payload: int) -> None:
        with self._lock:
            self._phase = phase
            self._payload = payload

    def _set_status_bit(self, mask: int, on: bool) -> None:
        with self._lock:
            if on:
                self._status |= mask
            else:
                self._status &= ~mask & 0xFF

    def _run(self) -> None:
        try:
            self._pump()
        except BaseException as exc:
            self._error = exc
            self._stop.set()

    def _pump(self) -> None:
        next_tx = time.monotonic()
        while not self._stop.is_set():
            waiting = self._ser.in_waiting
            if waiting:
                for byte in self._ser.read(waiting):
                    if byte in vp.CMD_NAMES:
                        with self._lock:
                            self._requests.append(byte)
                    elif byte in (0x0A, 0x0D):
                        self._flush_text()
                    elif 0x20 <= byte < 0x7F:
                        self._partial.append(byte)
                        if len(self._partial) >= MAX_TEXT_LINE:
                            self._flush_text()

            now = time.monotonic()
            if now >= next_tx:
                with self._lock:
                    self._seq = vp.next_seq(self._seq)
                    frame = vp.build_frame(self._seq, self._phase,
                                           self._payload, self._status)
                self._ser.write(frame)
                next_tx += self._period
                if next_tx < now:
                    next_tx = now + self._period
            else:
                time.sleep(min(0.002, next_tx - now))
