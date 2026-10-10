#!/usr/bin/env python3

"""
vision_protocol.py — Orin <-> Teensy protocol v2, pure codec

Purpose : Build and parse the 8-byte command frames the Orin sends the
          Teensy, with nothing else attached. No serial port, no threads, no
          hardware — import it from anywhere and it does the same thing.
Spec    : lib/Vision/PROTOCOL.md
C++ twin: lib/Vision/VisionProtocol.hpp / .cpp — this file and that one must
          stay byte-identical. Change one, change both, then run
          test/vision/protocol_selftest.py.
Runs on : anything with Python 3.8+, no dependencies
Frame   : AA 55 | VER | SEQ | PHASE | PAYLOAD | STATUS | CRC8

PAYLOAD means different things in different phases — that is the whole point
of v2. The intakes and separator are addressable only in BEANS, the two
benefit doors only in BENEFITS; a door has no bit representation at all in a
beans-stage frame.

Usage (see teensy_link.py for the real thing)
    frame = build_beans_frame(seq, upper=True, lower=False,
                              separator=SEP_LEFT, status=STATUS_ORIN_READY)
    port.write(frame)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

# Frame layout
SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x02
FRAME_LEN = 8

OFS_SOF1, OFS_SOF2, OFS_VERSION, OFS_SEQ = 0, 1, 2, 3
OFS_PHASE, OFS_PAYLOAD, OFS_STATUS, OFS_CRC = 4, 5, 6, 7

# Phases
PHASE_IDLE = 0x00
PHASE_BEANS = 0x01
PHASE_BENEFITS = 0x02
PHASE_HALT = 0x03

PHASE_NAMES = {
    PHASE_IDLE: "IDLE",
    PHASE_BEANS: "BEANS",
    PHASE_BENEFITS: "BENEFITS",
    PHASE_HALT: "HALT",
}

# BEANS payload
MASK_INTAKE_UPPER = 0x01
MASK_INTAKE_LOWER = 0x02
MASK_SEPARATOR = 0x0C
SHIFT_SEPARATOR = 2
BEANS_RESERVED = 0xF0

SEP_NEUTRAL = 0
SEP_LEFT = 1 # mature
SEP_RIGHT = 2 # overmature
SEP_INVALID = 3 # reserved; accepted but forced to neutral by the Teensy

SEP_NAMES = {SEP_NEUTRAL: "neutral", SEP_LEFT: "left", SEP_RIGHT: "right",
             SEP_INVALID: "invalid"}

# BENEFITS payload
MASK_BENEFIT_1 = 0x01
MASK_BENEFIT_2 = 0x02
MASK_BOX_VISIBLE = 0x04 # any ROI sees a box colour, centred or not
BENEFITS_RESERVED = 0xF8

# STATUS byte
STATUS_MAIN_FAULT = 0x01 # critical on the Teensy
STATUS_SEPARATOR_FAULT = 0x02
STATUS_BENEFITS_FAULT = 0x04
STATUS_CAMERA_FAULT = 0x08 # critical on the Teensy
STATUS_BEANS_RUNNING = 0x10
STATUS_BENEFITS_RUNNING = 0x20
STATUS_ORIN_READY = 0x40
STATUS_RESERVED = 0x80 # must be zero

# Uplink, Teensy to Orin (bare bytes, no framing)
CMD_START_BEANS = 0xA0
CMD_STOP = 0xA1
CMD_STATUS = 0xA2
CMD_START_BENEFITS = 0xA3

CMD_NAMES = {
    CMD_START_BEANS: "START_BEANS",
    CMD_STOP: "STOP",
    CMD_STATUS: "STATUS",
    CMD_START_BENEFITS: "START_BENEFITS",
}

# Link watchdog on the Teensy. Stream faster than this or it goes safe.
LINK_TIMEOUT_MS = 500

BAUD = 115200


# CRC
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# Payload builders
def make_beans_payload(upper: bool, lower: bool, separator: int) -> int:
    #Pack the BEANS payload byte. Never hand-roll these bits elsewhere.
    payload = 0
    if upper:
        payload |= MASK_INTAKE_UPPER
    if lower:
        payload |= MASK_INTAKE_LOWER
    payload |= (separator & 0x03) << SHIFT_SEPARATOR
    return payload


def make_benefits_payload(door1_open: bool, door2_open: bool,
                          box_visible: bool = False) -> int:
    # Pack the BENEFITS payload byte.
    payload = 0
    if door1_open:
        payload |= MASK_BENEFIT_1
    if door2_open:
        payload |= MASK_BENEFIT_2
    if box_visible:
        payload |= MASK_BOX_VISIBLE
    return payload


def payload_reserved_ok(phase: int, payload: int) -> bool:
    #True when payload's reserved bits are legal for that phase.
    if phase in (PHASE_IDLE, PHASE_HALT):
        return payload == 0x00
    if phase == PHASE_BEANS:
        return (payload & BEANS_RESERVED) == 0
    if phase == PHASE_BENEFITS:
        return (payload & BENEFITS_RESERVED) == 0
    return False


# Frame builders
def build_frame(seq: int, phase: int, payload: int, status: int = 0) -> bytes:
    if phase not in PHASE_NAMES:
        raise ValueError(f"invalid phase 0x{phase:02X}")
    if not payload_reserved_ok(phase, payload):
        raise ValueError(
            f"payload 0x{payload:02X} sets reserved bits for phase "
            f"{PHASE_NAMES[phase]}"
        )
    if status & STATUS_RESERVED:
        raise ValueError("STATUS bit 7 is reserved and must be 0")

    body = bytes([VERSION, seq & 0xFF, phase, payload & 0xFF, status & 0xFF])
    return bytes([SOF1, SOF2]) + body + bytes([crc8(body)])


def build_idle_frame(seq: int, status: int = 0) -> bytes:
    """IDLE: nothing running, everything on the robot goes safe."""
    return build_frame(seq, PHASE_IDLE, 0x00, status)


def build_halt_frame(seq: int, status: int = 0) -> bytes:
    """HALT: emergency stop. The Teensy acts on this immediately, without
    waiting for any confirmation frames."""
    return build_frame(seq, PHASE_HALT, 0x00, status)


def build_beans_frame(seq: int, upper: bool, lower: bool,
                      separator: int, status: int = 0) -> bytes:
    """BEANS: intakes + separator. Benefit doors stay shut all stage."""
    return build_frame(seq, PHASE_BEANS,
                       make_beans_payload(upper, lower, separator), status)


def build_benefits_frame(seq: int, door1_open: bool, door2_open: bool,
                         status: int = 0, box_visible: bool = False) -> bytes:
    """BENEFITS: the two doors. Intakes and separator are parked all stage."""
    return build_frame(seq, PHASE_BENEFITS,
                       make_benefits_payload(door1_open, door2_open, box_visible), status)


# Decoding (for tests, log tools, and anything that reads frames back)
@dataclass
class Command:

    phase: int
    seq: int
    status: int
    payload: int
    intake_upper: bool = False
    intake_lower: bool = False
    separator: int = SEP_NEUTRAL
    door1_open: bool = False
    door2_open: bool = False
    box_visible: bool = False
    separator_invalid: bool = False

    def describe(self) -> str:
        name = PHASE_NAMES.get(self.phase, "?")
        if self.phase == PHASE_BEANS:
            detail = (f"upper={int(self.intake_upper)} "
                      f"lower={int(self.intake_lower)} "
                      f"sep={SEP_NAMES[self.separator]}")
        elif self.phase == PHASE_BENEFITS:
            detail = (f"door1={int(self.door1_open)} door2={int(self.door2_open)} "
                      f"visible={int(self.box_visible)}")
        else:
            detail = "-"
        return (f"seq={self.seq:3d} {name:<8} {detail}  "
                f"status=0x{self.status:02X}")


def decode_command(phase: int, payload: int, status: int, seq: int = 0) -> Command:
    """Turn a validated (phase, payload, status) triple into a Command."""
    cmd = Command(phase=phase, seq=seq, status=status, payload=payload)

    if phase == PHASE_BEANS:
        cmd.intake_upper = bool(payload & MASK_INTAKE_UPPER)
        cmd.intake_lower = bool(payload & MASK_INTAKE_LOWER)
        sep = (payload & MASK_SEPARATOR) >> SHIFT_SEPARATOR
        cmd.separator_invalid = sep == SEP_INVALID
        cmd.separator = SEP_NEUTRAL if cmd.separator_invalid else sep
    elif phase == PHASE_BENEFITS:
        cmd.door1_open = bool(payload & MASK_BENEFIT_1)
        cmd.door2_open = bool(payload & MASK_BENEFIT_2)
        cmd.box_visible = bool(payload & MASK_BOX_VISIBLE)

    return cmd


def parse_frame(frame: bytes) -> Tuple[Optional[Command], str]:
    """Validate and decode one complete 8-byte frame.

    Returns (Command, "") on success or (None, reason). Validation order
    matches the Teensy: SOF, CRC, version, phase, reserved bits.
    """
    if len(frame) != FRAME_LEN:
        return None, f"wrong length {len(frame)}"
    if frame[OFS_SOF1] != SOF1 or frame[OFS_SOF2] != SOF2:
        return None, "bad SOF"
    if crc8(frame[OFS_VERSION:OFS_CRC]) != frame[OFS_CRC]:
        return None, "bad CRC"
    if frame[OFS_VERSION] != VERSION:
        return None, f"bad version 0x{frame[OFS_VERSION]:02X}"

    phase = frame[OFS_PHASE]
    if phase not in PHASE_NAMES:
        return None, f"bad phase 0x{phase:02X}"

    payload = frame[OFS_PAYLOAD]
    status = frame[OFS_STATUS]
    if not payload_reserved_ok(phase, payload) or (status & STATUS_RESERVED):
        return None, "reserved bits set"

    return decode_command(phase, payload, status, frame[OFS_SEQ]), ""


def next_seq(seq: int) -> int:
    #Rolling sequence number, wrapping 255 -> 0.
    return (seq + 1) & 0xFF
