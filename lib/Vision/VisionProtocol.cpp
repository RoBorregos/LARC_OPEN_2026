/**
   @file VisionProtocol.cpp
   @date 2026-08-19
 
   @brief Implementation of the Orin -> Teensy command protocol v2.
 */

#include "VisionProtocol.hpp"

#include <string.h> // memmove

namespace VisionProto
{

// ── CRC-8: poly 0x07 (x^8+x^2+x+1), init 0x00, MSB-first, no reflection,
//    no final XOR. Computed over VER SEQ PHASE PAYLOAD STATUS.
uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Validation helpers

bool phaseValid(uint8_t phaseByte)
{
    return phaseByte < kPhaseCount;
}

bool payloadReservedOk(Phase phase, uint8_t payload)
{
    switch (phase)
    {
        case Phase::IDLE:
        case Phase::HALT:     return payload == 0x00;
        case Phase::BEANS:    return (payload & kBeansReserved)    == 0;
        case Phase::BENEFITS: return (payload & kBenefitsReserved) == 0;
    }
    return false;
}

// Payload builders

uint8_t makeBeansPayload(bool intakeUpper, bool intakeLower, SeparatorCode separator)
{
    uint8_t p = 0;
    if (intakeUpper) p |= kMaskIntakeUpper;
    if (intakeLower) p |= kMaskIntakeLower;
    p |= (uint8_t)(((uint8_t)separator & 0x03) << kShiftSeparator);
    return p;
}

uint8_t makeBenefitsPayload(bool benefit1Open, bool benefit2Open, bool boxVisible)
{
    uint8_t p = 0;
    if (benefit1Open) p |= kMaskBenefit1;
    if (benefit2Open) p |= kMaskBenefit2;
    if (boxVisible)   p |= kMaskBoxVisible;
    return p;
}

void buildFrame(uint8_t seq, Phase phase, uint8_t payload, uint8_t status,
                uint8_t out[kFrameLen])
{
    out[OFS_SOF1]    = kSof1;
    out[OFS_SOF2]    = kSof2;
    out[OFS_VERSION] = kVersion;
    out[OFS_SEQ]     = seq;
    out[OFS_PHASE]   = (uint8_t)phase;
    out[OFS_PAYLOAD] = payload;
    out[OFS_STATUS]  = status;
    out[OFS_CRC]     = crc8(&out[OFS_VERSION], kCrcSpan);
}

// Decode

Command decodeCommand(Phase phase, uint8_t payload, uint8_t status)
{
    Command cmd;
    cmd.phase            = phase;
    cmd.intakeUpper      = false;
    cmd.intakeLower      = false;
    cmd.separator        = SeparatorCode::NEUTRAL;
    cmd.benefit1Open     = false;
    cmd.benefit2Open     = false;
    cmd.boxVisible       = false;
    cmd.separatorInvalid = false;
    cmd.status           = status;
    cmd.rawPayload       = payload;

    if (phase == Phase::BEANS)
    {
        cmd.intakeUpper = (payload & kMaskIntakeUpper) != 0;
        cmd.intakeLower = (payload & kMaskIntakeLower) != 0;

        const uint8_t sepBits = (uint8_t)((payload & kMaskSeparator) >> kShiftSeparator);
        cmd.separatorInvalid  = (sepBits == (uint8_t)SeparatorCode::INVALID);
        cmd.separator = cmd.separatorInvalid ? SeparatorCode::NEUTRAL
                                             : (SeparatorCode)sepBits;
    }
    else if (phase == Phase::BENEFITS)
    {
        cmd.benefit1Open = (payload & kMaskBenefit1) != 0;
        cmd.benefit2Open = (payload & kMaskBenefit2) != 0;
        cmd.boxVisible   = (payload & kMaskBoxVisible) != 0;
    }

    return cmd;
}

//FrameParser

void FrameParser::_resync(uint8_t dropFirst)
{
    uint8_t i = dropFirst;
    while (i < _len && _buf[i] != kSof1)
        i++;

    _len = (uint8_t)(_len - i);
    if (_len > 0)
        memmove(_buf, &_buf[i], _len);
}

FrameParser::Result FrameParser::feed(uint8_t byte)
{
    Result r = { false, FrameError::NONE, 0, Phase::IDLE, 0, 0 };

    _buf[_len++] = byte;
    if (_len < kFrameLen)
    {
        if (_buf[0] != kSof1)
            _resync(0);
        return r;
    }

    // Full window: it must start with the two byte marker.
    if (_buf[OFS_SOF1] != kSof1 || _buf[OFS_SOF2] != kSof2)
    {
        _resync(1);
        return r;
    }

    // Candidate frame. CRC first, corrupt bytes make every other field
    // meaningless, then version, then phase, then the reserved bits.
    const uint8_t phaseByte = _buf[OFS_PHASE];
    const uint8_t payload   = _buf[OFS_PAYLOAD];
    const uint8_t status    = _buf[OFS_STATUS];

    if (crc8(&_buf[OFS_VERSION], kCrcSpan) != _buf[OFS_CRC])
        r.error = FrameError::BAD_CRC;
    else if (_buf[OFS_VERSION] != kVersion)
        r.error = FrameError::BAD_VERSION;
    else if (!phaseValid(phaseByte))
        r.error = FrameError::BAD_PHASE;
    else if (!payloadReservedOk((Phase)phaseByte, payload) || (status & kStatusReserved))
        r.error = FrameError::BAD_RESERVED;

    if (r.error != FrameError::NONE)
    {
        _resync(1);
        return r;
    }

    r.frame   = true;
    r.seq     = _buf[OFS_SEQ];
    r.phase   = (Phase)phaseByte;
    r.payload = payload;
    r.status  = status;
    _len      = 0;
    return r;
}

// LinkManager

LinkManager::LinkManager(uint32_t timeoutMs)
    : _timeoutMs(timeoutMs)
    , _lastAcceptedMs(0)
    , _lastSeq(0)
    , _haveSeq(false)
    , _linkUp(false)
    , _stats{0, 0, 0}
{
}

LinkEvent LinkManager::feedByte(uint8_t byte, uint32_t nowMs)
{
    LinkEvent ev = {};
    ev.error = FrameError::NONE;

    const FrameParser::Result r = _parser.feed(byte);

    if (r.error != FrameError::NONE)
    {
        ev.error = r.error;
        _stats.rejected++;
        return ev; // a rejected frame never refreshes the watchdog
    }
    if (!r.frame)
        return ev; // still collecting

    ev.seq = r.seq;

    if (_haveSeq && r.seq == _lastSeq)
    {
        ev.duplicate = true;
        _stats.duplicates++;
        return ev; // a duplicate never refreshes the watchdog either
    }

    _lastSeq        = r.seq;
    _haveSeq        = true;
    _lastAcceptedMs = nowMs;
    _linkUp         = true;

    ev.accepted = true;
    ev.cmd      = decodeCommand(r.phase, r.payload, r.status);
    _stats.accepted++;
    return ev;
}

bool LinkManager::checkTimeout(uint32_t nowMs)
{
    if (_linkUp && (nowMs - _lastAcceptedMs) >= _timeoutMs)
    {
        _linkUp  = false;
        _haveSeq = false; // a reconnecting sender may restart at any SEQ
        _parser.reset();
        return true;
    }
    return false;
}

} // namespace VisionProto
