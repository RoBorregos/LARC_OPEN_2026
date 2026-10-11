/**
   @file VisionProtocol.hpp
   @date 2026-08-19
  
   @brief Portable codec for the Orin -> Teensy command protocol v2.
          Framing, CRC-8, phase-scoped payload decoding, sequence and
          duplicate handling, and the link watchdog clock.

    Frame, 8 bytes:
  
        AA 55 | VER | SEQ | PHASE | PAYLOAD | STATUS | CRC8
  
    PAYLOAD means different things in different phases.
 */

#ifndef VISION_PROTOCOL_HPP
#define VISION_PROTOCOL_HPP

#include <stdint.h>
#include <stddef.h>

namespace VisionProto
{

//Frame layout
constexpr uint8_t kSof1     = 0xAA;
constexpr uint8_t kSof2     = 0x55;
constexpr uint8_t kVersion  = 0x02;
constexpr uint8_t kFrameLen = 8;

enum Offset : uint8_t
{
    OFS_SOF1    = 0,
    OFS_SOF2    = 1,
    OFS_VERSION = 2,
    OFS_SEQ     = 3,
    OFS_PHASE   = 4,
    OFS_PAYLOAD = 5,
    OFS_STATUS  = 6,
    OFS_CRC     = 7
};
constexpr uint8_t kCrcSpan = 5;

// Phases
// The phase selects what PAYLOAD means, and which actuators are even
// addressable. A benefit door has no representation at all in a BEANS frame
enum class Phase : uint8_t
{
    IDLE     = 0, // nothing running; everything safe
    BEANS    = 1, // intakes and separator live
    BENEFITS = 2, // the two benefit doors live
    HALT     = 3  // emergency stop
};
constexpr uint8_t kPhaseCount = 4;

// BEANS payload
constexpr uint8_t kMaskIntakeUpper  = 0x01;
constexpr uint8_t kMaskIntakeLower  = 0x02;
constexpr uint8_t kMaskSeparator    = 0x0C;
constexpr uint8_t kShiftSeparator   = 2;
constexpr uint8_t kBeansReserved    = 0xF0; // must be zero

enum class SeparatorCode : uint8_t
{
    NEUTRAL = 0,
    LEFT    = 1, // mature
    RIGHT   = 2, // overmature
    INVALID = 3  // accepted, forced to NEUTRAL, and flagged
};

//BENEFITS payload
constexpr uint8_t kMaskBenefit1     = 0x01;
constexpr uint8_t kMaskBenefit2     = 0x02;
constexpr uint8_t kMaskBoxVisible   = 0x04; // any ROI sees a box colour
constexpr uint8_t kBenefitsReserved = 0xF8; // must be zero

//STATUS byte
constexpr uint8_t kStatusMainFault       = 0x01; // critical
constexpr uint8_t kStatusSeparatorFault  = 0x02; // non critical
constexpr uint8_t kStatusBenefitsFault   = 0x04; // non critical
constexpr uint8_t kStatusCameraFault     = 0x08; // critical
constexpr uint8_t kStatusBeansRunning    = 0x10;
constexpr uint8_t kStatusBenefitsRunning = 0x20;
constexpr uint8_t kStatusOrinReady       = 0x40;
constexpr uint8_t kStatusReserved        = 0x80; // must be zero

constexpr uint8_t kStatusCriticalMask = kStatusMainFault | kStatusCameraFault;

// Uplink, Teensy to Orin (bare bytes, no framing)
constexpr uint8_t kCmdStartBeans    = 0xA0;
constexpr uint8_t kCmdStop          = 0xA1;
constexpr uint8_t kCmdStatus        = 0xA2;
constexpr uint8_t kCmdStartBenefits = 0xA3;

// Default watchdog window
constexpr uint32_t kDefaultLinkTimeoutMs = 500;

// Decoded command
struct Command
{
    Phase         phase;
    bool          intakeUpper;
    bool          intakeLower;
    SeparatorCode separator;
    bool          benefit1Open;
    bool          benefit2Open;
    bool          boxVisible;   // BENEFITS: a box is in view, centred or not
    bool          separatorInvalid; // saw 0b11; separator forced NEUTRAL
    uint8_t       status;
    uint8_t       rawPayload; // as received, for the confirmation filter
};

enum class FrameError : uint8_t
{
    NONE = 0,
    BAD_CRC,
    BAD_VERSION,
    BAD_PHASE,
    BAD_RESERVED
};

// Free functions

uint8_t crc8(const uint8_t *data, size_t len);

/// True when payload's reserved bits are legal for that phase.
bool payloadReservedOk(Phase phase, uint8_t payload);

/// True when the byte is a phase this version understands.
bool phaseValid(uint8_t phaseByte);

/// Build the BEANS payload byte.
uint8_t makeBeansPayload(bool intakeUpper, bool intakeLower, SeparatorCode separator);

/// Build the BENEFITS payload byte.
uint8_t makeBenefitsPayload(bool benefit1Open, bool benefit2Open, bool boxVisible = false);

/// Fill out[kFrameLen] with a complete, CRC'd frame.
void buildFrame(uint8_t seq, Phase phase, uint8_t payload, uint8_t status,
                uint8_t out[kFrameLen]);

/// Turn a validated (phase, payload, status) triple into a Command.
Command decodeCommand(Phase phase, uint8_t payload, uint8_t status);

// Byte at a time frame parser
class FrameParser
{
public:
    struct Result
    {
        bool       frame; // a fully valid frame ended on this byte
        FrameError error; // why the candidate was thrown away
        uint8_t    seq;
        Phase      phase;
        uint8_t    payload;
        uint8_t    status;
    };

    FrameParser() : _len(0) {}

    Result feed(uint8_t byte);
    void   reset() { _len = 0; }

private:
    uint8_t _buf[kFrameLen];
    uint8_t _len;

    void _resync(uint8_t dropFirst);
};

// Link manager: parser + sequence + watchdog
struct LinkStats
{
    uint32_t accepted;
    uint32_t rejected;
    uint32_t duplicates;
};

struct LinkEvent
{
    bool       accepted; // valid, new SEQ — cmd is filled in
    bool       duplicate; // valid frame repeating the last SEQ
    FrameError error;
    uint8_t    seq;
    Command    cmd;
};

class LinkManager
{
public:
    explicit LinkManager(uint32_t timeoutMs = kDefaultLinkTimeoutMs);

    LinkEvent feedByte(uint8_t byte, uint32_t nowMs);

    bool checkTimeout(uint32_t nowMs);

    bool             linkUp() const { return _linkUp; }
    const LinkStats &stats()  const { return _stats; }
    void             resetStats()   { _stats = LinkStats{0, 0, 0}; }

private:
    FrameParser _parser;
    uint32_t    _timeoutMs;
    uint32_t    _lastAcceptedMs;
    uint8_t     _lastSeq;
    bool        _haveSeq;
    bool        _linkUp;
    LinkStats   _stats;
};

} // namespace VisionProto

#endif
