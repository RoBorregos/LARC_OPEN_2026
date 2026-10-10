/**
   @file CommandConfirm.hpp
   @date 2026-08-19
  
   @brief The firmware's ONE confirmation mechanism.
  
    A single command frame can be wrong — a glitched bit that survived the
    CRC, a vision frame that flickered, a half-seen bean. Acting on the
    first frame makes the servos twitchy; acting on N identical frames in a
    row makes them calm. This struct is where that trade-off lives, and it
    is the only place in the firmware that counts frames.
  
    It confirms the pair (phase, payload) TOGETHER, as one value. That
    matters: if the phase had its own filter and the actuator command had
    another, a stage change would pay both delays back to back. Confirming
    the pair means a phase change and the command that arrives with it are
    confirmed in the same window.
 */

#ifndef COMMAND_CONFIRM_HPP
#define COMMAND_CONFIRM_HPP

#include <stdint.h>

struct CommandConfirm
{
    uint8_t candPhase;
    uint8_t candPayload;
    uint8_t count;

    uint8_t appliedPhase;
    uint8_t appliedPayload;
    bool    haveApplied;

    void reset(uint8_t phase, uint8_t payload)
    {
        candPhase      = phase;
        candPayload    = payload;
        count          = 0;
        appliedPhase   = phase;
        appliedPayload = payload;
        haveApplied    = true;
    }

    /// Returns true exactly when (phase, payload) becomes newly confirmed.
    bool sample(uint8_t phase, uint8_t payload, uint8_t threshold)
    {
        if (threshold == 0)
            threshold = 1;

        if (haveApplied && phase == appliedPhase && payload == appliedPayload)
        {
            // Already the active command. Clear any pending candidate so a
            // half-confirmed different command cannot survive an
            // interruption and jump in later.
            candPhase   = phase;
            candPayload = payload;
            count       = 0;
            return false;
        }

        if (phase != candPhase || payload != candPayload)
        {
            // Any change — including a phase change — restarts the count.
            candPhase   = phase;
            candPayload = payload;
            count       = 1;
        }
        else if (count < 255)
        {
            count++;
        }

        if (count >= threshold)
        {
            appliedPhase   = candPhase;
            appliedPayload = candPayload;
            haveApplied    = true;
            count          = 0;
            return true;
        }
        return false;
    }

    /// How many more identical frames this candidate still needs. For
    /// debug printouts only.
    uint8_t remaining(uint8_t threshold) const
    {
        return (count >= threshold) ? 0 : (uint8_t)(threshold - count);
    }
};

#endif
