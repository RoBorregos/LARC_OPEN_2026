/*
*@author:  Ximena Patricia García Magdaleno
* YawSettle.hpp
* Non-blocking check that the BNO085 yaw stopped moving (robot standing still).
*/
#pragma once
#include <Arduino.h>
#include "constants.h"

class YawSettle {
public:
    void reset(uint32_t now, float yawRad) {
        startMs_  = now;
        stableMs_ = now;
        refYaw_   = yawRad;
    }

    // True once minMs has passed and the yaw stayed within kStableDeg for kStableWindowMs,
    // or after kMaxMs even if it never settled.
    bool update(uint32_t now, float yawRad, uint32_t minMs) {
        namespace cfg = Constants::StateMachine::ImuSettle;
        static constexpr float kStableRad = cfg::kStableDeg * PI / 180.0f;

        const float diff = atan2f(sinf(yawRad - refYaw_), cosf(yawRad - refYaw_));
        if (fabsf(diff) > kStableRad) {
            refYaw_   = yawRad;
            stableMs_ = now;
        }

        if ((now - startMs_) >= cfg::kMaxMs)
            return true;
        return (now - startMs_) >= minMs && (now - stableMs_) >= cfg::kStableWindowMs;
    }

private:
    uint32_t startMs_  = 0;
    uint32_t stableMs_ = 0;
    float    refYaw_   = 0.0f;
};
