/**
 * @file WheelSpeedController.hpp
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Closed-loop RPM control of one wheel: setpoint ramp + feedforward + PI(D)
 *        with anti-windup, same chain as the per-wheel EncoderCalibration sketches.
 */

#ifndef WHEEL_SPEED_CONTROLLER_HPP
#define WHEEL_SPEED_CONTROLLER_HPP

#include <Arduino.h>
#include "constants.h"

class WheelSpeedController
{
public:
    using Gains = Constants::WheelPID::WheelGains;

    explicit WheelSpeedController(const Gains& gains);

    void reset(long ticks, uint32_t nowUs);
    void rebaseTicks(long ticks) { lastTicks_ = ticks; } // after the encoder is zeroed

    // targetRpm is signed (sign = motor direction). Returns signed PWM [-255, 255].
    int update(float targetRpm, long ticks, uint32_t nowUs);

    float getRpm() const { return rpmFilt_; }
    float getSetpoint() const { return spRamped_; }
    int   getOutput() const { return output_; }

private:
    Gains g_;

    long     lastTicks_ = 0;
    uint32_t lastUs_    = 0;

    float rpmFilt_   = 0.0f; // magnitude, like the calibration
    float spRamped_  = 0.0f; // signed
    float integral_  = 0.0f;
    float lastError_ = 0.0f;
    int   output_    = 0;
};

#endif
