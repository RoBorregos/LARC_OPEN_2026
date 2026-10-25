#include "WheelSpeedController.hpp"
#include <math.h>

using namespace Constants::WheelPID;

WheelSpeedController::WheelSpeedController(const Gains& gains)
    : g_(gains)
{}

void WheelSpeedController::reset(long ticks, uint32_t nowUs)
{
    lastTicks_ = ticks;
    lastUs_    = nowUs;
    rpmFilt_   = 0.0f;
    spRamped_  = 0.0f;
    integral_  = 0.0f;
    lastError_ = 0.0f;
    output_    = 0;
}

int WheelSpeedController::update(float targetRpm, long ticks, uint32_t nowUs)
{
    const uint32_t dtUs = nowUs - lastUs_;
    if (dtUs == 0) return output_;
    const float dt = dtUs * 1e-6f;

    if (fabsf(targetRpm) < kMinRpm) targetRpm = 0.0f;

    const long dTicks = ticks - lastTicks_;
    lastTicks_ = ticks;
    lastUs_    = nowUs;

    // Edges per second -> rpm, same units as measureRPM() in the calibration sketches
    const float rpmRaw = (float)labs(dTicks) * 60.0f / (dt * 4.0f * g_.ppr);
    rpmFilt_ += kRpmAlpha * (rpmRaw - rpmFilt_);

    // Ramp only when speeding up; slowing down and stopping are immediate
    const float prevSp = spRamped_;
    if (targetRpm * spRamped_ < 0.0f) spRamped_ = 0.0f;
    if (fabsf(targetRpm) <= fabsf(spRamped_)) {
        spRamped_ = targetRpm;
    } else {
        const float step = fminf(kSpRamp * dt, fabsf(targetRpm) - fabsf(spRamped_));
        spRamped_ += copysignf(step, targetRpm);
    }

    if (spRamped_ == 0.0f || prevSp * spRamped_ < 0.0f) {
        integral_  = 0.0f;
        lastError_ = 0.0f;
    }
    if (spRamped_ == 0.0f) {
        output_ = 0;
        return output_;
    }

    // Work on magnitudes in the commanded direction, output never reverses the wheel
    const float sp    = fabsf(spRamped_);
    const float error = sp - rpmFilt_;
    const float ff    = g_.ffOffset + g_.ffSlope * sp;
    const float deriv = (error - lastError_) / dt;
    const float u     = ff + g_.kp * error + g_.ki * integral_ + g_.kd * deriv;

    // Anti-windup: integrate only while not saturated
    if (u > 0.0f && u < 255.0f && g_.ki > 0.0f) {
        integral_ += error * dt;
        const float lim = g_.iMaxPwm / g_.ki;
        integral_ = constrain(integral_, -lim, lim);
    }
    lastError_ = error;

    const int pwm = (int)constrain(u, 0.0f, 255.0f);
    output_ = (spRamped_ > 0.0f) ? pwm : -pwm;
    return output_;
}
