/**
 * @file LinePD.hpp
 *
 * @brief PD (+ optional I) for line correction, driven by an explicit dt.
 *        D acts on the error through a first-order low-pass (dTauS), I uses
 *        conditional integration as anti-windup, and the output is clamped
 *        to +-outMax and slew-rate limited.
 */

#pragma once

#include <math.h>

class LinePD
{
public:
    struct Gains
    {
        float kp;
        float ki;
        float kd;
        float dTauS;
        float outMax;
        float slewPerS;
    };

    explicit LinePD(const Gains& g) : g_(g) {}

    void reset()
    {
        resetState();
        out_ = 0.0f;
    }

    float update(float err, float dt)
    {
        if (dt <= 0.0f)
            return out_;

        if (first_)
        {
            prevErr_ = err;
            first_   = false;
        }
        dFilt_ += ((err - prevErr_) / dt - dFilt_) * (dt / (g_.dTauS + dt));
        prevErr_ = err;

        float u = g_.kp * err + g_.kd * dFilt_ + g_.ki * integ_;
        if (g_.ki != 0.0f && (fabsf(u) < g_.outMax || u * err < 0.0f))
            integ_ += err * dt;

        command_ = clampf(u, -g_.outMax, g_.outMax);
        return slewTo(command_, dt);
    }

    // Slews the output toward a fixed target and clears the PD memory, so the
    // derivative does not kick when update() resumes.
    float hold(float target, float dt)
    {
        resetState();
        command_ = clampf(target, -g_.outMax, g_.outMax);
        return slewTo(command_, dt);
    }

    float output() const { return out_; }
    float command() const { return command_; }

private:
    Gains g_;
    bool  first_   = true;
    float prevErr_ = 0.0f;
    float dFilt_   = 0.0f;
    float integ_   = 0.0f;
    float command_ = 0.0f;
    float out_     = 0.0f;

    void resetState()
    {
        first_   = true;
        dFilt_   = 0.0f;
        integ_   = 0.0f;
        command_ = 0.0f;
    }

    float slewTo(float target, float dt)
    {
        const float step = g_.slewPerS * dt;
        out_ += clampf(target - out_, -step, step);
        return out_;
    }

    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};
