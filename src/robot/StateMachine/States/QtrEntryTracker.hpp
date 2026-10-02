/*
*@author:  Ximena Patricia García Magdaleno
* QtrEntryTracker.hpp
* Confirms a line on a QTR only if it enters from one end and progresses to the center.
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class QtrEntryTracker {
public:
    enum class Phase : uint8_t { IDLE, ENTERING, TRACKING, ON };

    struct Config {
        bool     entryAtHighIndex;   // true: C6 is the first sensor to touch the line
        uint16_t onThreshold;        // per-sensor hysteresis (norm 0..1000)
        uint16_t offThreshold;
        float    entryMaxProgress;   // first sensors (0 = entry sensor) accepted as entry
        float    centerProgress;
        float    centerTolerance;
        float    maxRegress;         // tolerated backwards jitter, in sensors
        uint8_t  confirmReads;       // ENTERING -> TRACKING (rejects single-read spikes)
        uint8_t  onReads;            // consecutive reads inside the center window
        uint8_t  lostReads;          // consecutive reads without signal -> IDLE
    };

    explicit QtrEntryTracker(QTR& qtr, const Config& cfg) : qtr_(qtr), cfg_(cfg) {}

    void reset() {
        phase_ = Phase::IDLE;
        for (uint8_t i = 0; i < QTR::N; i++) sensorOn_[i] = false;
        maxProgress_ = 0.0f;
        progress_    = 0.0f;
        counter_     = 0;
        lostCount_   = 0;
    }

    Phase update() {
        if (phase_ == Phase::ON)
            return phase_;

        const bool seen = readProgress(progress_);

        if (!seen) {
            if (phase_ == Phase::ENTERING) {
                toIdle();
            } else if (phase_ == Phase::TRACKING && ++lostCount_ >= cfg_.lostReads) {
                toIdle();
            }
            return phase_;
        }
        lostCount_ = 0;

        switch (phase_) {
        case Phase::IDLE:
            // A line that shows up directly in the center is a reflection/smudge.
            if (progress_ <= cfg_.entryMaxProgress) {
                phase_       = Phase::ENTERING;
                counter_     = 1;
                maxProgress_ = progress_;
            }
            break;

        case Phase::ENTERING:
            if (progress_ < maxProgress_ - cfg_.maxRegress) {
                toIdle();
                break;
            }
            maxProgress_ = max(maxProgress_, progress_);
            if (++counter_ >= cfg_.confirmReads) {
                phase_   = Phase::TRACKING;
                counter_ = 0;
            }
            break;

        case Phase::TRACKING:
            if (progress_ < maxProgress_ - cfg_.maxRegress) {
                toIdle();
                break;
            }
            maxProgress_ = max(maxProgress_, progress_);

            if (fabsf(progress_ - cfg_.centerProgress) <= cfg_.centerTolerance) {
                if (++counter_ >= cfg_.onReads)
                    phase_ = Phase::ON;
            } else if (progress_ > cfg_.centerProgress + cfg_.centerTolerance) {
                // Crossed the window faster than onReads: the progression was still valid.
                phase_ = Phase::ON;
            } else {
                counter_ = 0;
            }
            break;

        case Phase::ON:
            break;
        }
        return phase_;
    }

    Phase phase() const { return phase_; }
    bool  isOn() const { return phase_ == Phase::ON; }
    float progress() const { return progress_; }

private:
    QTR&   qtr_;
    Config cfg_;

    Phase phase_ = Phase::IDLE;
    bool  sensorOn_[QTR::N] = {};
    float maxProgress_ = 0.0f;
    float progress_    = 0.0f;
    uint8_t counter_   = 0;
    uint8_t lostCount_ = 0;

    void toIdle() {
        phase_       = Phase::IDLE;
        maxProgress_ = 0.0f;
        counter_     = 0;
        lostCount_   = 0;
    }

    // Progress in sensors from the entry end (0 = entry sensor, N-1 = far end).
    bool readProgress(float& out) {
        const uint16_t* norm = qtr_.getNorm();
        float sumW = 0.0f, sumWI = 0.0f;

        const uint8_t n = qtr_.size();
        for (uint8_t i = 0; i < n; i++) {
            if (sensorOn_[i]) {
                if (norm[i] <= cfg_.offThreshold) sensorOn_[i] = false;
            } else {
                if (norm[i] >= cfg_.onThreshold) sensorOn_[i] = true;
            }
            if (!sensorOn_[i]) continue;

            const float w   = (float)(norm[i] - cfg_.offThreshold) + 1.0f;
            const uint8_t d = cfg_.entryAtHighIndex ? (n - 1 - i) : i;
            sumW  += w;
            sumWI += w * d;
        }

        if (sumW <= 0.0f)
            return false;
        out = sumWI / sumW;
        return true;
    }
};
