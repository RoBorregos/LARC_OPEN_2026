#pragma once

#include <Arduino.h>

/**
 * Small cooperative scheduler primitive.
 *
 * This intentionally has no RTOS dependency. Later, each call site can be
 * moved almost one-to-one into a FreeRTOS task using vTaskDelayUntil().
 */
class PeriodicRunner
{
public:
    explicit PeriodicRunner(uint32_t periodMs)
        : periodMs_(periodMs), nextRunMs_(0)
    {
    }

    bool due(uint32_t now)
    {
        if (nextRunMs_ == 0)
        {
            nextRunMs_ = now + periodMs_;
            return true;
        }

        if ((int32_t)(now - nextRunMs_) < 0)
            return false;

        // Advance from the previous deadline to avoid accumulating drift.
        do
        {
            nextRunMs_ += periodMs_;
        }
        while ((int32_t)(now - nextRunMs_) >= 0);

        return true;
    }

private:
    uint32_t periodMs_;
    uint32_t nextRunMs_;
};
