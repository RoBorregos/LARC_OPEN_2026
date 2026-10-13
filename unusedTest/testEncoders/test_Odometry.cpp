#include <Arduino.h>
#include "testOdometry.hpp"

OdomMovement movement;

enum class Step : uint8_t
{
    FORWARD,
    RIGHT,
    BACKWARD,
    LEFT,
    DONE
};

Step step = Step::FORWARD;
bool started = false;

void setup()
{
    Serial.begin(115200);
    movement.begin();
}

void loop()
{
    movement.update();

    switch (step)
    {
        case Step::FORWARD:
            if (!started)
            {
                movement.resetPose();
                movement.forward(60.0f);
                started = true;
            }

            if (movement.getForwardProgress() >= 0.50f - 0.03f)
            {
                movement.stop();
                step = Step::RIGHT;
                started = false;
            }
            break;

        case Step::RIGHT:
            if (!started)
            {
                movement.resetPose();
                movement.right(60.0f);
                started = true;
            }

            if (movement.getLateralProgress() >= 0.50f - 0.03f)
            {
                movement.stop();
                step = Step::BACKWARD;
                started = false;
            }
            break;

        case Step::BACKWARD:
            if (!started)
            {
                movement.resetPose();
                movement.backward(60.0f);
                started = true;
            }

            if (movement.getForwardProgress() >= 0.50f - 0.03f)
            {
                movement.stop();
                step = Step::LEFT;
                started = false;
            }
            break;

        case Step::LEFT:
            if (!started)
            {
                movement.resetPose();
                movement.left(60.0f);
                started = true;
            }

            if (movement.getLateralProgress() >= 0.50f - 0.03f)
            {
                movement.stop();
                step = Step::DONE;
                started = false;
            }
            break;

        case Step::DONE:
            movement.stop();
            break;
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();
        Serial.print("x: "); Serial.print(movement.getX(), 3);
        Serial.print(" y: "); Serial.print(movement.getY(), 3);
        Serial.print(" th: "); Serial.print(movement.getThetaDeg(), 2);
        Serial.print(" yaw: "); Serial.println(movement.getYawNow() * 180.0f / PI, 2);
    }
}