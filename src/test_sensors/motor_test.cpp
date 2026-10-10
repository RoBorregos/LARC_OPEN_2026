/**
 * @file motor_test.cpp
 * @brief Diagnostico standalone de los 4 motores DC del chasis (M1-M4, ver
 *        include/pins.h): mismo cableado/inversion que Drive.cpp
 *        (subsystem/Drive), pero sin BNO, sin PID, sin EKF -- solo mueve
 *        cada motor uno por uno (adelante/atras) reportando el delta de
 *        encoder para confirmar giro y sentido, y despues los 4 al mismo
 *        tiempo.
 *
 *        M5 (elevador) queda fuera de este test a proposito: no tiene
 *        encoder de cuadratura propio (usa limit switches) y no se quiere
 *        moverlo aqui -- ver env:motor_direct_pins_test o el subsistema
 *        Elevator para probarlo por separado.
 *
 * pio run -e motor_test -t upload -t monitor
 */

#include <Arduino.h>

#include "motors.hpp"
#include "pins.h"

constexpr int TEST_PWM      = 120; // 0-255
constexpr uint32_t RUN_MS   = 1500;
constexpr uint32_t PAUSE_MS = 500;

// Mismo mapeo de pines e inversion que Drive.cpp (m1_ul_, m2_ur_, m3_ll_, m4_lr_).
DCMotor m1_ul(33, 34, 8, true, Pins::kEncoders[1], Pins::kEncoders[0]);
DCMotor m2_ur(35, 36, 9, true, 2,13); // M2 en codigo es M3 en chasis
DCMotor m3_ll(37, 38, 10, true, Pins::kEncoders[4], Pins::kEncoders[5]); // M3 en codigo es M2 en chasis
DCMotor m4_lr(40, 39, 11, true, Pins::kEncoders[6], Pins::kEncoders[7]);

/*
    m1_ul_(33, 34, 8, true, UL_ENC_A, UL_ENC_B, diameter), //M1
    m2_ur_(37, 38, 10, true, LL_ENC_A, LL_ENC_B, diameter), //M3
    m3_ll_(35, 36, 9, true, UR_ENC_A, UR_ENC_B, diameter), //M2
    m4_lr_(40, 39, 11, true, LR_ENC_A, LR_ENC_B, diameter), //M4
*/

struct MotorSlot
{
    const char* name;
    
    DCMotor*    motor;
};

MotorSlot motors[] = {
    {"M1 upper-left ", &m1_ul},
    {"M2 upper-right", &m2_ur},
    {"M3 lower-left ", &m3_ll},
    {"M4 lower-right", &m4_lr},
};

constexpr uint8_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

void runDirection(DCMotor& motor, const char* label, int pwm, uint32_t runMs)
{
    motor.resetEncoder();
    motor.move(pwm);
    delay(runMs);

    motor.stop();

    Serial.print("    ");
    Serial.print(label);
    Serial.print(": encoder delta = ");
    Serial.println(motor.getEncoderCount());

    delay(PAUSE_MS);
}

void testOneByOne()
{
    Serial.println();
    Serial.println("=== Prueba individual (uno por uno) ===");

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print("  ");
        Serial.println(motors[i].name);

        runDirection(*motors[i].motor, "adelante", +TEST_PWM, RUN_MS);
        runDirection(*motors[i].motor, "atras   ", -TEST_PWM, RUN_MS);
    }
}

void testAllTogether(int pwm, const char* label)
{
    Serial.println();
    Serial.print("=== Todos juntos (chasis M1-M4): ");
    Serial.print(label);
    Serial.println(" ===");

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].motor->resetEncoder();
        motors[i].motor->move(pwm);
    }

    delay(RUN_MS);

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print("    ");
        Serial.print(motors[i].name);
        Serial.print(": encoder delta = ");
        Serial.println(motors[i].motor->getEncoderCount());
    }

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].motor->stop();
    }

    delay(PAUSE_MS);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   MOTOR TEST (M1-M4 chasis)");
    Serial.println("================================");

    analogWriteResolution(8);

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].motor->begin();
    }

    Serial.println("Motores inicializados.");
}

void loop()
{
    testOneByOne();
    testAllTogether(+TEST_PWM, "adelante");
    testAllTogether(-TEST_PWM, "atras");

    Serial.println();
    Serial.println("Ciclo completo. Repitiendo en 3s...");
    delay(3000);
}
