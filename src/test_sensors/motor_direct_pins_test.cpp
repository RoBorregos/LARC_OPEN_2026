/**
 * @file motor_direct_pins_test.cpp
 * @brief Diagnostico standalone (sin RTOS, sin clase DCMotor) de los 4
 *        motores del chasis (M1-M4): mueve IN1/IN2/PWM directo con
 *        pinMode/digitalWrite/analogWrite sobre los pines de
 *        include/pins.h (Pins::kUpperMotors/kLowerMotors/kPwmPin), sin
 *        encoder, sin inversion logica (invert), sin BNO/PID/EKF.
 *
 *        Util para confirmar cableado y sentido de giro pin por pin
 *        (p.ej. despues de cambiar el orden IN1/IN2 en pins.h) sin que la
 *        capa DCMotor (invert, PID, etc.) enmascare el resultado.
 *
 *        M5 (elevador) no se incluye aqui: ver motor_test.cpp para eso.
 *
 * pio run -e motor_direct_pins_test -t upload -t monitor
 */

#include <Arduino.h>
#include "pins.h"

constexpr int      TEST_PWM  = 120; // 0-255
constexpr uint32_t RUN_MS    = 1500;
constexpr uint32_t PAUSE_MS  = 500;

struct DirectMotor
{
    const char* name;
    uint8_t     in1;
    uint8_t     in2;
    uint8_t     pwm;
};

// Mismo mapeo de pines que Drive.cpp / motor_test.cpp (M1-M4), pero sin
// pasar por DCMotor: aqui in1/in2 se manejan tal cual estan en pins.h,
// sin ningun flag "invert".
static DirectMotor motors[] = {
    {"M1 upper-left ", Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[0]},
    {"M2 upper-right", Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[1]},
    {"M3 lower-left ", Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[2]},
    {"M4 lower-right", Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[3]},
};

constexpr uint8_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

void driveDirect(const DirectMotor& m, bool forward, int pwm)
{
    // FORWARD: IN1=HIGH, IN2=LOW. BACKWARD: IN1=LOW, IN2=HIGH.
    digitalWrite(m.in1, forward ? HIGH : LOW);
    digitalWrite(m.in2, forward ? LOW : HIGH);
    analogWrite(m.pwm, pwm);
}

void stopDirect(const DirectMotor& m)
{
    analogWrite(m.pwm, 0);
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
}

void testOneByOne()
{
    Serial.println();
    Serial.println("=== Prueba individual (pines directos, uno por uno) ===");

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print("  ");
        Serial.print(motors[i].name);
        Serial.printf(" (IN1=%u, IN2=%u, PWM=%u)\n", motors[i].in1, motors[i].in2, motors[i].pwm);

        Serial.println("    adelante (IN1=HIGH, IN2=LOW)");
        driveDirect(motors[i], true, TEST_PWM);
        delay(RUN_MS);
        stopDirect(motors[i]);
        delay(PAUSE_MS);

        Serial.println("    atras    (IN1=LOW, IN2=HIGH)");
        driveDirect(motors[i], false, TEST_PWM);
        delay(RUN_MS);
        stopDirect(motors[i]);
        delay(PAUSE_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   MOTOR TEST (pines directos, M1-M4 chasis)");
    Serial.println("================================");

    analogWriteResolution(8);

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        pinMode(motors[i].in1, OUTPUT);
        pinMode(motors[i].in2, OUTPUT);
        pinMode(motors[i].pwm, OUTPUT);
        stopDirect(motors[i]);
    }

    Serial.println("Pines inicializados (sin DCMotor, sin invert).");
}

void loop()
{
    testOneByOne();

    Serial.println();
    Serial.println("Ciclo completo. Repitiendo en 3s...");
    delay(3000);
}
