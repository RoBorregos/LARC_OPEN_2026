/**
 * @file motor_test_m1_m3.cpp
 * @brief Diagnostico standalone de M1 (upper-left) y M3 (lower-left)
 *        movidos AL MISMO TIEMPO (ver include/pins.h), mismo cableado/
 *        inversion que Drive.cpp (subsystem/Drive), pero sin BNO, sin
 *        PID, sin EKF -- solo mueve ambos motores juntos (adelante/
 *        atras) reportando el delta de encoder de cada uno para
 *        confirmar giro, sentido y que ambos cuentan parejo.
 *
 * pio run -e motor_test_m1_m3 -t upload -t monitor
 */

#include <Arduino.h>

#include "motors.hpp"
#include "pins.h"

constexpr int TEST_PWM      = 120; // 0-255
constexpr uint32_t RUN_MS   = 1500;
constexpr uint32_t PAUSE_MS = 500;

// Mismo mapeo de pines e inversion que Drive.cpp (m1_ul_, m3_ll_).
DCMotor m1_ul(33, 34, 8, true, Pins::kEncoders[1], Pins::kEncoders[0]);
DCMotor m3_ll(37, 38, 10, true, 2, 13); // M3 en codigo es M2 en chasis

void testM1M3Together(int pwm, const char* label)
{
    Serial.println();
    Serial.print("=== M1 + M3 al mismo tiempo: ");
    Serial.print(label);
    Serial.println(" ===");

    m1_ul.resetEncoder();
    m3_ll.resetEncoder();
    m1_ul.move(pwm);
    m3_ll.move(pwm);

    delay(RUN_MS);

    m1_ul.stop();
    m3_ll.stop();

    Serial.print("    M1 upper-left : encoder delta = ");
    Serial.println(m1_ul.getEncoderCount());

    Serial.print("    M3 lower-left : encoder delta = ");
    Serial.println(m3_ll.getEncoderCount());

    delay(PAUSE_MS);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   MOTOR TEST (M1 + M3 juntos)");
    Serial.println("================================");

    analogWriteResolution(8);

    m1_ul.begin();
    m3_ll.begin();

    Serial.println("Motores inicializados.");
}

void loop()
{
    testM1M3Together(+TEST_PWM, "adelante");
    testM1M3Together(-TEST_PWM, "atras");

    Serial.println();
    Serial.println("Ciclo completo. Repitiendo en 3s...");
    delay(3000);
}
