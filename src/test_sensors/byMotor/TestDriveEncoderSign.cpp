/**
 * @file TestDriveEncoderSign.cpp
 * @brief testDrive + signo de encoders: usa el stack COMPLETO de Drive (BNO085
 *        yaw-hold PID + cinematica omni + EKF), igual que testDrive.cpp, pero
 *        en vez de recorrer un cuadrado solo manda drive.forward(speed) todo
 *        el tiempo, imprimiendo en vivo los metros (con signo) de cada rueda
 *        (getM1..M4Meters). Sirve para confirmar que, con el codigo de
 *        produccion (Drive), las 4 ruedas cuentan en el sentido correcto al
 *        ir siempre hacia adelante -- no solo con pines/PWM sueltos.
 *
 * Todas las ruedas deben marcar [+] (metros subiendo) yendo adelante, porque
 * diametro y PPR son positivos: el signo de metros = signo del conteo crudo
 * del encoder (Encoder.h, sin decodificador propio). drive.begin() ya pone
 * los 4 encoders en 0 (resetOdometry).
 *
 * pio run -e test_drive_encoder_sign -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive drive;

static constexpr float kSpeed   = Constants::PID::kcurrentVelocity;
static constexpr uint32_t PRINT_MS = 200;

float lastM1 = 0.0f, lastM2 = 0.0f, lastM3 = 0.0f, lastM4 = 0.0f;
uint32_t lastPrint = 0;

char signChar(float delta)
{
    if (delta > 0.0005f) return '+';
    if (delta < -0.0005f) return '-';
    return ' ';
}

void setup() {
    Serial.begin(115200);
    delay(500);

    drive.begin();
    drive.holdYaw(true);
    drive.setTargetYaw(drive.getYaw());

    Serial.println("=== testDrive + signo de encoders: siempre ADELANTE ===");
    Serial.println("Esperado: M1(UL) M2(UR) M3(LL) M4(LR) todas en [+].");
    Serial.println();
}

void loop() {
    drive.update();
    drive.forward(kSpeed);

    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    float m1 = drive.getM1Meters();
    float m2 = drive.getM2Meters();
    float m3 = drive.getM3Meters();
    float m4 = drive.getM4Meters();

    char c1 = signChar(m1 - lastM1);
    char c2 = signChar(m2 - lastM2);
    char c3 = signChar(m3 - lastM3);
    char c4 = signChar(m4 - lastM4);
    lastM1 = m1; lastM2 = m2; lastM3 = m3; lastM4 = m4;

    Serial.print("UL["); Serial.print(c1); Serial.print("]m="); Serial.print(m1, 3);
    Serial.print("  UR["); Serial.print(c2); Serial.print("]m="); Serial.print(m2, 3);
    Serial.print("  LL["); Serial.print(c3); Serial.print("]m="); Serial.print(m3, 3);
    Serial.print("  LR["); Serial.print(c4); Serial.print("]m="); Serial.print(m4, 3);
    Serial.println();
}
