/**
 * @file motor_test.cpp
 * @brief Diagnostico standalone de los 5 motores DC (M1-M4 chasis + M5
 *        elevador, ver include/pins.h): mismo cableado/inversion que
 *        Drive.cpp (subsystem/Drive) para M1-M4, pero sin BNO, sin PID,
 *        sin EKF -- solo mueve cada motor uno por uno (adelante/atras)
 *        reportando el delta de encoder para confirmar giro y sentido, y
 *        despues mueve los 5 al mismo tiempo.
 *
 *        M5 (elevador) no tiene encoder propio en el esquematico (usa
 *        limit switches) y su inversion todavia no esta confirmada por
 *        el equipo -- ver Pins::kElevator en pins.h. CUIDADO: este test
 *        no vigila limit switches, asi que corre el elevador con tiempos
 *        cortos y este atento para no forzarlo contra sus topes.
 *
 * pio run -e motor_test -t upload -t monitor
 */

#include <Arduino.h>

#include "motors.hpp"
#include "pins.h"

constexpr int TEST_PWM      = 120; // 0-255
constexpr uint32_t RUN_MS   = 1500;
constexpr uint32_t PAUSE_MS = 500;

// M5 (elevador) no tiene wiring de chasis que copiar: PWM corto y sin
// encoder para no forzarlo contra sus topes durante el diagnostico.
constexpr int      ELEVATOR_TEST_PWM = 120; // 0-255
constexpr uint32_t  ELEVATOR_RUN_MS  = 600;

// Mismo mapeo de pines e inversion que Drive.cpp (m1_ul_, m2_ur_, m3_ll_, m4_lr_).
DCMotor m1_ul(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], true, Pins::kEncoders[1], Pins::kEncoders[0]);
DCMotor m2_ur(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true, Pins::kEncoders[2], Pins::kEncoders[3]);
DCMotor m3_ll(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true, Pins::kEncoders[4], Pins::kEncoders[5]);
DCMotor m4_lr(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], true, Pins::kEncoders[6], Pins::kEncoders[7]);

// M5: elevador. Inversion sin confirmar por el equipo (placeholder: false).
// DCMotor exige 2 pines de encoder pero M5 no tiene encoder de cuadratura
// propio (ver Pins::kElevator en pins.h, usa limit switches) -- se le pasan
// los pines de los limit switches solo para satisfacer el constructor; el
// "encoder delta" de M5 NO es una lectura real y se ignora abajo (ver
// hasEncoder en MotorSlot).
DCMotor m5_elevator(Pins::kElevator[0], Pins::kElevator[1], Pins::kPwmPin[4], false, Pins::kLimitSwitch, Pins::kLimitSwitch2);

struct MotorSlot
{
    const char* name;
    DCMotor*    motor;
    int         testPwm;
    uint32_t    runMs;
    bool        hasEncoder;
};

MotorSlot motors[] = {
    {"M1 upper-left ", &m1_ul,       TEST_PWM,          RUN_MS,          true},
    {"M2 upper-right", &m2_ur,       TEST_PWM,          RUN_MS,          true},
    {"M3 lower-left ", &m3_ll,       TEST_PWM,          RUN_MS,          true},
    {"M4 lower-right", &m4_lr,       TEST_PWM,          RUN_MS,          true},
    {"M5 elevator   ", &m5_elevator, ELEVATOR_TEST_PWM, ELEVATOR_RUN_MS, false},
};

constexpr uint8_t NUM_MOTORS         = sizeof(motors) / sizeof(motors[0]);
constexpr uint8_t NUM_CHASSIS_MOTORS = 4; // M1-M4; M5 (elevador) se prueba solo individual

void printEncoderDelta(DCMotor& motor, bool hasEncoder)
{
    if (hasEncoder)
    {
        Serial.println(motor.getEncoderCount());
    }
    else
    {
        Serial.println("N/A (sin encoder)");
    }
}

void runDirection(DCMotor& motor, const char* label, int pwm, uint32_t runMs, bool hasEncoder)
{
    motor.resetEncoder();
    motor.move(pwm);
    delay(runMs);

    motor.stop();

    Serial.print("    ");
    Serial.print(label);
    Serial.print(": encoder delta = ");
    printEncoderDelta(motor, hasEncoder);

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

        runDirection(*motors[i].motor, "adelante", +motors[i].testPwm, motors[i].runMs, motors[i].hasEncoder);
        runDirection(*motors[i].motor, "atras   ", -motors[i].testPwm, motors[i].runMs, motors[i].hasEncoder);
    }
}

void testAllTogether(int pwm, const char* label)
{
    Serial.println();
    Serial.print("=== Todos juntos (chasis M1-M4): ");
    Serial.print(label);
    Serial.println(" ===");

    for (uint8_t i = 0; i < NUM_CHASSIS_MOTORS; i++)
    {
        motors[i].motor->resetEncoder();
        motors[i].motor->move(pwm);
    }

    delay(RUN_MS);

    for (uint8_t i = 0; i < NUM_CHASSIS_MOTORS; i++)
    {
        Serial.print("    ");
        Serial.print(motors[i].name);
        Serial.print(": encoder delta = ");
        printEncoderDelta(*motors[i].motor, motors[i].hasEncoder);
    }

    for (uint8_t i = 0; i < NUM_CHASSIS_MOTORS; i++)
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
    Serial.println("   MOTOR TEST (M1-M4 chasis + M5 elevador)");
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
