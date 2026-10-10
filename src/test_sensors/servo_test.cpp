/**
 * @file servo_test.cpp
 * @brief Diagnostico standalone: mueve un unico servo conectado directo
 *        al pin 8 del Teensy (Servo library, PWM directo, sin PCA9685).
 *        Segun el equipo, pin 8 es el unico servo conectado por el
 *        momento -- OJO: en include/pins.h, el pin 8 esta asignado a
 *        kPwmPin[0] (PWM_M1, motor de chasis upper-left), no a ningun
 *        servo. Confirmar con el equipo si es un cableado temporal para
 *        esta prueba o si el mapeo de pines real cambio.
 *
 * Barre 0 -> 180 -> 0 grados en loop continuo.
 *
 * pio run -e servo_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t SERVO_PIN = 8;

constexpr int ANGLE_MIN = 0;
constexpr int ANGLE_MAX = 180;
constexpr int STEP_DEG  = 1;
constexpr uint32_t STEP_DELAY_MS = 15;

Servo servo;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   SERVO TEST (pin 8, directo)");
    Serial.println("================================");

    servo.attach(SERVO_PIN);
    Serial.print("Servo attached en pin ");
    Serial.println(SERVO_PIN);
}

void loop()
{
    static int angle = ANGLE_MIN;
    static int direction = STEP_DEG;

    servo.write(angle);

    Serial.print("angle=");
    Serial.println(angle);

    angle += direction;

    if (angle >= ANGLE_MAX)
    {
        angle = ANGLE_MAX;
        direction = -STEP_DEG;
    }
    else if (angle <= ANGLE_MIN)
    {
        angle = ANGLE_MIN;
        direction = STEP_DEG;
    }

    delay(STEP_DELAY_MS);
}
