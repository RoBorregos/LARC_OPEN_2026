/*
 * SingleMotorEncoderTest.cpp -- UN motor a la vez, movido por PWM real
 * (adelante y luego atras), imprimiendo los ticks del encoder EN VIVO para
 * ver como suben y bajan. A diferencia de EncoderManualTest (gira a mano,
 * sin PWM), aqui el motor gira solo: sirve para confirmar si el canal B de
 * LL/LR falla tambien bajo vibracion/carga real del motor, no solo a mano.
 * Sin RTOS, sin DCMotor/Drive, sin BNO: setup()/loop() plano.
 *
 * PINES DIRECTOS (temporal): los mismos que Drive.cpp / testDrive /
 * FourWheelsPID / EncoderManualTest.
 *
 * Cambiar WHICH_WHEEL abajo para probar otra rueda.
 *
 * pio run -e single_motor_encoder_test -t upload -t monitor
 */

#include <Arduino.h>

// =====================================================================
//  CONFIG (lo que se toca entre pruebas)
// =====================================================================
enum Wheel : uint8_t { LL = 0, LR = 1, UL = 2, UR = 3 };

constexpr Wheel   WHICH_WHEEL = LR;   // <<< CAMBIAR AQUI: LL, LR, UL, UR
constexpr int     TEST_PWM    = 120;  // 0-255
constexpr uint32_t RUN_MS     = 2500; // tiempo girando en cada sentido
constexpr uint32_t PAUSE_MS   = 800;  // motor parado entre sentidos
constexpr uint32_t PRINT_MS   = 100;  // refresco del monitor serial

// =====================================================================
//  Pines por rueda (in1, in2, pwm, motorDir, encA, encB)
//  motorDir: +1 = output>0 -> IN1 HIGH/IN2 LOW ; -1 = invertido
//  (mismo sentido/pines que Drive.cpp: UL/UR/LL invert=true, LR false)
// =====================================================================
struct WheelCfg {
    const char* name;
    uint8_t in1, in2, pwm;
    int8_t  motorDir;
    uint8_t encA, encB;
};

constexpr WheelCfg CFG[4] = {
    { "LL", 37, 38, 10, +1, 32, 31 },
    { "LR", 39, 40, 11, +1, 21, 20 },
    { "UL", 33, 34,  8, +1,  1,  0 },
    { "UR", 36, 35,  9, -1,  2, 13 },
};

constexpr WheelCfg& C = const_cast<WheelCfg&>(CFG[WHICH_WHEEL]);

// =====================================================================
//  Encoder (ISR): cuadratura con signo + diagnostico
// =====================================================================
volatile long     ticks   = 0;
volatile uint8_t  prev_ab = 0;
volatile uint32_t edgesA = 0, edgesB = 0, bad = 0;

// indice = (estado_previo << 2) | estado_nuevo, estado = (A << 1) | B
// secuencia positiva: 00 -> 10 -> 11 -> 01 -> 00 ; transiciones invalidas = 0
constexpr int8_t QUAD_STEP[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

void encoderISR()
{
    uint8_t ab   = (digitalRead(C.encA) << 1) | digitalRead(C.encB);
    uint8_t diff = ab ^ prev_ab;
    int8_t  step = QUAD_STEP[(prev_ab << 2) | ab];

    if (diff & 2) edgesA = edgesA + 1;
    if (diff & 1) edgesB = edgesB + 1;
    if (step == 0) bad = bad + 1;
    ticks   = ticks + step;
    prev_ab = ab;
}

// =====================================================================
//  Motor
// =====================================================================
// fwd = true -> sentido "adelante" de la rueda (mismo criterio que Drive/FourWheelsPID)
void setMotor(bool fwd, int pwm)
{
    bool ih = fwd ? (C.motorDir > 0) : (C.motorDir < 0);
    digitalWrite(C.in1, ih ? HIGH : LOW);
    digitalWrite(C.in2, ih ? LOW : HIGH);
    analogWrite(C.pwm, constrain(pwm, 0, 255));
}

void stopMotor()
{
    digitalWrite(C.in1, LOW);
    digitalWrite(C.in2, LOW);
    analogWrite(C.pwm, 0);
}

// =====================================================================
//  Loop de una pasada: gira RUN_MS imprimiendo ticks en vivo
// =====================================================================
void runDirection(bool fwd, const char* label)
{
    Serial.print("=== ");
    Serial.print(C.name);
    Serial.print(" ");
    Serial.print(label);
    Serial.println(" ===");

    noInterrupts();
    ticks = 0; edgesA = 0; edgesB = 0; bad = 0;
    interrupts();

    setMotor(fwd, TEST_PWM);

    uint32_t t0 = millis();
    uint32_t lastPrint = 0;
    while (millis() - t0 < RUN_MS) {
        if (millis() - lastPrint >= PRINT_MS) {
            lastPrint = millis();

            noInterrupts();
            long     t  = ticks;
            uint32_t ea = edgesA, eb = edgesB, bd = bad;
            interrupts();

            Serial.print("  t="); Serial.print(millis() - t0);
            Serial.print("ms  ticks="); Serial.print(t);
            Serial.print(" (A="); Serial.print(ea);
            Serial.print(" B="); Serial.print(eb);
            Serial.print(" inv="); Serial.print(bd);
            Serial.println(")");
        }
    }

    stopMotor();

    noInterrupts();
    long final_ticks = ticks;
    interrupts();
    Serial.print("  -> ticks finales = "); Serial.println(final_ticks);
    Serial.println();

    delay(PAUSE_MS);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(C.encA, INPUT_PULLUP);
    pinMode(C.encB, INPUT_PULLUP);
    pinMode(C.in1, OUTPUT);
    pinMode(C.in2, OUTPUT);
    pinMode(C.pwm, OUTPUT);
    analogWriteResolution(8);
    analogWriteFrequency(C.pwm, 25000);

    prev_ab = (digitalRead(C.encA) << 1) | digitalRead(C.encB);
    attachInterrupt(digitalPinToInterrupt(C.encA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C.encB), encoderISR, CHANGE);

    Serial.print("Rueda: "); Serial.print(C.name);
    Serial.print("  in1="); Serial.print(C.in1);
    Serial.print(" in2="); Serial.print(C.in2);
    Serial.print(" pwm="); Serial.print(C.pwm);
    Serial.print(" encA="); Serial.print(C.encA);
    Serial.print(" encB="); Serial.println(C.encB);
    Serial.println("ticks deben SUBIR en adelante y BAJAR en atras.");
    Serial.println();
}

void loop()
{
    runDirection(true,  "ADELANTE");
    runDirection(false, "ATRAS");
}
