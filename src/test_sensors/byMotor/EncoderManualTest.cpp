/*
 * EncoderManualTest.cpp -- prueba MANUAL de los 4 encoders (UL, UR, LL, LR).
 * Los motores NO se mueven: gira cada rueda a mano y mira el monitor serial.
 * Sin RTOS, sin DCMotor/Drive, sin BNO: setup()/loop() plano.
 *
 * Cuadratura de A y B (CHANGE en los dos), igual que FourWheelsPID:
 *   ticks = conteo CON SIGNO, positivo cuando A adelanta a B.
 *   Al girar la rueda para un lado los ticks SUBEN, para el otro BAJAN.
 * Si una rueda no sube/baja (o solo se queda cerca de 0), aparecen los
 * flancos por canal y las transiciones invalidas para saber por que:
 *   A~B e invalidos~0  -> encoder sano
 *   B=0 (o A=0)        -> ese canal no llega al Teensy
 *   invalidos altos    -> A y B cambian juntos (ruido / hilos mal)
 *
 * Pines directos (mismos que usa Drive.cpp / testDrive / FourWheelsPID).
 * Envia 'r' por el monitor serial para poner los 4 contadores en 0.
 *
 * pio run -e encoder_manual_test -t upload -t monitor
 */

#include <Arduino.h>

constexpr uint8_t NUM_WHEELS = 4;
enum Wheel : uint8_t { UL = 0, UR = 1, LL = 2, LR = 3 };

struct EncPins { const char* name; uint8_t a, b; };

// PINES DIRECTOS (temporal): los mismos que usa Drive.cpp / FourWheelsPID.
// Equivalente en Pins::kEncoders entre corchetes.
constexpr EncPins ENC[NUM_WHEELS] = {
    { "UL",  0,  1 },   // A/B invertidos como en Drive.cpp (signo al reves)
    { "UR", 13,  2 },   // A/B invertidos como en Drive.cpp (signo al reves)
    { "LL", 32, 31 },   // A=[4], B=[5]
    { "LR", 21, 20 },   // A=[6], B=[7]
};

struct EncoderState {
    volatile long     ticks;
    volatile uint8_t  prev_ab;
    volatile uint32_t edgesA, edgesB, bad;
};
EncoderState enc[NUM_WHEELS];

// indice = (estado_previo << 2) | estado_nuevo, estado = (A << 1) | B
// secuencia positiva: 00 -> 10 -> 11 -> 01 -> 00 ; transiciones invalidas = 0
constexpr int8_t QUAD_STEP[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

template <uint8_t I>
void encoderISR()
{
    EncoderState& e = enc[I];
    uint8_t ab   = (digitalRead(ENC[I].a) << 1) | digitalRead(ENC[I].b);
    uint8_t diff = ab ^ e.prev_ab;
    int8_t  step = QUAD_STEP[(e.prev_ab << 2) | ab];

    if (diff & 2) e.edgesA = e.edgesA + 1;
    if (diff & 1) e.edgesB = e.edgesB + 1;
    if (step == 0) e.bad = e.bad + 1;   // A y B cambian juntos, o rebote sin cambio
    e.ticks   = e.ticks + step;
    e.prev_ab = ab;
}

void resetAll()
{
    noInterrupts();
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        enc[i].ticks = 0;
        enc[i].edgesA = enc[i].edgesB = enc[i].bad = 0;
    }
    interrupts();
}

long lastTicks[NUM_WHEELS] = {0, 0, 0, 0};
uint32_t lastPrint = 0;
constexpr uint32_t PRINT_MS = 100;

void setup()
{
    Serial.begin(115200);

    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        pinMode(ENC[i].a, INPUT_PULLUP);
        pinMode(ENC[i].b, INPUT_PULLUP);
        enc[i].prev_ab = (digitalRead(ENC[i].a) << 1) | digitalRead(ENC[i].b);
    }

    attachInterrupt(digitalPinToInterrupt(ENC[UL].a), encoderISR<UL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[UL].b), encoderISR<UL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[UR].a), encoderISR<UR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[UR].b), encoderISR<UR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[LL].a), encoderISR<LL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[LL].b), encoderISR<LL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[LR].a), encoderISR<LR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC[LR].b), encoderISR<LR>, CHANGE);

    Serial.println("Encoders manual: gira cada rueda a mano. 'r' = reset.");
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        Serial.print(ENC[i].name);
        Serial.print(": A="); Serial.print(ENC[i].a);
        Serial.print(" B=");  Serial.println(ENC[i].b);
    }
}

void loop()
{
    while (Serial.available()) {
        if (Serial.read() == 'r') {
            resetAll();
            for (uint8_t i = 0; i < NUM_WHEELS; i++) lastTicks[i] = 0;
            Serial.println("--- reset ---");
        }
    }

    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        noInterrupts();
        long     t  = enc[i].ticks;
        uint32_t ea = enc[i].edgesA, eb = enc[i].edgesB, bd = enc[i].bad;
        interrupts();

        long d = t - lastTicks[i];
        lastTicks[i] = t;

        Serial.print(ENC[i].name);
        Serial.print(d > 0 ? " [+] " : d < 0 ? " [-] " : " [ ] ");
        Serial.print("ticks="); Serial.print(t);
        Serial.print(" (A="); Serial.print(ea);
        Serial.print(" B=");  Serial.print(eb);
        Serial.print(" inv="); Serial.print(bd);
        Serial.print(")   ");
    }
    Serial.println();
}
