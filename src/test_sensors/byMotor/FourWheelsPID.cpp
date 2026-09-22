/*
 * FourWheelsPID.cpp -- PID de RPM de las 4 ruedas (LL, LR, UL, UR) al mismo
 * tiempo, con los valores calibrados por separado en
 * LL/LR/UL/UR EncoderCalibration.cpp. Sin RTOS, sin DCMotor/Drive, sin BNO:
 * setup()/loop() plano.
 *
 * Cada rueda usa la misma cadena que su sketch individual:
 *   rpm = promedio de FILTER_SIZE periodos de flanco (A y B con CHANGE) -> EMA
 *   output = FF_OFFSET + FF_SLOPE * sp_rampeado + Kp*error + Ki*integral
 *   (anti-windup: solo integra si el output no esta saturado)
 *
 * PINES DIRECTOS (temporal): los numeros de CFG[] son los de los sketches de
 * calibracion (pares motor+encoder ya comprobados moviendo la rueda y leyendo
 * su rpm). pins.h NO tiene un nombre por rueda (LL/LR/UL/UR) y su M2/M3 esta
 * cruzado respecto a esos sketches; en cada rueda queda anotado el indice de
 * Pins:: equivalente para volver a pins.h cuando se corrija.
 *
 * pio run -e four_wheels_pid -t upload -t monitor
 *
 * ---------------------------------------------------------------------
 * ETAPAS (una sola linea que cambiar entre etapas: setpoint / DIRECTION_CHECK)
 *
 *  Etapa 0  DIRECTION_CHECK = true, ruedas EN EL AIRE, bateria cargada.
 *           Mueve una rueda a la vez (DIR_CHECK_RPM), primero ADELANTE y luego
 *           ATRAS, e imprime el conteo de cuadratura CON SIGNO. Ver el texto de
 *           cada rueda en el monitor serial (no en el plotter). Adelante debe
 *           dar POSITIVO y atras NEGATIVO; si ADELANTE sale NEGATIVO, poner
 *           encFwdSign = -1 (motorDir ya copia el sentido de Drive/testDrive).
 *  Etapa 1  DIRECTION_CHECK = false, setpoint = 45, ruedas EN EL AIRE.
 *  Etapa 2  igual, en el SUELO, bateria cargada.
 *  Etapa 3  en el suelo: setpoint = 30, luego setpoint = 60.
 * ---------------------------------------------------------------------
 */

#include <Arduino.h>

// =====================================================================
//  CONFIG (lo que se toca entre etapas)
// =====================================================================
float setpoint = 45.0f;                 // rpm comun a las 4 ruedas  <<< CAMBIAR AQUI

constexpr bool  DIRECTION_CHECK = true; // true = etapa 0 (una rueda a la vez, texto)
constexpr float DIR_CHECK_RPM   = 45.0f; // rpm en la etapa 0

constexpr bool PRINT_OUTPUTS    = true;  // agrega out_LL,out_LR,out_UL,out_UR al plotter
constexpr bool PLOT_SIGNED_RPM  = false; // true: rpm negativo si la rueda gira al reves de
                                         // lo esperado (usar SOLO despues de la etapa 0)

// Deshabilitar una rueda (queda apagada, su rpm se sigue leyendo): LL, LR, UL, UR
constexpr bool WHEEL_ENABLED[4] = { true, true, true, true };

// =====================================================================
//  Parametros comunes (iguales en los 4 sketches de calibracion)
// =====================================================================
constexpr uint8_t FILTER_SIZE = 12;
constexpr float   Ts          = 0.05f;   // 50 ms
constexpr float   RPM_ALPHA   = 0.2f;    // 0=sin cambio, 1=sin filtro
constexpr float   SP_RAMP     = 90.0f;   // rpm/s
constexpr uint32_t PWM_FREQ_HZ = 25000;

// =====================================================================
//  Configuracion por rueda
// =====================================================================
constexpr uint8_t NUM_WHEELS = 4;
enum Wheel : uint8_t { LL = 0, LR = 1, UL = 2, UR = 3 };

struct WheelCfg {
    const char* name;
    // pines (Pins::)
    uint8_t pwm, in1, in2, encA, encB;
    // PID + feedforward (tabla de calibracion)
    float kp, ki, kd;
    float ffOffset, ffSlope;
    float ppr;          // SIN COMPROBAR
    // sentido
    int8_t motorDir;    // +1: output>0 = IN1 HIGH / IN2 LOW (igual que en la calibracion)
                        // -1: invertido. Poner -1 si la rueda empuja hacia atras.
    int8_t encFwdSign;  // signo del conteo de cuadratura cuando la rueda va "adelante"
                        // (+1 = A adelanta a B). Sale de la etapa 0.
};

// PINES DIRECTOS (temporal): los mismos que usa Drive.cpp (testDrive), que ya
// van en el sentido correcto. motorDir sale de los flags invert de Drive:
// UL/UR/LL invert=true, LR false. Entre corchetes, el equivalente en Pins::.
constexpr WheelCfg CFG[NUM_WHEELS] = {
    // LL: pwm=kPwmPin[1], in1=kLowerMotors[1], in2=kLowerMotors[0],
    //     encA=kEncoders[4], encB=kEncoders[5]
    { "LL", 10, 37, 38,
      32, 31,
      0.5f, 0.6f, 0.0f,   8.0f, 1.85f,  188.0f,   +1, +1 },

    // LR: pwm=kPwmPin[3], in1=kUpperMotors[3], in2=kUpperMotors[2],
    //     encA=kEncoders[6], encB=kEncoders[7]
    { "LR", 11, 39, 40,
      21, 20,
      0.5f, 0.8f, 0.0f,  32.0f, 1.30f,  189.0f,   +1, +1 },

    // UL: pwm=kPwmPin[0], in1=kLowerMotors[2], in2=kLowerMotors[3].
    //     Encoder A=1, B=0 como en Drive (ENA_M1=kEncoders[1], ENB_M1=kEncoders[0]);
    //     la calibracion los tenia al reves, que solo cambia el signo del conteo.
    { "UL", 8, 33, 34,
      1, 0,
      0.5f, 0.6f, 0.0f,  32.0f, 1.05f,  190.0f,   +1, -1 },

    // UR: pwm=kPwmPin[2], in1=kUpperMotors[1], in2=kUpperMotors[0],
    //     encA=kEncoders[2], encB=kEncoders[3]
    { "UR", 9, 36, 35,
      2, 13,
      0.5f, 0.6f, 0.0f,  30.0f, 1.05f,  188.0f,   -1, -1 },
};

// =====================================================================
//  Encoder (ISR): periodos para el rpm + cuadratura con signo
// =====================================================================
struct EncoderState {
    volatile unsigned long period_buf[FILTER_SIZE];
    volatile uint8_t       period_idx;
    volatile unsigned long last_pulse_us;
    volatile bool          got_pulse;
    volatile long          ticks;      // cuadratura, positivo = A adelanta a B
    volatile uint8_t       prev_ab;
    // diagnostico (etapa 0): flancos por canal y transiciones sin sentido
    volatile uint32_t      edgesA, edgesB, bad;
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

// Un ISR por rueda (A y B con CHANGE apuntan al mismo): lee los dos canales
template <uint8_t I>
void encoderISR()
{
    EncoderState& e = enc[I];
    unsigned long now = micros();

    uint8_t ab = (digitalRead(CFG[I].encA) << 1) | digitalRead(CFG[I].encB);
    uint8_t diff = ab ^ e.prev_ab;
    int8_t  step = QUAD_STEP[(e.prev_ab << 2) | ab];
    if (diff & 2) e.edgesA = e.edgesA + 1;
    if (diff & 1) e.edgesB = e.edgesB + 1;
    if (step == 0) e.bad = e.bad + 1;   // A y B cambian juntos, o rebote sin cambio
    e.ticks   = e.ticks + step;
    e.prev_ab = ab;

    unsigned long p = now - e.last_pulse_us;
    e.last_pulse_us = now;
    if (p > 200UL) {                       // ignora rebotes < 200 us (solo para el rpm)
        e.period_buf[e.period_idx] = p;
        e.period_idx = (e.period_idx + 1) % FILTER_SIZE;
        e.got_pulse  = true;
    }
}

float measureRPM(uint8_t i)
{
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t k = 0; k < FILTER_SIZE; k++) buf[k] = enc[i].period_buf[k];
    unsigned long last    = enc[i].last_pulse_us;
    bool          has_got = enc[i].got_pulse;
    interrupts();

    if (!has_got) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    unsigned long sum   = 0;
    uint8_t       count = 0;
    for (uint8_t k = 0; k < FILTER_SIZE; k++) {
        if (buf[k] > 0) { sum += buf[k]; count++; }
    }
    if (count == 0) return 0.0f;

    float avg_period = (float)(sum / count);
    return 60000000.0f / (avg_period * 4.0f * CFG[i].ppr);
}

long readTicks(uint8_t i)
{
    noInterrupts();
    long t = enc[i].ticks;
    interrupts();
    return t;
}

// =====================================================================
//  Motor + PID por rueda
// =====================================================================
struct PidState {
    float sp_ramped  = 0.0f;
    float integral   = 0.0f;
    float last_error = 0.0f;
    float rpm_filt   = 0.0f;
    float output     = 0.0f;
    long  last_ticks = 0;
    float rpm_plot   = 0.0f;   // rpm_filt, con signo si PLOT_SIGNED_RPM
};
PidState pid[NUM_WHEELS];

int8_t dc_flip = +1;   // -1 solo en la pasada "atras" de la etapa 0

// pwm >= 0, en el sentido "adelante" de la rueda
void setMotor(uint8_t i, float pwm)
{
    const WheelCfg& c = CFG[i];
    bool fwd = (c.motorDir * dc_flip > 0);
    digitalWrite(c.in1, fwd ? HIGH : LOW);
    digitalWrite(c.in2, fwd ? LOW : HIGH);
    analogWrite(c.pwm, (int)constrain(pwm, 0.0f, 255.0f));
}

void stepWheel(uint8_t i, float target)
{
    const WheelCfg& c = CFG[i];
    PidState&       s = pid[i];

    float rpm_raw = measureRPM(i);
    s.rpm_filt   += RPM_ALPHA * (rpm_raw - s.rpm_filt);
    float rpm     = s.rpm_filt;

    long t   = readTicks(i);
    long dt  = t - s.last_ticks;
    s.last_ticks = t;
    s.rpm_plot   = (PLOT_SIGNED_RPM && (long)c.encFwdSign * dt < -2) ? -rpm : rpm;

    if (!WHEEL_ENABLED[i]) target = 0.0f;

    if (s.sp_ramped < target)      s.sp_ramped = min(target, s.sp_ramped + SP_RAMP * Ts);
    else if (s.sp_ramped > target) s.sp_ramped = max(target, s.sp_ramped - SP_RAMP * Ts);

    // Rueda apagada: motor en 0 y PID limpio para el proximo arranque
    if (target <= 0.0f && s.sp_ramped <= 0.0f) {
        s.integral = 0.0f;
        s.last_error = 0.0f;
        s.output = 0.0f;
        setMotor(i, 0.0f);
        return;
    }

    float error      = s.sp_ramped - rpm;
    float derivative = (error - s.last_error) / Ts;
    float ff         = (s.sp_ramped > 0.0f) ? c.ffOffset + c.ffSlope * s.sp_ramped : 0.0f;
    float output_unclamped = ff + c.kp * error + c.ki * s.integral + c.kd * derivative;

    // Solo integra si el output no esta saturado (anti-windup real)
    if (output_unclamped > 0.0f && output_unclamped < 255.0f) {
        s.integral += error * Ts;
        // Limite = rango completo de PWM (255) / Ki
        float lim  = (c.ki > 0.0f) ? 255.0f / c.ki : 0.0f;
        s.integral = constrain(s.integral, -lim, lim);
    }

    s.output     = constrain(output_unclamped, 0.0f, 255.0f);
    s.last_error = error;
    setMotor(i, s.output);
}

// =====================================================================
//  Etapa 0: comprobacion de sentido, una rueda a la vez
// =====================================================================
constexpr uint32_t DC_SETTLE_MS  = 1500;  // rampa + asentar
constexpr uint32_t DC_MEASURE_MS = 2000;  // ventana donde se cuentan ticks
constexpr uint32_t DC_PAUSE_MS   = 1500;  // parado antes de la siguiente
constexpr uint32_t DC_SLOT_MS    = DC_SETTLE_MS + DC_MEASURE_MS + DC_PAUSE_MS;

uint32_t dc_start = 0;
uint32_t dc_slot_prev = 0xFFFFFFFF;
bool     dc_measuring = false;
bool     dc_reported  = false;
long     dc_ticks0    = 0;
uint32_t dc_t0        = 0;

void directionCheckTick()
{
    uint32_t t    = millis() - dc_start;
    uint32_t slot = t / DC_SLOT_MS;          // 2 slots por rueda: adelante, atras
    uint8_t  w    = (slot / 2) % NUM_WHEELS;
    bool     rev  = (slot & 1);
    uint32_t in   = t % DC_SLOT_MS;

    if (slot != dc_slot_prev) {
        dc_slot_prev = slot;
        dc_measuring = false;
        dc_reported  = false;
        dc_flip      = rev ? -1 : +1;        // todas las ruedas estan en 0 en este instante
        Serial.print("--- "); Serial.print(CFG[w].name);
        Serial.print(rev ? " ATRAS" : " ADELANTE");
        Serial.print(WHEEL_ENABLED[w] ? ": girando a " : ": DESHABILITADA (");
        if (WHEEL_ENABLED[w]) { Serial.print(DIR_CHECK_RPM, 0); Serial.println(" rpm. Mira hacia donde se mueve la PARTE SUPERIOR de la rueda."); }
        else Serial.println(")");
    }

    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        bool active = (i == w) && in < (DC_SETTLE_MS + DC_MEASURE_MS);
        stepWheel(i, active ? DIR_CHECK_RPM : 0.0f);
    }

    if (!dc_measuring && !dc_reported && in >= DC_SETTLE_MS) {
        dc_measuring = true;
        noInterrupts();
        enc[w].edgesA = 0; enc[w].edgesB = 0; enc[w].bad = 0;
        interrupts();
        dc_ticks0 = readTicks(w);
        dc_t0 = millis();
    }
    if (dc_measuring && !dc_reported && in >= DC_SETTLE_MS + DC_MEASURE_MS) {
        dc_measuring = false;
        dc_reported  = true;
        long  d   = readTicks(w) - dc_ticks0;
        float sec = (millis() - dc_t0) / 1000.0f;
        float rpm_cnt = fabsf((float)d) / (4.0f * CFG[w].ppr) / sec * 60.0f;

        Serial.print("    "); Serial.print(CFG[w].name);
        Serial.print(": dTicks="); Serial.print(d);
        Serial.print(" en "); Serial.print(sec, 2); Serial.print(" s");
        Serial.print(" | rpm(periodo)="); Serial.print(pid[w].rpm_filt, 1);
        Serial.print(" | rpm(conteo)="); Serial.print(rpm_cnt, 1);
        Serial.print(" | out="); Serial.print(pid[w].output, 0);
        Serial.print(" | cuenta "); Serial.print(d >= 0 ? "POSITIVO" : "NEGATIVO");
        // esperado: adelante = signo encFwdSign, atras = el contrario
        int expected = CFG[w].encFwdSign * (rev ? -1 : +1);
        bool ok = (d != 0) && ((d > 0) == (expected > 0));
        Serial.print(" (esperado "); Serial.print(expected > 0 ? "POSITIVO" : "NEGATIVO");
        Serial.print(") -> "); Serial.println(ok ? "OK" : "REVISAR");

        noInterrupts();
        uint32_t ea = enc[w].edgesA, eb = enc[w].edgesB, bd = enc[w].bad;
        interrupts();
        Serial.print("      flancos A="); Serial.print(ea);
        Serial.print(" B="); Serial.print(eb);
        Serial.print(" invalidos="); Serial.print(bd);
        Serial.println("  (sano: A~B y invalidos~0)");
    }
}

// =====================================================================
//  Setup / loop
// =====================================================================
unsigned long last_time = 0;

void setup()
{
    Serial.begin(115200);

    analogWriteResolution(8);

    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        const WheelCfg& c = CFG[i];
        pinMode(c.encA, INPUT_PULLUP);
        pinMode(c.encB, INPUT_PULLUP);
        pinMode(c.in1, OUTPUT);
        pinMode(c.in2, OUTPUT);
        pinMode(c.pwm, OUTPUT);
        analogWriteFrequency(c.pwm, PWM_FREQ_HZ);
        analogWrite(c.pwm, 0);

        enc[i].prev_ab = (digitalRead(c.encA) << 1) | digitalRead(c.encB);
    }

    attachInterrupt(digitalPinToInterrupt(CFG[LL].encA), encoderISR<LL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[LL].encB), encoderISR<LL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[LR].encA), encoderISR<LR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[LR].encB), encoderISR<LR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[UL].encA), encoderISR<UL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[UL].encB), encoderISR<UL>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[UR].encA), encoderISR<UR>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CFG[UR].encB), encoderISR<UR>, CHANGE);

    // Pines efectivos, para compararlos con los sketches de calibracion
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        const WheelCfg& c = CFG[i];
        Serial.print(c.name);
        Serial.print(": pwm="); Serial.print(c.pwm);
        Serial.print(" in1=");  Serial.print(c.in1);
        Serial.print(" in2=");  Serial.print(c.in2);
        Serial.print(" encA="); Serial.print(c.encA);
        Serial.print(" encB="); Serial.println(c.encB);
    }

    dc_start  = millis();
    last_time = millis();
}

void loop()
{
    unsigned long now = millis();
    if (now - last_time < (unsigned long)(Ts * 1000.0f)) return;
    last_time += (unsigned long)(Ts * 1000.0f);

    if (DIRECTION_CHECK) {
        directionCheckTick();
        return;
    }

    for (uint8_t i = 0; i < NUM_WHEELS; i++) stepWheel(i, setpoint);

    Serial.print("rpm_LL:");  Serial.print(pid[LL].rpm_plot, 2);
    Serial.print(",rpm_LR:"); Serial.print(pid[LR].rpm_plot, 2);
    Serial.print(",rpm_UL:"); Serial.print(pid[UL].rpm_plot, 2);
    Serial.print(",rpm_UR:"); Serial.print(pid[UR].rpm_plot, 2);
    Serial.print(",Setpoint:"); Serial.print(setpoint);
    if (PRINT_OUTPUTS) {
        Serial.print(",out_LL:"); Serial.print(pid[LL].output, 1);
        Serial.print(",out_LR:"); Serial.print(pid[LR].output, 1);
        Serial.print(",out_UL:"); Serial.print(pid[UL].output, 1);
        Serial.print(",out_UR:"); Serial.print(pid[UR].output, 1);
    }
    Serial.println();
}
