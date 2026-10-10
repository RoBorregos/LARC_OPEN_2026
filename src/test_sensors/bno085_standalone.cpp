/**
 * @file bno085_standalone.cpp
 * @brief Diagnostico puro del BNO085, sin RTOS: aisla si el problema es del
 *        scheduler/mutex o del sensor/handshake SH-2 en si. Misma logica de
 *        deteccion e inicializacion que test_RTOS/rtos_bno085_only.cpp, pero
 *        en un loop() normal de Arduino.
 *
 * pio run -e bno085_standalone -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO08x.h>

// ============================================================
// Configuración del BNO085
// ============================================================

constexpr uint8_t BNO_ADDRESS_PRIMARY   = 0x4A; // ADR a GND
constexpr uint8_t BNO_ADDRESS_SECONDARY = 0x4B; // ADR a VCC

// Teensy 4.1 (Wire2):
// SDA2 -> pin 25
// SCL2 -> pin 24
constexpr uint8_t SDA_PIN = 25;
constexpr uint8_t SCL_PIN = 24;

constexpr uint32_t I2C_FREQUENCY    = 50000; // 100kHz fallaba "I2C address not found" en begin_I2C; volviendo a 50kHz (heredado del BNO055)
constexpr uint32_t SAMPLE_PERIOD_MS = 500;
constexpr uint8_t MAX_ATTEMPTS      = 20;

// El chip a veces deja de mandar reportes (getSensorEvent() siempre false)
// sin que el bus se caiga -- el ultimo valor decodificado se queda pegado
// para siempre porque nada vuelve a pisarlo. Si pasa este tiempo sin una
// lectura valida, se asume que el SH-2 quedo colgado y se fuerza un
// reset+reinit completo.
constexpr uint32_t STALL_TIMEOUT_MS = 1500;

constexpr int8_t BNO_RST_PIN = 2; // RST del BNO085 cableado al pin 2 de la Teensy

// OJO: NO se le pasa BNO_RST_PIN al constructor a proposito. Si se le pasa,
// _init() (dentro de begin_I2C) dispara SU PROPIO hardwareReset() justo
// antes de escribir el primer paquete SH-2, con solo 30ms de margen fijo
// (ver hal_hardwareReset() en Adafruit_BNO08x.cpp) -- no alcanza a que el
// firmware SH-2 termine de arrancar en esta placa. Manejamos el reset
// nosotros mismos (manualResetWithReadback(), mas abajo) con un margen
// mayor y controlado, y dejamos que la libreria NUNCA vuelva a tocar el
// pin de reset.
static Adafruit_BNO08x bno;

static uint8_t bnoAddress = BNO_ADDRESS_PRIMARY;

// ============================================================
// Utilidades
// ============================================================

static void quaternionToEuler(float qw, float qx, float qy, float qz,
                               float &yaw, float &pitch, float &roll)
{
    yaw = atan2f(2.0f * (qw * qz + qx * qy),
                 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / PI);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp       = fmaxf(-1.0f, fminf(1.0f, sinp));
    pitch      = asinf(sinp) * (180.0f / PI);

    roll = atan2f(2.0f * (qw * qx + qy * qz),
                  1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / PI);
}

// ============================================================
// Diagnostico crudo de bus (sin pasar por Adafruit_BNO08x)
// ============================================================
// Todo esto usa Wire directamente, para separar "el bus I2C en si no
// funciona" de "el bus funciona pero el chip/protocolo SH-2 falla" --
// que es exactamente la duda cuando algo funciono en protoboard y no en
// la placa real (cambia el cableado/conector, no el codigo).

static void printAddressHex(uint8_t address)
{
    Serial.print("0x");
    if (address < 0x10)
        Serial.print("0");
    Serial.print(address, HEX);
}

// Antes de tocar Wire.begin(): si SDA o SCL ya estan en LOW en reposo,
// el bus esta trabado (otro dispositivo reteniendo la linea, falta de
// pull-up, o un corto), y ningun escaneo posterior va a servir de nada
// hasta resolver eso primero.
static void checkBusIdleLines()
{
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(10);

    bool sdaHigh = digitalRead(SDA_PIN);
    bool sclHigh = digitalRead(SCL_PIN);

    Serial.print("[BUS] Lineas en reposo -- SDA: ");
    Serial.print(sdaHigh ? "HIGH (ok)" : "LOW (!!)");
    Serial.print("  SCL: ");
    Serial.println(sclHigh ? "HIGH (ok)" : "LOW (!!)");

    if (!sdaHigh || !sclHigh)
    {
        Serial.println("[BUS] ADVERTENCIA: linea atascada en LOW antes de iniciar I2C -- revisa pull-ups/cortos/otro dispositivo en el bus.");
    }
}

static const char* i2cErrorName(uint8_t error)
{
    switch (error)
    {
        case 0: return "OK (ACK)";
        case 1: return "buffer de escritura lleno";
        case 2: return "NACK en direccion -- nadie contesto a esa direccion";
        case 3: return "NACK en dato";
        case 4: return "error desconocido del bus";
        case 5: return "timeout (SDA/SCL posiblemente atascado)";
        default: return "codigo no documentado";
    }
}

// Probe crudo (beginTransmission/endTransmission) a UNA direccion puntual,
// sin pasar por Adafruit_I2CDevice ni SH-2. Si esto falla, el problema es
// de bus/cableado/alimentacion, no de la libreria ni del protocolo.
static void rawProbe(uint8_t address)
{
    Wire2.beginTransmission(address);
    uint8_t error = Wire2.endTransmission();

    Serial.print("[BUS] Probe crudo ");
    printAddressHex(address);
    Serial.print(": ");
    Serial.println(i2cErrorName(error));
}

// Escaneo completo del bus (0x01-0x7F): si aparece CUALQUIER otro
// dispositivo pero no 0x4A/0x4B, el bus en si esta sano y el problema es
// especifico del BNO085 (alimentacion propia, ADR, o el conector/cable
// que le toca a el). Si no aparece nada, el problema es del bus completo
// (alimentacion general, SDA/SCL, o conector compartido).
static void rawBusScan()
{
    Serial.println("[BUS] Escaneo completo (0x01-0x7F):");
    uint8_t found = 0;

    for (uint8_t address = 0x01; address <= 0x7F; address++)
    {
        Wire2.beginTransmission(address);
        if (Wire2.endTransmission() == 0)
        {
            found++;
            Serial.print("  ");
            printAddressHex(address);
            Serial.println("  ACK");
        }
    }

    if (found == 0)
    {
        Serial.println("  (nada respondio en todo el bus)");
    }
}

// Reset manual con lectura de vuelta del pin tras cada escritura: si el
// pin no llega al nivel esperado, hay un corto/mal contacto en la propia
// linea de RST entre la Teensy y el conector (no detecta un cable
// desconectado en el otro extremo, solo un corto en el tramo del pin).
static void manualResetWithReadback()
{
    pinMode(BNO_RST_PIN, OUTPUT);

    digitalWrite(BNO_RST_PIN, HIGH);
    delay(10);
    Serial.print("[RST] HIGH -> lectura: ");
    Serial.println(digitalRead(BNO_RST_PIN) ? "HIGH (ok)" : "LOW (!! revisa corto a GND)");

    digitalWrite(BNO_RST_PIN, LOW);
    delay(10);
    Serial.print("[RST] LOW  -> lectura: ");
    Serial.println(digitalRead(BNO_RST_PIN) ? "HIGH (!! revisa corto a VCC)" : "LOW (ok)");

    digitalWrite(BNO_RST_PIN, HIGH);
    delay(10);
    Serial.print("[RST] HIGH -> lectura: ");
    Serial.println(digitalRead(BNO_RST_PIN) ? "HIGH (ok)" : "LOW (!!)");
}

// NOTA: se sospecho que reinicializar Wire en cada intento de begin_I2C()
// (ver Adafruit_I2CDevice::begin() -> _wire->begin()) era lo que rompia el
// ACK -- descartado con testWireReinitGlitch() (ver mas abajo): repetir
// Wire.begin() + probe nunca falla. El probe crudo (0 bytes) sigue
// funcionando siempre; igual dejamos esta espera con probe crudo porque no
// hace daño y evita reintentar begin_I2C() a lo pavote antes de que la
// direccion siquiera aparezca en el bus.
static bool waitForAddressRaw(uint8_t address, uint8_t maxAttempts, uint32_t retryDelayMs)
{
    for (uint8_t attempt = 1; attempt <= maxAttempts; ++attempt)
    {
        Wire2.beginTransmission(address);
        if (Wire2.endTransmission() == 0)
        {
            return true;
        }
        delay(retryDelayMs);
    }
    return false;
}

// begin_I2C() dispara SU PROPIO reset de hardware internamente (ver _init()
// en Adafruit_BNO08x.cpp: se llama a hardwareReset() ahi, sin importar que
// ya hayamos reseteado a mano antes) justo antes de abrir el protocolo
// SH-2. Entonces aunque el chip ya conteste al probe crudo (ACK a nivel de
// silicio), esa MISMA llamada puede fallar si el firmware SH-2 todavia no
// termino de arrancar tras ese reset interno -- el ACK de bus no implica
// que el firmware ya este listo para el handshake SH-2. Por eso reintenta
// unas pocas veces con una espera larga entre intentos: cada intento ya
// incluye un ciclo completo de reset+arranque+handshake (algunos cientos
// de ms), no tiene sentido reintentar cada 100ms como con el probe crudo.
constexpr uint8_t BEGIN_I2C_ATTEMPTS        = 5;
constexpr uint32_t BEGIN_I2C_RETRY_DELAY_MS = 300;

// Prueba aislada de la hipotesis "Wire.begin() reinicializa el periferico
// y ESO es lo que rompe el ACK": llama a Wire.begin() otra vez (como hace
// Adafruit_I2CDevice::begin() adentro de begin_I2C) y sondea la direccion
// justo despues, N veces seguidas. Si esto falla de forma consistente
// mientras que rawProbe() suelto (sin reinit) nunca falla, es una prueba
// directa y reproducible de que el problema esta en ese reinit especifico
// del Wire de la Teensy con esta placa/harness -- no en la libreria BNO,
// no en el chip, y no en el cableado (que ya sabemos que responde bien).
static void testWireReinitGlitch(uint8_t address, uint8_t rounds)
{
    Serial.println("[DIAG] Repitiendo Wire.begin() (lo que hace begin_I2C por dentro) + probe inmediato:");

    for (uint8_t i = 1; i <= rounds; ++i)
    {
        Wire2.begin();
        Wire2.setClock(I2C_FREQUENCY);

        Serial.print("  [");
        Serial.print(i);
        Serial.print("/");
        Serial.print(rounds);
        Serial.print("] ");
        rawProbe(address);

        delay(50);
    }
}

// Replica el write EXACTO que begin_I2C hace por dentro al abrir el
// protocolo SH-2 (ver i2chal_open() en Adafruit_BNO08x.cpp): un paquete
// SHTP de 5 bytes de "software reset", no un probe de 0 bytes como
// rawProbe(). testWireReinitGlitch() ya descarto que el problema sea
// reinicializar Wire; esta prueba aisla si el problema es, especificamente,
// escribir datos reales (no solo el ACK de direccion) -- que apunta a
// integridad de señal (pull-ups/capacitancia de la placa) mas que a un bug
// de software, ya que el ACK de direccion sale perfecto siempre.
static void testRawDataWrite(uint8_t address)
{
    const uint8_t softresetPkt[] = {5, 0, 1, 0, 1};

    Serial.println("[DIAG] Escritura real de 5 bytes (paquete SH-2 softreset), sin la libreria Adafruit:");

    for (uint8_t i = 1; i <= 5; ++i)
    {
        Wire2.beginTransmission(address);
        Wire2.write(softresetPkt, sizeof(softresetPkt));
        uint8_t error = Wire2.endTransmission();

        Serial.print("  [");
        Serial.print(i);
        Serial.print("/5] escritura 5 bytes -> ");
        Serial.println(i2cErrorName(error));

        delay(30);
    }
}

static bool tryBeginAt(uint8_t address)
{
    if (!waitForAddressRaw(address, MAX_ATTEMPTS, 100))
    {
        return false;
    }

    for (uint8_t attempt = 1; attempt <= BEGIN_I2C_ATTEMPTS; ++attempt)
    {
        if (bno.begin_I2C(address, &Wire2))
        {
            bnoAddress = address;
            return true;
        }

        Serial.print("[BNO085] begin_I2C intento ");
        Serial.print(attempt);
        Serial.println(" fallo (ACK de bus OK, pero handshake SH-2 no respondio a tiempo). Reintentando...");

        delay(BEGIN_I2C_RETRY_DELAY_MS);
    }

    return false;
}

static bool initBNO085()
{
    Serial.println("[BNO085] Buscando en 0x4A...");

    if (!tryBeginAt(BNO_ADDRESS_PRIMARY))
    {
        Serial.println("[BNO085] No respondió en 0x4A, probando 0x4B...");

        if (!tryBeginAt(BNO_ADDRESS_SECONDARY))
        {
            Serial.println("[BNO085] init FALLO: chip no detectado en 0x4A ni 0x4B.");
            return false;
        }
    }

    Serial.print("[BNO085] begin_I2C OK en dirección 0x");
    Serial.println(bnoAddress, HEX);

    if (!bno.enableReport(SH2_ROTATION_VECTOR, 20000))
    {
        Serial.println("[BNO085] init FALLO: no se pudo activar SH2_ROTATION_VECTOR.");
        return false;
    }

    Serial.println("[BNO085] init OK, reporte de rotación activo.");
    return true;
}

static uint32_t g_noEventCount = 0;   // getSensorEvent() devolvió false (sin dato / error de comm)
static uint32_t g_wrongTypeCount = 0; // hubo evento pero no era SH2_ROTATION_VECTOR
static uint8_t g_lastSensorId = 0xFF;
static uint32_t g_stallRecoveryCount = 0;

// Reset completo (hardware + reapertura SH-2 + reactivar reporte) cuando el
// chip deja de mandar reportes. Reusa el mismo camino que setup(): reset
// manual con margen de arranque, y de ahi initBNO085() de nuevo (que
// internamente reintenta 0x4A/0x4B y vuelve a activar SH2_ROTATION_VECTOR).
static bool recoverBNO085()
{
    g_stallRecoveryCount++;

    Serial.print("[BNO085] Sin lecturas validas por >");
    Serial.print(STALL_TIMEOUT_MS);
    Serial.print("ms -- forzando reset+reinit (recovery #");
    Serial.print(g_stallRecoveryCount);
    Serial.println(").");

    manualResetWithReadback();
    delay(500); // mismo margen de arranque del firmware SH-2 que en setup()

    return initBNO085();
}

static bool readYawPitchRoll(float& yaw, float& pitch, float& roll, uint8_t& accuracy)
{
    sh2_SensorValue_t value;

    if (!bno.getSensorEvent(&value))
    {
        g_noEventCount++;
        return false;
    }

    g_lastSensorId = value.sensorId;

    if (value.sensorId != SH2_ROTATION_VECTOR)
    {
        g_wrongTypeCount++;
        return false;
    }

    const float qw = value.un.rotationVector.real;
    const float qx = value.un.rotationVector.i;
    const float qy = value.un.rotationVector.j;
    const float qz = value.un.rotationVector.k;

    quaternionToEuler(qw, qx, qy, qz, yaw, pitch, roll);

    if (yaw < 0.0f)
        yaw += 360.0f;

    accuracy = value.status & 0x03;

    return true;
}

// ============================================================
// Arduino
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   DIAGNOSTICO BNO085 (sin RTOS)");
    Serial.println("================================");

    // Teensy 4.1 (Wire2):
    // SDA2 = pin 25
    // SCL2 = pin 24
    checkBusIdleLines();

    Wire2.begin();
    Wire2.setClock(I2C_FREQUENCY);
    Serial.println("I2C initialized");

    Serial.println();
    Serial.println("--- Diagnostico ANTES del reset ---");
    rawBusScan();
    rawProbe(BNO_ADDRESS_PRIMARY);
    rawProbe(BNO_ADDRESS_SECONDARY);

    // Reset manual, unico (la libreria ya no recibe el pin de reset -- ver
    // nota junto a "static Adafruit_BNO08x bno;" mas arriba), con un margen
    // de arranque generoso: los intentos anteriores fallaban consistentemente
    // en el handshake SH-2 (no en el ACK de bus ni en escrituras crudas de
    // datos) apenas 30ms despues del reset interno de la libreria -- todo
    // apunta a que el firmware SH-2 de este chip, en esta placa, necesita
    // mas de esos 30ms fijos para terminar de arrancar antes de poder
    // procesar el primer paquete SH-2.
    Serial.println();
    Serial.print("[BNO085] Forzando reset manual (unico) en pin ");
    Serial.println(BNO_RST_PIN);
    manualResetWithReadback();
    delay(500); // margen de arranque del firmware SH-2 tras salir de reset

    Serial.println();
    Serial.println("--- Diagnostico DESPUES del reset ---");
    rawBusScan();
    rawProbe(BNO_ADDRESS_PRIMARY);
    rawProbe(BNO_ADDRESS_SECONDARY);
    Serial.println();

    testWireReinitGlitch(BNO_ADDRESS_PRIMARY, 5);
    Serial.println();
    testRawDataWrite(BNO_ADDRESS_PRIMARY);
    Serial.println();

    if (!initBNO085())
    {
        Serial.println("[BNO085] No se pudo inicializar. Quedando en modo escaneo en vivo");
        Serial.println("(mueve/presiona el conector y observa si 0x4A/0x4B aparecen).");

        while (true)
        {
            delay(1000);
            Serial.print("[t=");
            Serial.print(millis());
            Serial.print("ms] ");
            rawProbe(BNO_ADDRESS_PRIMARY);
        }
    }

    Serial.println();
    Serial.println("Comenzando lecturas...");
}

void loop()
{
    static uint32_t previousPrintTime = 0;
    static uint32_t validReadings = 0;
    static uint32_t failedReadings = 0;
    static uint32_t lastOkMillis = millis();

    static float lastYaw = 0.0f, lastPitch = 0.0f, lastRoll = 0.0f;
    static uint8_t lastAccuracy = 0;
    static bool haveReading = false;

    // getSensorEvent() hay que llamarlo en cada vuelta del loop, no solo
    // cuando toca imprimir: internamente va procesando los paquetes SHTP
    // que llegan, y si se llama poco se pierden los reportes intermedios.
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    uint8_t accuracy = 0;

    if (readYawPitchRoll(yaw, pitch, roll, accuracy))
    {
        validReadings++;
        lastYaw = yaw;
        lastPitch = pitch;
        lastRoll = roll;
        lastAccuracy = accuracy;
        haveReading = true;
        lastOkMillis = millis();
    }
    else
    {
        failedReadings++;

        // El chip dejo de mandar reportes (getSensorEvent() siempre false):
        // sin esto, lastYaw/Pitch/Roll se quedan pegados en el ultimo valor
        // para siempre. Forzar reset+reinit para que el SH-2 vuelva a andar.
        if (millis() - lastOkMillis > STALL_TIMEOUT_MS)
        {
            recoverBNO085();
            lastOkMillis = millis(); // no reintentar de nuevo en el proximo loop() inmediato
        }
    }

    delay(2); // no machacar el bus I2C sin respiro entre intentos

    if (millis() - previousPrintTime < SAMPLE_PERIOD_MS)
    {
        return;
    }

    previousPrintTime = millis();

    static const char *kAccuracyName[4] = {"Unreliable", "Low", "Medium", "High"};

    Serial.print("[t=");
    Serial.print(previousPrintTime);
    Serial.print("ms] ");

    if (haveReading)
    {
        Serial.print("Yaw: ");
        Serial.print(lastYaw, 2);
        Serial.print(" Pitch: ");
        Serial.print(lastPitch, 2);
        Serial.print(" Roll: ");
        Serial.print(lastRoll, 2);
        Serial.print(" | accuracy: ");
        Serial.print(kAccuracyName[lastAccuracy]);
    }
    else
    {
        Serial.print("Sin lectura todavía");
    }

    Serial.print(" | OK: ");
    Serial.print(validReadings);
    Serial.print(" | FAIL: ");
    Serial.print(failedReadings);
    Serial.print(" | noEvent: ");
    Serial.print(g_noEventCount);
    Serial.print(" | wrongType: ");
    Serial.print(g_wrongTypeCount);
    Serial.print(" | lastSensorId: 0x");
    Serial.print(g_lastSensorId, HEX);
    Serial.print(" | recoveries: ");
    Serial.println(g_stallRecoveryCount);
}
