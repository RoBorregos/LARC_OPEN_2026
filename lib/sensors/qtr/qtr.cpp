#include "qtr.hpp"
#include "constants.h"

// 74HC4067
// Wiring (per schematic):
//   SIG - A0
//   S0 - pin 26
//   S1 - pin 27
//   S2 - pin 28
//   S3 - pin 29
//   EN - GND (always enabled)

static constexpr bool LINE_IS_BLACK = false;

QTR::QTR(uint8_t firstChannel, Mux74HC4067& mux_)
    : firstCh(firstChannel), initialized(false), mux(mux_), position(0), posHistoryIdx(0), lastRawPos(0)
{
    for (uint8_t i = 0; i < N; i++)
    {
        raw[i]    = 0;
        calMin[i] = 0;
        calMax[i] = 1;
        norm[i]   = 0;
    }
    for (uint8_t i = 0; i < 3; i++)
        posHistory[i] = 0;
}

bool QTR::begin()
{
    mux.begin();

    // Select the first channel on the array (the rest will be read sequentially)
    mux.select(firstCh);

    initialized = true;
    return true;
}

void QTR::ensureCalValid()
{
    for (uint8_t i = 0; i < N; i++)
    {
        if (calMax[i] <= calMin[i])
            calMax[i] = calMin[i] + 1;
    }
}

void QTR::setCalibration(const uint16_t* minVals, const uint16_t* maxVals)
{
    for (uint8_t i = 0; i < N; i++)
    {
        calMin[i] = minVals[i];
        calMax[i] = maxVals[i];
    }
    ensureCalValid();
}

void QTR::calibrate(uint32_t durationMs)
{
    uint16_t cMin[N], cMax[N];

    // Init min to max possible, max to 0
    for (uint8_t i = 0; i < N; i++) { cMin[i] = 65535; cMax[i] = 0; }

    uint32_t t0 = millis();
    while (millis() - t0 < durationMs) {
        update();
        // Track min/max raw values seen per sensor
        for (uint8_t i = 0; i < N; i++) {
            if (raw[i] < cMin[i]) cMin[i] = raw[i];
            if (raw[i] > cMax[i]) cMax[i] = raw[i];
        }
        delay(5);
    }

    // Apply captured range as calibration
    setCalibration(cMin, cMax);
}

void QTR::useDefaultCalibration(uint8_t profile)
{
    using namespace Constants::QTRCalibration;

    switch (profile)
    {
    case 1: // REAR
        setCalibration(Rear.min, Rear.max);
        break;
    case 0: // FRONT
    default:
        setCalibration(Front.min, Front.max);
        break;
    }
}

void QTR::update()
{
    if (!initialized)
        return;

    // 1) Read raw (ADC)
    for (uint8_t i = 0; i < N; i++)
        raw[i] = mux.read(firstCh + i);

    // 2) Normalize raw values to 0..1000 based on calibration
    for (uint8_t i = 0; i < N; i++)
    {
        const long x = (long)(raw[i] - calMin[i]) * 1000L;
        const long d = (long)(calMax[i] - calMin[i]);
        long v = (d > 0) ? (x / d) : 0;

        if (v < 0)   v = 0;
        if (v > 1000) v = 1000;
        norm[i] = (uint16_t)v;
    }

    // 3) Calculate pos as a weighted average of sensor indices, where 0 = extreme left, 7000 = extreme right.
    uint32_t sum      = 0;
    uint32_t weighted = 0;

    for (uint8_t i = 0; i < N; i++)
    {
        const uint16_t v = norm[i];
        sum      += v;
        weighted += (uint32_t)v * (uint32_t)(i * 1000);
    }

    if (sum == 0)
    {
        //If no line was detected or value is too low, it keeps the last pos.
        return;
    }

    const int rawPos = (int)(weighted / sum); // 0 to 7000
    lastRawPos = rawPos;

    // Filtro mediana-de-3: un pico de ruido en una sola lectura (EMI de
    // motores, etc.) queda descartado en vez de propagarse al control.
    posHistory[posHistoryIdx] = rawPos;
    posHistoryIdx = (posHistoryIdx + 1) % 3;

    const int a = posHistory[0], b = posHistory[1], c = posHistory[2];
    position = max(min(a, b), min(max(a, b), c)); // mediana de a,b,c
}

void QTR::resetFilter()
{
    for (uint8_t i = 0; i < 3; i++)
        posHistory[i] = lastRawPos;
    position = lastRawPos;
}

int QTR::getPosition() const
{
    return position;
}

bool QTR::onLine(uint16_t threshold) const
{
    uint16_t maxv = 0;
    for (uint8_t i = 0; i < N; i++)
        if (norm[i] > maxv)
            maxv = norm[i];

    return maxv > threshold;
}

int QTR::getBinaryPosition() const { //Hubo cambio de funcion
    uint32_t weightedSum = 0;
    uint32_t totalWeight = 0;

    for (uint8_t i = 0; i < N; i++) {
        if (norm[i] > Constants::QTRCalibration::kBinaryThreshold) {
            weightedSum += (uint32_t)norm[i] * i * 1000;
            totalWeight += norm[i];
        }
    }

    if (totalWeight == 0) return position;
    return (int)(weightedSum / totalWeight);
}


void QTR::printCalibration(const char* label) const
{
    // Print in constants.h ready format, copy paste directly
    Serial.print(label); Serial.println(":");
    Serial.print("min = {");
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(calMin[i]);
        if (i < N - 1) Serial.print(", ");
    }
    Serial.println("};");

    Serial.print("max = {");
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(calMax[i]);
        if (i < N - 1) Serial.print(", ");
    }
    Serial.println("};");
}

void QTR::debugPrint() const
{
    Serial.print(F("RAW : "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(raw[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("NORM: "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(norm[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("POS : "));
    Serial.println(position);
}