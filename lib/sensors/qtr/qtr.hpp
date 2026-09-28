/**
 * @file qtr.hpp
 * @date 2026-01-28
 *
 * @brief QTR-8A (8-channel reflectance array) read through a shared 74HC4067 analog mux.
 *
 * ## Signal path: sensor -> mux -> ADC
 * Each QTR-8A phototransistor outputs an analog voltage proportional to
 * reflected IR light (more reflective surface = higher voltage). Instead of
 * wiring every sensor to its own analog pin, all sensor outputs feed the 16
 * input channels (C0..C15) of a shared 74HC4067 analog mux (see mux.h /
 * Mux74HC4067). Only the mux's single SIG pin is wired to a Teensy analog
 * input; the S0..S3 select lines pick which channel is currently connected
 * to SIG.
 *
 * QTR itself owns no pins and no mux -- it only stores a `firstChannel`
 * offset plus a reference to the shared Mux74HC4067 instance. Reading
 * sensor i means: mux.select(firstChannel + i) -> settle -> analogRead(SIG)
 * (done inside Mux74HC4067::read()). This lets two independent 7-sensor
 * arrays time-share one mux and one ADC pin:
 *  - QTR Frontal front connected form C0 to C7 (firstChannel = 0)
 *  - QTR Back connected from C8 to C15 (firstChannel = 8)
 * State machine can use them independently
 *
 * ## Read -> position pipeline (see QTR::update())
 *  1. Raw:       mux.read(firstChannel + i) for each of the N=7 sensors,
 *                giving raw[i] as a 0..1023 ADC count.
 *  2. Normalize: raw[i] is rescaled to 0..1000 using per-sensor
 *                calMin/calMax (from setCalibration(), from a
 *                useDefaultCalibration() profile in constants.h, or from a
 *                runtime calibrate() sweep) -> norm[i].
 *  3. Position:  weighted average of sensor index using norm[i] as weight,
 *                from 0 (line under sensor 0, extreme left) to 7000 (line
 *                under sensor 6, extreme right). If every sensor reads 0
 *                (no line seen), the previous position is kept.
 *
 * getBinaryPosition() is an alternate estimator that only counts sensors
 * above Constants::QTRCalibration::kBinaryThreshold, which helps reject
 * noise from faint reflections when a firm on/off read is preferred.
 */

#ifndef QTR_HPP
#define QTR_HPP

#include <Arduino.h>
#include <stdint.h>
#include "mux.h"

class QTR
{
public:

    static constexpr uint8_t N = 7;

    // firstChannel as it states is where the QTR starts in the mux.
    // QTR qtrFront(0, mux); (C0..C7)
    // QTR qtrRear(8, mux);  (C8..C15)
    explicit QTR(uint8_t firstChannel, Mux74HC4067& mux);

    // Initializes the pin arrangement on the mux
    // Called once on setup()
    bool begin();

    // Loads preset calibration for the 8 sensors,
    void setCalibration(const uint16_t* minVals, const uint16_t* maxVals);

    // Calls setcalibration and loads the profile for each qtr from constants.h
    void useDefaultCalibration(uint8_t profile = 0);

    // Reads the sensor and updates the cache
    void update();

    // Fuerza al filtro de mediana-de-3 (ver update()) a "saltar" a la
    // ultima posicion cruda leida en vez de arrastrar 2 lecturas viejas
    // de un contexto anterior (ej. otro estado de la maquina que uso el
    // sensor para seguir una parte distinta de la linea). Llamar justo
    // al entrar a un estado/maniobra donde la posicion previa ya no es
    // representativa.
    void resetFilter();

    // Pos on the array from 0 (extreme left) to 7000 (extreme right)
    int getPosition() const;

    // True if any sensor is over the threshold.
    // From 0 to 1000 after normalization
    bool onLine(uint16_t threshold = 200) const;

    // Function that performs calibration routine for 10 seconds
    void calibrate(uint32_t durationMs = 10000);

    //Prints the calibration values for each sensor
    // Useful for copying to constants.h.
    void printCalibration(const char* label) const;

    // Returns the position reading the qtr sensors as binary values
    // Where each bit represents whether the corresponding sensor is over the threshold or not
    // Uses already normalized values
    int getBinaryPosition() const;

    // Debug
    const uint16_t* getRaw()  const { return raw;  }

    const uint16_t* getNorm() const { return norm; }

    void debugPrint() const;

private:

    uint8_t         firstCh;
    bool            initialized;

    Mux74HC4067&    mux;

    // Raw values form the ADC
    uint16_t raw[N];
    // Minimum values per sensor
    uint16_t calMin[N];
    // Maximum values per sensor
    uint16_t calMax[N];
    // Normalized values
    uint16_t norm[N];

    // Final result for control.
    int position;

    // Historial para el filtro de mediana-de-3 sobre position (rechaza
    // picos aislados de una sola lectura, tipicamente ruido/EMI de los
    // motores acoplado a la linea analogica compartida del mux).
    int posHistory[3];
    uint8_t posHistoryIdx;
    int lastRawPos;

    void ensureCalValid();
};

#endif // QTR_HPP