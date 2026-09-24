/**
 * TCA9548A - Multiplexor I2C de 8 canales.
 *
 * Resuelve el problema de tener varios dispositivos I2C con la misma
 * dirección fija (ej. sensores ToF VL53L1X en 0x29) conectados al mismo
 * bus: el mux expone 8 canales y solo el dispositivo del canal
 * seleccionado queda "visible" en el bus a la vez.
 *
 * Uso en RoBorregos: se instancia en instances.cpp (i2cMux) y lo usan
 * los sensores ToF (tofLeft, tofRight) en tof.cpp a través de
 * selectIfMux() antes de cada lectura, para elegir su canal correcto.
 *
 * scanAll()/scanChannel()/scanDirect() son utilidades de debug para
 * verificar qué direcciones I2C responden en cada canal (útil para
 * checar cableado).
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>

class TCA9548A {
public:
    /**
     * Constructor
     * @param address: Dirección I2C del TCA (default 0x70)
     * @param wire: Referencia al bus I2C (default Wire)
     */
    TCA9548A(uint8_t address = 0x70, TwoWire& wire = Wire);
    
    /**
     * Inicializar el multiplexor
     * @return true si el TCA responde
     */
    bool begin();
    
    /**
     * Seleccionar un canal (0-7)
     */
    void selectChannel(uint8_t channel);
    
    /**
     * Desactivar todos los canales
     */
    void disableAll();
    
    /**
     * Verificar si un dispositivo está presente en el canal actual
     * @param deviceAddress: Dirección I2C del dispositivo
     * @return true si el dispositivo responde
     */
    bool checkDevice(uint8_t deviceAddress);
    
    /**
     * Escanear un canal específico
     * @param channel: Canal a escanear (0-7)
     * @param callback: Función a llamar por cada dispositivo encontrado
     */
    void scanChannel(uint8_t channel, void (*callback)(uint8_t address));

    void scanAll();

    void scanDirect();

    /// Bus I2C donde vive el mux (los ToF detras de el deben usar el mismo).
    TwoWire& wire() const { return wire_; }

private:
    uint8_t address_;
    TwoWire& wire_;
    uint8_t currentChannel_;
};