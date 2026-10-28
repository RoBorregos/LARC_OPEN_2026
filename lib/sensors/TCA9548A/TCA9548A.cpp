#include "TCA9548A.h"

TCA9548A::TCA9548A(uint8_t address, TwoWire& wire)
    : address_(address)
    , wire_(wire)
    , currentChannel_(255)  // Ningún canal seleccionado
{}

bool TCA9548A::begin() {
    wire_.beginTransmission(address_);
    return (wire_.endTransmission() == 0);
}

bool TCA9548A::selectChannel(uint8_t channel) {
    if (channel > 7) return false;
    if (channel == currentChannel_) return true;

    wire_.beginTransmission(address_);
    wire_.write(1 << channel);
    // Only cache on success, so a failed switch is retried on the next call.
    const bool ok = (wire_.endTransmission() == 0);
    currentChannel_ = ok ? channel : 255;
    return ok;
}

void TCA9548A::disableAll() {
    wire_.beginTransmission(address_);
    wire_.write(0);
    wire_.endTransmission();
    
    currentChannel_ = 255;
}

bool TCA9548A::checkDevice(uint8_t deviceAddress) {
    wire_.beginTransmission(deviceAddress);
    return (wire_.endTransmission() == 0);
}

void TCA9548A::scanChannel(uint8_t channel, void (*callback)(uint8_t address)) {
    selectChannel(channel);
    delay(5);
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (addr == address_) continue;  // Skip TCA itself
        
        if (checkDevice(addr)) {
            if (callback) {
                callback(addr);
            }
        }
    }
}

void TCA9548A::scanAll() {
    Serial.println("Scanning all 8 channels...\n");

    for (uint8_t ch = 0; ch < 8; ch++) {
        selectChannel(ch);
        delay(5);

        Serial.print("CH");
        Serial.print(ch);
        Serial.print(": ");

        bool found = false;
        for (uint8_t addr = 1; addr < 127; addr++) {
            if (addr == address_) continue;
            if (checkDevice(addr)) {
                if (found) Serial.print(", ");
                Serial.print("0x");
                if (addr < 16) Serial.print("0");
                Serial.print(addr, HEX);
                found = true;
            }
        }

        if (!found) Serial.print("no device");
        Serial.println();
    }

    disableAll();
    Serial.println("\n--- Scan complete ---\n");
}

void TCA9548A::scanDirect() {
    Serial.println("=== Direct I2C Scan (no mux) ===");
    disableAll(); // make sure mux is out of the way
    delay(5);

    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (addr == address_) continue; // skip TCA itself
        if (checkDevice(addr)) {
            Serial.print("    0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found = true;
        }
    }

    if (!found) Serial.println("    (none)");
    Serial.println("=== Done ===");
}