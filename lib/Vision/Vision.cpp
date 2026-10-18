#include "Vision.hpp"

Vision::Vision(Stream &port)
    : _serial(port)
    , _beansSent(false)
    , _benefitsSent(false)
    , _stopSent(false)
    , _bitfield(0)
    , _boxType(0)
    , _piReady(false)
    , _beansRunning(false)
    , _benefitsRunning(false)
    , _criticalError(false)
    , _separatorError(false)
    , _benefitsError(false)
    , _lastError(0)
    , _parseState(0)
    , _parseHeader(0)
{}

void Vision::begin()
{
    resetGuards();
    _bitfield    = 0;
    _boxType     = 0;
    _parseState  = 0;
    _parseHeader = 0;
    _piReady         = false;
    _beansRunning    = false;
    _benefitsRunning = false;
    clearErrors();
}

void Vision::update()
{
    while (_serial.available()) {
        uint8_t b = _serial.read();

        // Collecting payload for a multi-byte message
        if (_parseState == 1) {
            if (_parseHeader == HEADER_BOX) {
                _boxType = b;
            }
            else if (_parseHeader >= ERROR_MIN && _parseHeader <= ERROR_MAX) {
                _lastError = _parseHeader;
                if (_parseHeader == ERR_SEPARATOR) {
                    _separatorError = true;
                } else if (_parseHeader == ERR_BENEFITS) {
                    _benefitsError   = true;
                    _benefitsRunning = false;
                } else {
                    _criticalError   = true;
                    _beansRunning    = false;
                    _benefitsRunning = false;
                }
            }
            _parseState = 0;
            continue;
        }

        // ACKs (single byte, no payload)
        if (b == ACK_READY)    { _piReady = true;          continue; }
        if (b == ACK_STARTING) {                            continue; }
        if (b == ACK_RUNNING)  { _beansRunning = true;     continue; }
        if (b == ACK_BENEFITS) { _benefitsRunning = true;  continue; }
        if (b == ACK_STOPPED)  { _beansRunning = false;
                                 _benefitsRunning = false;  continue; }

        // Box header (0xFE + 1 payload byte)
        if (b == HEADER_BOX) {
            _parseHeader = b;
            _parseState  = 1;
            continue;
        }

        // Error header (0xE0-0xEF + 1 payload byte)
        if (b >= ERROR_MIN && b <= ERROR_MAX) {
            _parseHeader = b;
            _parseState  = 1;
            continue;
        }

        // Vision bitfield (0x00-0x0F) — naked single byte, no header
        if (b <= 0x0F) {
            _bitfield = b;
            continue;
        }

        // Unknown byte — ignore
    }
}

void Vision::startBeans()
{
    if (!_beansSent) {
        _serial.write(CMD_START_BEANS);
        _beansSent = true;
    }
}

void Vision::startBenefits()
{
    if (!_benefitsSent) {
        _serial.write(CMD_START_BENEFITS);
        _benefitsSent = true;
    }
}

void Vision::stop()
{
    if (!_stopSent) {
        _serial.write(CMD_STOP);
        _stopSent = true;
    }
}

void Vision::requestStatus()
{
    _serial.write(CMD_STATUS);
}

void Vision::clearErrors()
{
    _criticalError  = false;
    _separatorError = false;
    _benefitsError  = false;
    _lastError      = 0;
}

void Vision::resetGuards()
{
    _beansSent    = false;
    _benefitsSent = false;
    _stopSent     = false;
}