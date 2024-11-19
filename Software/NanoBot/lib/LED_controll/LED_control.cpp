#include "LED_control.h"

LEDControl::LEDControl(uint8_t ledPin) : _ledPin(ledPin), _lastToggleTime(0), _ledState(false)
{
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);
}

void LEDControl::BlinkLED(uint16_t interval)
{
    uint32_t currentTime = millis();
    if (currentTime - _lastToggleTime >= interval)
    {
        _lastToggleTime = currentTime;
        _ledState = !_ledState;
        digitalWrite(_ledPin, _ledState ? HIGH : LOW);
    }
}

void LEDControl::TurnOnLED()
{
    _ledState = true;
    digitalWrite(_ledPin, HIGH);
}

void LEDControl::TurnOffLED()
{
    _ledState = false;
    digitalWrite(_ledPin, LOW);
}