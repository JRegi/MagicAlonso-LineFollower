#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <Arduino.h>

class LEDControl
{
private:
    uint8_t _ledPin;
    uint32_t _lastToggleTime;
    bool _ledState;

public:
    LEDControl(uint8_t ledPin);
    void BlinkLED(uint16_t interval);
    void TurnOnLED();
    void TurnOffLED();
};

#endif
