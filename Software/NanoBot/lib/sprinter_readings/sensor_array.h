#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include <arduino.h>
#include "LED_control.h"

class SensorArray
{
private:
    LEDControl *_ledControl;
    void CalculateReference();
    void ReadSensors();
    uint16_t ReadMultiplexer(uint8_t channel);
    bool *_sensorsState;
    uint8_t _s0, _s1, _s2, _s3, _sig;
    uint16_t *_whiteValues;
    uint16_t *_blackValues;
    uint16_t *_reference;
    size_t _numSensors;

public:
    enum Color
    {
        BLACK,
        WHITE
    };
    SensorArray(uint8_t numSensors = 16, uint8_t pinS0, uint8_t pinS1, uint8_t pinS2,
                uint8_t pinS3, uint8_t pinSig, uint8_t ledPin, Color color = BLACK);
    ~SensorArray();
    void CalculateColorAverage(Color color, uint8_t numSamples);
    Color _color;
    void CalibrateSensors();
    uint16_t ReadPosition();
};

#endif