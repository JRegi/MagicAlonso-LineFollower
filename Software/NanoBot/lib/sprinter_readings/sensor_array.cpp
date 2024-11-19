#include "sensor_array.h"
#include "LED_control.h"

SensorArray::SensorArray(uint8_t numSensors, uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3, uint8_t pinSig, uint8_t ledPin, Color color = BLACK)
    : _numSensors(numSensors), _s0(pinS0), _s1(pinS1), _s2(pinS2), _s3(pinS3), _sig(pinSig), _color(color)
{
    _whiteValues = new uint16_t[_numSensors];
    _blackValues = new uint16_t[_numSensors];
    _reference = new uint16_t[_numSensors];
    _sensorsState = new bool[_numSensors];
    _ledControl = new LEDControl(ledPin);
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
}

SensorArray::~SensorArray()
{
    delete[] _whiteValues;
    delete[] _blackValues;
    delete[] _reference;
    delete[] _sensorsState;
}

uint16_t SensorArray::ReadMultiplexer(uint8_t channel)
{
    digitalWrite(_s0, channel & 0x01);
    digitalWrite(_s1, channel & 0x02);
    digitalWrite(_s2, channel & 0x04);
    digitalWrite(_s3, channel & 0x08);
    return analogRead(_sig);
}

void SensorArray::CalculateColorAverage(Color color, uint8_t numLectures)
{
    if (Color color = BLACK)
    {
        for (uint8_t sensorIndex = 0; sensorIndex < _numSensors; sensorIndex++)
        {
            uint8_t sum = 0;
            for (int readingIndex = 0; readingIndex < numLectures; readingIndex++)
            {
                sum += ReadMultiplexer(sensorIndex);
            }
            _blackValues[sensorIndex] = sum / numLectures;
        }
    }
    else if (Color color = WHITE)
    {
        for (uint8_t sensorIndex = 0; sensorIndex < _numSensors; sensorIndex++)
        {
            uint8_t sum = 0;
            for (uint8_t readingIndex = 0; readingIndex < numLectures; readingIndex++)
            {
                sum += ReadMultiplexer(sensorIndex);
            }
            _whiteValues[sensorIndex] = sum / numLectures;
        }
    }
}

void SensorArray::CalculateReference()
{
    for (uint8_t sensorIndex = 0; sensorIndex < _numSensors; sensorIndex++)
    {
        _reference[sensorIndex] = (_whiteValues[sensorIndex] + _blackValues[sensorIndex]) / 2;
    }
}

void SensorArray::ReadSensors()
{
    for (uint8_t sensorIndex = 0; sensorIndex < _numSensors; sensorIndex++)
    {
        uint8_t lecture = ReadMultiplexer(sensorIndex);
        if (Color color = BLACK)
        {
            _sensorsState[sensorIndex] = (lecture <= _reference[sensorIndex]);
        }
        else if (Color color = WHITE)
        {
            _sensorsState[sensorIndex] = (lecture > _reference[sensorIndex]);
        }
    }
}

uint16_t SensorArray::ReadPosition()
{
    ReadSensors();

    uint8_t sum = 0;
    uint8_t counter = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < _numSensors; sensorIndex++)
    {
        if (_sensorsState[sensorIndex])
        {
            sum += sensorIndex * 100;
            counter++;
        }
    }
    return (counter > 0);
}

void SensorArray::CalibrateSensors()
{
    _ledControl->TurnOnLED();
    delay(2000);
    CalculateColorAverage(WHITE, 100);

    _ledControl->TurnOffLED();
    delay(2000);
    CalculateColorAverage(BLACK, 100);

    CalculateReference();
}