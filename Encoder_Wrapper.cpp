#include <Arduino.h>
#include "Encoder_Wrapper.h"

unsigned int Encoder_Wrapper::_instanceNum = 0;

size_t Encoder_Wrapper::_sensorNum = -1;

Encoder **Encoder_Wrapper::_encodersPtr;

unsigned int *Encoder_Wrapper::_pins;

Encoder_Wrapper::Encoder_Wrapper(unsigned int* pins, size_t sensorNum)
{
    _sensorNum = sensorNum;
    _instanceNum++;

    _encodersPtr = new Encoder *[_sensorNum];
    _pins = new unsigned int[_sensorNum * _pinsPerSensor];
    _resetCounts = new long int[_sensorNum];
    _setCounts = new long int[_sensorNum];
    for (size_t i = 0; i < _sensorNum; i++)
    {
        _pins[i * _pinsPerSensor] = pins[i * _pinsPerSensor];
        _pins[i * _pinsPerSensor + 1] = pins[i * _pinsPerSensor + 1];
        _encodersPtr[i] = new Encoder(_pins[i * _pinsPerSensor],
                                      _pins[i * _pinsPerSensor + 1]);
        _resetCounts[i] = 0;
        _setCounts[i] = 0;
    }
}

Encoder_Wrapper::~Encoder_Wrapper()
{
    if(_instanceNum == 1)
    {
        _instanceNum--;

        for (size_t i = 0; i < _sensorNum; i++)
        {
            delete _encodersPtr[i];
        }
        delete[] _encodersPtr;
        delete[] _pins;
        delete[] _resetCounts;
        delete[] _setCounts;
    }
    else
    {
        _instanceNum--;
    }
}

void Encoder_Wrapper::setCount(long int newCount, size_t sensor /*= 0*/)
{
    _setCounts[sensor] = newCount;
    _resetCounts[sensor] = _encodersPtr[sensor]->read();
}

void Encoder_Wrapper::resetCount(size_t sensor /*= -1*/)
{
    if (sensor == 1000)
    {
        for (size_t i = 0; i < _sensorNum; i++)
        {
            setCount(0, i);
        }
    }
    else
    {
        setCount(0, sensor);
    }
}

long int Encoder_Wrapper::getCount(size_t sensor /*= 0*/)
{
    return _encodersPtr[sensor]->read() - _resetCounts[sensor] + _setCounts[sensor];
}

unsigned int Encoder_Wrapper::getPin(size_t sensor /*= 0*/, size_t index /*= 0*/)
{
    return _pins[sensor * _pinsPerSensor + index];
}