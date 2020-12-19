#include <Arduino.h>
#include "Encoder_Wrapper.h"

unsigned int Encoder_Wrapper::_instanceNum = 0;

size_t Encoder_Wrapper::_totalSensorNum = -1;

Encoder **Encoder_Wrapper::_encodersPtr;

unsigned int *Encoder_Wrapper::_pins;

Encoder_Wrapper::Encoder_Wrapper(unsigned int* pins, size_t sensorNum)
{
    _instanceNum++;
    _sensorNum = sensorNum;
    _resetCounts = new long int[_sensorNum];
    _setCounts = new long int[_sensorNum];
    for (size_t i = 0; i < _sensorNum; i++)
    {
        _resetCounts[i] = 0;
        _setCounts[i] = 0;
    }

    if (_instanceNum == 1)
    {
        _totalSensorNum = _sensorNum;
        _encodersPtr = new Encoder *[_totalSensorNum];
        _pins = new unsigned int[_totalSensorNum * _pinsPerSensor];
        for (size_t i = 0; i < _totalSensorNum; i++)
        {
            _pins[i * _pinsPerSensor] = pins[i * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = pins[i * _pinsPerSensor + 1];
            _encodersPtr[i] = new Encoder(_pins[i * _pinsPerSensor],
                                          _pins[i * _pinsPerSensor + 1]);
            
        }
    }
    else if (_sensorNum > _totalSensorNum) 
    {
        Encoder **encodersPtr = _encodersPtr;
        unsigned int *oldPins = _pins;
        _encodersPtr = new Encoder *[_sensorNum];
        _pins = new unsigned int[_sensorNum * _pinsPerSensor];
        for (size_t i = 0; i < _totalSensorNum; i++)
        {
            _pins[i * _pinsPerSensor] = oldPins[i * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = oldPins[i * _pinsPerSensor + 1];
            _encodersPtr[i] = encodersPtr[i];
        }
        for (size_t i = _totalSensorNum; i < _sensorNum; i++)
        {
            _pins[i * _pinsPerSensor] = pins[(i - _totalSensorNum) * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = pins[(i - _totalSensorNum) * _pinsPerSensor + 1];
            _encodersPtr[i] = new Encoder(_pins[i * _pinsPerSensor],
                                          _pins[i * _pinsPerSensor + 1]);
        }
        _totalSensorNum = _sensorNum;
        delete[] encodersPtr;
        delete[] oldPins;
    }
}

Encoder_Wrapper::~Encoder_Wrapper()
{
    if (_instanceNum == 1)
    {
        for (size_t i = 0; i < _totalSensorNum; i++)
        {
            delete _encodersPtr[i];
        }
        delete[] _encodersPtr;
        delete[] _pins;
    }
    _instanceNum--;
    delete[] _resetCounts;
    delete[] _setCounts;
}

void Encoder_Wrapper::setCount(long int newCount, size_t sensor /*= 0*/)
{
    _setCounts[sensor] = newCount;
    _resetCounts[sensor] = _encodersPtr[sensor]->read();
}

void Encoder_Wrapper::resetCount(size_t sensor /*= 0xFFFFFFFF*/)    //sensor = -1
{
    if (sensor == 0xFFFFFFFF)   //sensor == -1
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