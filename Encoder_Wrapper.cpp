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
        //Initialize data for upcoming procedure
        size_t oldSensorNum = _totalSensorNum;
        _totalSensorNum = _sensorNum;
        size_t newSensorNum = _totalSensorNum;

        //Save old data
        Encoder **encodersPtr = _encodersPtr;
        unsigned int *oldPins = _pins;

        //Allocate new memory
        _encodersPtr = new Encoder *[newSensorNum];
        _pins = new unsigned int[newSensorNum * _pinsPerSensor];

        //Populate new memory with old memory
        for (size_t i = 0; i < oldSensorNum; i++)
        {
            _pins[i * _pinsPerSensor] = oldPins[i * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = oldPins[i * _pinsPerSensor + 1];
            _encodersPtr[i] = encodersPtr[i];
        }

        //skipIndex at which pair of pins repeats
        size_t skipIndex = _find(pins, newSensorNum, oldPins, oldSensorNum);
        size_t pinsIndex;

        //Add new Encoder to new memory
        for (size_t i = oldSensorNum; i < newSensorNum; i++)
        {
            //Check if index to skip has been reached
            pinsIndex = i - oldSensorNum;
            if (pinsIndex == skipIndex)
            {
                pinsIndex++;
            }

            //Populate new memory with new data
            _pins[i * _pinsPerSensor] = pins[pinsIndex * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = pins[pinsIndex * _pinsPerSensor + 1];
            _encodersPtr[i] = new Encoder(_pins[i * _pinsPerSensor],
                                          _pins[i * _pinsPerSensor + 1]);
        }

        //Delete just old array not what it contained
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

size_t Encoder_Wrapper::_find(unsigned int* newPins, size_t newSensorNum, unsigned int *oldPins, size_t oldSensorNum)
{
    //found var for breaking twice
    bool found = false;

    //Initialized outside so they can be returned
    size_t newSensor = 0;
    size_t oldSensor = 0;

    //Iterate through new pin data
    for (newSensor = 0; newSensor < newSensorNum; newSensor++)
    {
        //Iterate through old pin data
        for (oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
        {
            //Compare old data to new data check for similarity
            if((oldPins[oldSensor * _pinsPerSensor] == newPins[newSensor * _pinsPerSensor]) 
            && (oldPins[oldSensor * _pinsPerSensor + 1] == newPins[newSensor * _pinsPerSensor + 1]))
            {
                //Indicate double break
                found = true;
                break;
            }
        }

        //Check for double break flag
        if (found)
        {
            //Break out of parent loop
            break;
        }
    }

    //Return index of repeated pair
    return newSensor;
}