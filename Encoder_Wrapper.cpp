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

    //Check if _totalSensorNum is initialized
    if (_instanceNum == 1)
    {
        //Initialize _totalSensorNum
        _totalSensorNum = _sensorNum;
    }

    size_t skipIndices[_sensorNum];
    size_t skipIndicesNum = 0;
    for (size_t i = 0; i < _sensorNum; i++)
    {
        _resetCounts[i] = 0;
        _setCounts[i] = 0;

        //Check if pin pair is repeated
        if (_find(pins, i, _pins, _totalSensorNum) != 0xFFFFFFFF)   //!= -1
        {
            //Store which index is repeated
            skipIndices[skipIndicesNum] = i;
            //Store how many are repeated
            skipIndicesNum++;
        }
    }

    if (_instanceNum == 1)
    {
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
    //Check for new sensors that need to be added
    else if (skipIndicesNum < _sensorNum) 
    {
        //Initialize data for upcoming procedure
        size_t oldSensorNum = _totalSensorNum;
        _totalSensorNum += _sensorNum - skipIndicesNum;
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

        //Separate index for new pins in a skip of index is necessary
        size_t pinsIndex = 0;

        //Add new Encoder to new memory
        for (size_t i = oldSensorNum; i < newSensorNum; i++)
        {
            //Increase until index is not one to skip
            while (_find(pinsIndex, skipIndices, skipIndicesNum) != 0xFFFFFFFF) //!= -1
            {
                //Move to next pin pair
                pinsIndex++;
            }

            //Populate new memory with new data
            _pins[i * _pinsPerSensor] = pins[pinsIndex * _pinsPerSensor];
            _pins[i * _pinsPerSensor + 1] = pins[pinsIndex * _pinsPerSensor + 1];
            _encodersPtr[i] = new Encoder(_pins[i * _pinsPerSensor],
                                          _pins[i * _pinsPerSensor + 1]);

            //Move to next pin pair
            pinsIndex++;
        }

        //Delete just old array not what it contained
        delete[] encodersPtr;
        delete[] oldPins;
    }

    //Initialize index translator due to varying order of pin input
    _indices = new size_t[_sensorNum];
    for (size_t i = 0; i < _sensorNum; i++)
    {
        _indices[i] = _find(pins, i, _pins, _totalSensorNum);
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
    delete[] _indices;
}

void Encoder_Wrapper::setCount(long int newCount, size_t sensor /*= 0*/)
{
    //Store new count that encoder was set to
    _setCounts[sensor] = newCount;
    //Store count at which encoder was reset
    _resetCounts[sensor] = _encodersPtr[_indices[sensor]]->read();
}

void Encoder_Wrapper::resetCount(size_t sensor /*= 0xFFFFFFFF*/)    //sensor = -1
{
    //Check if parameter was not entered
    if (sensor == 0xFFFFFFFF) //sensor == -1
    {
        //Iterate through encoders
        for (size_t i = 0; i < _sensorNum; i++)
        {
            //Set each encoder to zero
            setCount(0, i);
        }
    }
    else
    {
        //Set specific encoder to zero
        setCount(0, sensor);
    }
}

long int Encoder_Wrapper::getCount(size_t sensor /*= 0*/)
{
    //Return count by taking the delta of current reading and reset
    //Then adding the delta to the count that the encoder was set to
    return _encodersPtr[_indices[sensor]]->read() - _resetCounts[sensor] + _setCounts[sensor];
}

unsigned int Encoder_Wrapper::getPin(size_t sensor /*= 0*/, size_t index /*= 0*/) const
{
    return _pins[_indices[sensor] * _pinsPerSensor + index];
}

size_t Encoder_Wrapper::_find(unsigned int* newPins, size_t newSensorIndex, unsigned int *oldPins, size_t oldSensorNum)
{
    //Iterate through old pin data
    for (size_t oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
    {
        //Compare old data to new data check for similarity
        if((oldPins[oldSensor * _pinsPerSensor] == newPins[newSensorIndex * _pinsPerSensor]) 
        && (oldPins[oldSensor * _pinsPerSensor + 1] == newPins[newSensorIndex * _pinsPerSensor + 1]))
        {
            //Return running iterator if match found
            return oldSensor;
        }
    }

    //Return -1 if no match found
    return 0xFFFFFFFF;
}

size_t Encoder_Wrapper::_find(size_t newPinIndex, size_t *oldPinIndices, size_t oldPinIndicesNum)
{
    //Iterate through old pin index data
    for (size_t oldPin = 0; oldPin < oldPinIndicesNum; oldPin++)
    {
        //Compare to old data to new data check for similarity
        if (newPinIndex == oldPinIndices[oldPin])
        {
            //Return running iterator if match found
            return oldPin;
        }
    }

    //Return -1 if no match found
    return 0xFFFFFFFF;
}