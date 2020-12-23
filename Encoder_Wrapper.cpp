#include <Arduino.h>
#include "Encoder_Wrapper.h"
#include "Utilities.h"

//Initialize static variables
unsigned int Encoder_Wrapper::_instanceNum = 0;

size_t Encoder_Wrapper::_totalSensorNum = -1;

Encoder **Encoder_Wrapper::_encodersPtr;

unsigned int *Encoder_Wrapper::_pins;

Encoder_Wrapper::Encoder_Wrapper(){};

Encoder_Wrapper::Encoder_Wrapper(unsigned int* pins, size_t sensorNum /*= 1*/)
{
    //Fill and initialize instance
    begin(pins, sensorNum);
}

void Encoder_Wrapper::begin(unsigned int* pins, size_t sensorNum /*= 1*/)
{
    //Log initialization of new instance
    _instanceNum++;

    //Store number of sensors
    _sensorNum = sensorNum;

    //Declare class specific data
    _resetCounts = new long int[_sensorNum];
    _setCounts = new long int[_sensorNum];
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Initialize data
        _resetCounts[sensor] = 0;
        _setCounts[sensor] = 0;
    }

    //Declare array of skipIndices and its length
    size_t skipIndices[_sensorNum];
    size_t skipIndicesNum = 0;

    //Check if this is not the first instance
    if (_instanceNum != 1)
    {
        //Iterate through sensors
        for (size_t sensor = 0; sensor < _sensorNum; sensor++)
        {
            //Check if pin pair is repeated
            if (_find(pins[sensor * PINS_PER_SENSOR], PINS_PER_SENSOR, _pins, _totalSensorNum) != 0xFFFFFFFF)   //!= -1
            {
                //Store which index is repeated
                skipIndices[skipIndicesNum] = sensor;
                //Store how many are repeated
                skipIndicesNum++;
            }
        }
    }

    if (_instanceNum == 1)
    {
        //Initialize _totalSensorNum
        _totalSensorNum = _sensorNum;

        //Construct static data structures
        _construct(pins, _totalSensorNum);
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

        //Allocate and populate memory with old data
        _construct(oldPins, newSensorNum, oldSensorNum);

        //Separate index for new pins in a skip of index is necessary
        size_t pinsIndex = 0;

        //Add new sensors to new memory
        for (size_t sensor = oldSensorNum; sensor < newSensorNum; sensor++)
        {
            //Increase until index is not one to skip
            while (_find(pinsIndex, skipIndices, skipIndicesNum) != 0xFFFFFFFF) //!= -1
            {
                //Move to next pin pair
                pinsIndex++;
            }

            //Populate new memory with new data
            _pins[sensor * PINS_PER_SENSOR + ENCODER_OUT_A] = pins[pinsIndex * PINS_PER_SENSOR + ENCODER_OUT_A];
            _pins[sensor * PINS_PER_SENSOR + ENCODER_OUT_B] = pins[pinsIndex * PINS_PER_SENSOR + ENCODER_OUT_B];
            _encodersPtr[sensor] = new Encoder(_pins[sensor * PINS_PER_SENSOR + ENCODER_OUT_A],
                                               _pins[sensor * PINS_PER_SENSOR + ENCODER_OUT_B]);

            //Move to next pin pair
            pinsIndex++;
        }

        //Delete just old array not what it contained
        delete[] encodersPtr;
        delete[] oldPins;
    }

    //Initialize index translator due to varying order of pin input
    _indices = new size_t[_sensorNum];
    _pinIndices = new size_t[_sensorNum * PINS_PER_SENSOR];
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        _indices[sensor] = _find(pins[sensor * PINS_PER_SENSOR], PINS_PER_SENSOR, _pins, _totalSensorNum);
        for (size_t pinIndex = 0; pinIndex < PINS_PER_SENSOR; pinIndex++)
        {
            _pinIndices[sensor * PINS_PER_SENSOR + pinIndex] = _find(pins[sensor * PINS_PER_SENSOR + pinIndex], _pins, _totalSensorNum * PINS_PER_SENSOR);
        }
    }
}

Encoder_Wrapper::~Encoder_Wrapper()
{
    //Check if this is only or last instance left
    if (_instanceNum == 1)
    {
        //Iterate through sensors
        for (size_t sensor = 0; sensor < _totalSensorNum; sensor++)
        {
            //Destroy each individual sensor
            delete _encodersPtr[sensor];
        }

        //Destroy encapsulating arrays
        delete[] _encodersPtr;
        delete[] _pins;
    }

    //Subtract this instance from total
    _instanceNum--;

    //Destory all instance specific data
    delete[] _resetCounts;
    delete[] _setCounts;
    delete[] _indices;
}

void Encoder_Wrapper::setCount(size_t sensor, long int newCount)
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);
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
        //Iterate through sensors
        for (size_t sensor = 0; sensor < _sensorNum; sensor++)
        {
            //Set each encoder to zero
            setCount(sensor, 0);
        }
    }
    else
    {
        //Prevent array overflow
        sensor = Utilities::indexCap(sensor, _sensorNum);

        //Set specific encoder to zero
        setCount(sensor, 0);
    }
}

long int Encoder_Wrapper::getCount(size_t sensor /*= ENCODER_LEFT*/)
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);

    //Return count by taking the delta of current reading and reset
    //Then adding the delta to the count that the encoder was set to
    return _encodersPtr[_indices[sensor]]->read() - _resetCounts[sensor] + _setCounts[sensor];
}

unsigned int Encoder_Wrapper::getPin(size_t sensor /*= ENCODER_LEFT*/, size_t index /*= ENCODER_OUT_A*/) const
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);
    index = Utilities::indexCap(index, PINS_PER_SENSOR);

    //Return specified pin
    return _pins[_pinIndices[sensor * PINS_PER_SENSOR + index]];
}

size_t Encoder_Wrapper::getSensorNum() const
{
    //Return instance specific number of encoders
    return _sensorNum;
}

size_t Encoder_Wrapper::getTotalSensorNum()
{
    //Return class wide number of encoders
    return _totalSensorNum;
}

size_t Encoder_Wrapper::_find(unsigned int& newPins, size_t newPinLen, unsigned int *oldPins, size_t oldSensorNum)
{
    //Convert first pin to pin pair array
    unsigned int *newPinsPtr = &newPins;

    //Pin iterator initialized where loop won't destroy it
    size_t newPin = 0;
    size_t oldPin = 0;

    //Iterate through old pin data
    for (size_t oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
    {
        //Switch between search methods for ordered and unordered pins
        for (size_t searchMethod = 0; searchMethod < 2; searchMethod++)
        {
            //Iterate through new pins
            for (newPin = 0; newPin < newPinLen; newPin++)
            {
                //Search method one assumes that pin is in correct order
                if (searchMethod == 0)
                {
                    //oldPin and newPin act as same
                    oldPin = newPin;
                }

                //Search method two assumes that pin is in wrong order
                else if (searchMethod == 1)
                {
                    //oldPin is iterating backwards unlike newPin
                    //1 is because indices begin on 0 not 1
                    oldPin = newPinLen - newPin - 1;
                }

                //Compare old data to new data check for similarity
                if (oldPins[oldSensor * PINS_PER_SENSOR + oldPin] != newPinsPtr[newPin])
                {
                    //Freeze newPin at current iteration value
                    break;
                }
            }

            //Check if loop wasn't broken and all pins matched
            if (newPin == newPinLen)
            {
                //Return running iterator if match found
                return oldSensor;
            }
        }
    }

    //Return -1 if no match found
    return 0xFFFFFFFF;
}

size_t Encoder_Wrapper::_find(size_t newPinIndex, size_t *oldPinIndices, size_t oldPinIndicesNum)
{
    //Iterate through old pin indices
    for (size_t oldPinIndex = 0; oldPinIndex < oldPinIndicesNum; oldPinIndex++)
    {
        //Check if pin index matches old pin iteration
        if (oldPinIndices[oldPinIndex] == newPinIndex)
        {
            //Return running iterator if match found
            return oldPinIndex;
        }
    }

    //Return -1 if no match found
    return 0xFFFFFFFF;
}

void Encoder_Wrapper::_construct(unsigned int *pins, size_t newSensorNum, size_t oldSensorNum)
{
    //Allocate new memory
    _encodersPtr = new Encoder *[newSensorNum];
    _pins = new unsigned int[newSensorNum * PINS_PER_SENSOR];

    //Iterate through sensors
    for (size_t oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
    {
        //Populate allocated memory
        _pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_A] = pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_A];
        _pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_B] = pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_B];
        _encodersPtr[oldSensor] = new Encoder(_pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_A],
                                              _pins[oldSensor * PINS_PER_SENSOR + ENCODER_OUT_B]);
    }
}

void Encoder_Wrapper::_construct(unsigned int *pins, size_t sensorNum)
{
    //Use other version of find for when the
    //new and old sensor num are the same
    _construct(pins, sensorNum, sensorNum);
}