#include <Arduino.h>
#include "Ultrasonic_Wrapper.h"
#include "Utilities.h"

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin, 
                                       unsigned int* echoPins,
                                       unsigned int* burstFrequencies, 
                                       size_t sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    //Initialize instance
    _init(echoPins, burstFrequencies);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int* echoPins,
                                       unsigned int burstFrequency,
                                       size_t sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    //Initialize instance
    _init(echoPins, &burstFrequency);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int echoPin,
                                       unsigned int burstFrequency) :
                                       _trigPin(trigPin),
                                       _sensorNum(1)
{
    //Initialize instance
    _init(&echoPin, &burstFrequency);
}

Ultrasonic_Wrapper::~Ultrasonic_Wrapper()
{
    //Iterate through sensors
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Destroy each sensor individually
        delete _ultrasonicsPtr[sensor];
    }

    //Destroy encapsulating array
    delete[] _ultrasonicsPtr;
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISRs[])())
{
    //Iterate through sensors
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Begin set up procedures with unique ISR
        _ultrasonicsPtr[sensor]->begin(externalEchoPinISRs[sensor]);
    }
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISR)())
{
    //Iterate through sensors
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Begin set up procedures with same ISR
        _ultrasonicsPtr[sensor]->begin(externalEchoPinISR);
    }
}

void Ultrasonic_Wrapper::update()
{
    //Iterate through sensors
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Update sensor
        _ultrasonicsPtr[sensor]->update();
    }
}

unsigned int Ultrasonic_Wrapper::getDistance(size_t sensor /*= ULTRASONIC_FRONT*/) const
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);

    //Return distance of specified sensor
    return _ultrasonicsPtr[sensor]->getDistance();
}

unsigned int Ultrasonic_Wrapper::getEchoPin(size_t sensor /*= ULTRASONIC_FRONT*/) const
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);

    //Return echo pin of specified sensor
    return _ultrasonicsPtr[sensor]->getEchoPin();
}

unsigned int Ultrasonic_Wrapper::getTrigPin() const
{
    //Return shared trig pin
    return _trigPin;
}

void Ultrasonic_Wrapper::echoPinISR(size_t sensor /*= ULTRASONIC_FRONT*/)
{
    //Prevent array overflow
    sensor = Utilities::indexCap(sensor, _sensorNum);

    //ISR wrapper for specified sensor
    _ultrasonicsPtr[sensor]->echoPinISR();
}

void Ultrasonic_Wrapper::_init(unsigned int *echoPins, unsigned int *burstFrequencies)
{
    //Allocate memory
    _ultrasonicsPtr = new Ultrasonic *[_sensorNum];

    //Iterate through sensors
    for (size_t sensor = 0; sensor < _sensorNum; sensor++)
    {
        //Populate memory
        _ultrasonicsPtr[sensor] = new Ultrasonic(_trigPin, echoPins[sensor], burstFrequencies[sensor]);
    }
}