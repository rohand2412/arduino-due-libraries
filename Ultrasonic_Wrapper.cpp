#include <Arduino.h>
#include "Ultrasonic_Wrapper.h"

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin, 
                                       unsigned int* echoPins,
                                       unsigned int* burstFrequencies, 
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    _init(echoPins, burstFrequencies);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int* echoPins,
                                       unsigned int burstFrequency,
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    _init(echoPins, &burstFrequency);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int echoPin,
                                       unsigned int burstFrequency) :
                                       _trigPin(trigPin),
                                       _sensorNum(1)
{
    _init(&echoPin, &burstFrequency);
}

Ultrasonic_Wrapper::~Ultrasonic_Wrapper()
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        delete _ultrasonicsPtr[i];
    }
    delete[] _ultrasonicsPtr;
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISRs[])())
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        _ultrasonicsPtr[i]->begin(externalEchoPinISRs[i]);
    }
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISR)())
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        _ultrasonicsPtr[i]->begin(externalEchoPinISR);
    }
}

void Ultrasonic_Wrapper::update()
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        _ultrasonicsPtr[i]->update();
    }
}

unsigned int Ultrasonic_Wrapper::getDistance(unsigned int index /*= 0*/) const
{
    return _ultrasonicsPtr[index]->getDistance();
}

unsigned int Ultrasonic_Wrapper::getEchoPin(unsigned int index /*= 0*/) const
{
    return _ultrasonicsPtr[index]->getEchoPin();
}

unsigned int Ultrasonic_Wrapper::getTrigPin() const
{
    return _trigPin;
}

void Ultrasonic_Wrapper::echoPinISR(unsigned int index /*= 0*/)
{
    _ultrasonicsPtr[index]->echoPinISR();
}

void Ultrasonic_Wrapper::_init(unsigned int *echoPins, unsigned int *burstFrequencies)
{
    _ultrasonicsPtr = new Ultrasonic *[_sensorNum];
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        _ultrasonicsPtr[i] = new Ultrasonic(_trigPin, *echoPins++, *burstFrequencies++);
    }
}