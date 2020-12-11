#include <Arduino.h>
#include "Ultrasonic_Wrapper.h"

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin, 
                                       unsigned int echoPins[],
                                       unsigned int burstFrequencies[], 
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    _ultrasonics.reserve(_sensorNum);
    for (int i = 0; i < _ultrasonics.size(); i++)
    {
        _ultrasonics.emplace_back(_trigPin, echoPins[i], burstFrequencies[i]);
    }
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int echoPins[],
                                       unsigned int burstFrequency,
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    _ultrasonics.reserve(_sensorNum);
    for (int i = 0; i < _ultrasonics.size(); i++)
    {
        _ultrasonics.emplace_back(_trigPin, echoPins[i], burstFrequency);
    }
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int echoPin,
                                       unsigned int burstFrequency) :
                                       _trigPin(trigPin),
                                       _sensorNum(1)
{
    _ultrasonics.reserve(_sensorNum);
    for (int i = 0; i < _ultrasonics.size(); i++)
    {
        _ultrasonics.emplace_back(_trigPin, echoPin, burstFrequency);
    }
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISRs[])())
{
    for (int i = 0; i < _ultrasonics.size(); i++)
    {
        _ultrasonics[i].begin(externalEchoPinISRs[i]);
    }
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISR)())
{
    for (int i = 0; i < _ultrasonics.size(); i++)
    {
        _ultrasonics[i].begin(externalEchoPinISR);
    }
}

void Ultrasonic_Wrapper::update()
{
    for (Ultrasonic& us: _ultrasonics)
    {
        us.update();
    }
}

unsigned int Ultrasonic_Wrapper::getDistance(unsigned int index /*= 0*/) const
{
    return _ultrasonics[index].getDistance();
}

unsigned int Ultrasonic_Wrapper::getEchoPin(unsigned int index /*= 0*/) const
{
    return _ultrasonics[index].getEchoPin();
}

unsigned int Ultrasonic_Wrapper::getTrigPin() const
{
    return _trigPin;
}

void Ultrasonic_Wrapper::echoPinISR(unsigned int index /*= 0*/)
{
    _ultrasonics[index].echoPinISR();
}