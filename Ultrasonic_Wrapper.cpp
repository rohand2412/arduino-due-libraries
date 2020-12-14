#include <Arduino.h>
#include <new>
#include "Ultrasonic_Wrapper.h"

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin, 
                                       unsigned int* echoPins,
                                       unsigned int* burstFrequencies, 
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    char *ultrasonics = new char[_sensorNum * sizeof(Ultrasonic)];
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        Ultrasonic *ultrasonic = new (&ultrasonics[i * sizeof(Ultrasonic)])
        Ultrasonic(_trigPin, *echoPins++, *burstFrequencies++); //end of previous line
    }
    _ultrasonics = reinterpret_cast<Ultrasonic *>(ultrasonics);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int* echoPins,
                                       unsigned int burstFrequency,
                                       unsigned int sensorNum) :
                                       _trigPin(trigPin),
                                       _sensorNum(sensorNum)
{
    char *ultrasonics = new char[_sensorNum * sizeof(Ultrasonic)];
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        Ultrasonic *ultrasonic = new (&ultrasonics[i * sizeof(Ultrasonic)])
        Ultrasonic(_trigPin, *echoPins++, burstFrequency); //end of previous line
    }
    _ultrasonics = reinterpret_cast<Ultrasonic *>(ultrasonics);
}

Ultrasonic_Wrapper::Ultrasonic_Wrapper(unsigned int trigPin,
                                       unsigned int echoPin,
                                       unsigned int burstFrequency) :
                                       _trigPin(trigPin),
                                       _sensorNum(1)
{
    char *ultrasonics = new char[_sensorNum * sizeof(Ultrasonic)];
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        Ultrasonic *ultrasonic = new (&ultrasonics[i * sizeof(Ultrasonic)])
        Ultrasonic(_trigPin, echoPin, burstFrequency); //end of previous line
    }
    _ultrasonics = reinterpret_cast<Ultrasonic *>(ultrasonics);
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISRs[])())
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        (*(_ultrasonics + i)).begin(externalEchoPinISRs[i]);
    }
}

void Ultrasonic_Wrapper::begin(void (*externalEchoPinISR)())
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        (*(_ultrasonics + i)).begin(externalEchoPinISR);
    }
}

void Ultrasonic_Wrapper::update()
{
    for (unsigned int i = 0; i < _sensorNum; i++)
    {
        (*(_ultrasonics + i)).update();
    }
}

unsigned int Ultrasonic_Wrapper::getDistance(unsigned int index /*= 0*/) const
{
    return (*(_ultrasonics + index)).getDistance();
}

unsigned int Ultrasonic_Wrapper::getEchoPin(unsigned int index /*= 0*/) const
{
    return (*(_ultrasonics + index)).getEchoPin();
}

unsigned int Ultrasonic_Wrapper::getTrigPin() const
{
    return _trigPin;
}

void Ultrasonic_Wrapper::echoPinISR(unsigned int index /*= 0*/)
{
    (*(_ultrasonics + index)).echoPinISR();
}