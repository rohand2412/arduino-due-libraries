#include <Arduino.h>
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(unsigned int trigPin, unsigned int echoPin, unsigned int burstFrequencyMS) : _trigPin(trigPin), _echoPin(echoPin), _burstFrequencyMS(burstFrequencyMS) 
{
    _burstMS = 0;
    _startMS = 0;
    _roundTripTime = 10000;     //will calculate to large distance so it won't trigger any procedures for obstacles since 10000 is not an actual end time
    _distance = 10000;          //so that it doesn't trigger any obstacle procedures
}

void Ultrasonic::begin(void (*externalEchoPinISR)())
{
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_echoPin), externalEchoPinISR, CHANGE);
}

void Ultrasonic::update()
{
    if(millis() >= (_burstMS + _burstFrequencyMS))
    {
        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);
        _burstMS = millis();
    }
    
    _distance = _roundTripTime * 0.343/2;
}

unsigned int Ultrasonic::getDistance() const
{
    return _distance;
}

unsigned int Ultrasonic::getEchoPin() const
{
    return _echoPin;
}

unsigned int Ultrasonic::getTrigPin() const
{
    return _trigPin;
}

void Ultrasonic::echoPinISR()
{
    if(digitalRead(_echoPin)==HIGH)
    {
        _startMS = micros();
    }
    else
    {
        _roundTripTime = micros() - _startMS;
    }
}