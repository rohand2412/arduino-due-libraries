/**
 * Ultrasonic sensor class optimized with interrupts
 * Copyright (C) 2022  Rohan Dugad
 * 
 * Contact info:
 * https://docs.google.com/document/d/17IhBs4cz7FXphE0praCaWMjz016a7BFU5IQbm1CNnUc/edit?usp=sharing
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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