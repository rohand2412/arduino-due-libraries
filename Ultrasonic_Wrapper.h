/**
 * Holds multiple Ultrasonic sensor objects together
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

#pragma once
#include <Arduino.h>
#include "Ultrasonic.h"

class Ultrasonic_Wrapper
{
    private:
        const unsigned int _trigPin;

        const size_t _sensorNum;

        Ultrasonic **_ultrasonicsPtr;

    public:
        static const size_t ULTRASONIC_FRONT = 0;
        static const size_t ULTRASONIC_LEFT = 1;
        static const size_t ULTRASONIC_RIGHT = 2;
        static const size_t ULTRASONIC_BACK = 3;
        static const size_t ULTRASONIC_TOP = 4;

    public:
        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int* burstFrequencies,
                           size_t sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int burstFrequency,
                           size_t sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int echoPin,
                           unsigned int burstFrequency);

        ~Ultrasonic_Wrapper();

        void begin(void (*externalEchoPinISRs[])());

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance(size_t sensor = ULTRASONIC_FRONT) const;

        unsigned int getEchoPin(size_t sensor = ULTRASONIC_FRONT) const;

        unsigned int getTrigPin() const;

        void echoPinISR(size_t sensor = ULTRASONIC_FRONT);
    
    private:
        void _init(unsigned int *echoPins, unsigned int *burstFrequencies);
};