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

#pragma once
#include <Arduino.h>

class Ultrasonic
{
    private:
        const unsigned int 
            _trigPin,
            _echoPin,
            _burstFrequencyMS;
        
        unsigned int
            _burstMS,
            _roundTripTime,
            _distance,      //in mm
            _startMS;

    public:
        Ultrasonic(unsigned int trigPin,
                   unsigned int echoPin,
                   unsigned int burstFrequencyMS);

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance() const;

        unsigned int getEchoPin() const;

        unsigned int getTrigPin() const;

        void echoPinISR();
};