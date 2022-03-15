/**
 * Button class optimized with interrupts
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

class Button
{
    private:
        const uint8_t _pin;

        bool _state = false;

        const unsigned long _debounceInterval;

        volatile unsigned long _lastInterruptTime;
    
    public:
        Button(uint8_t pin, unsigned long debounceInterval = 30);

        void begin(void (*externalPinISR)());

        void update();

        bool getState() const;

        uint8_t getPin() const;

        void pinISR();
};