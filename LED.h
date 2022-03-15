/**
 * Wraps an LED with convenient methods
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

class LED
{
    private:
        unsigned int _pin;

        bool _state = false;

        uint8_t _pwm = 0;

    public:
        LED(unsigned int pin);

        void begin();

        void setPwm(uint8_t pwm);

        uint8_t getPwm() const;

        void setState(bool state);

        bool getState() const;

        void toggle();

        void on();

        void off();

    private:
        void _update();
};