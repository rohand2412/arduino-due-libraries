/**
 * Wraps the R, G, and B channels of an RGB_LED
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
#include "Arduino.h"
#include "LED.h"

class RGB_LED
{
    public:
        static const size_t LED_NUM = 3;
        static const size_t RED_INDEX = 0;
        static const size_t GREEN_INDEX = 1;
        static const size_t BLUE_INDEX = 2;

    private:
        LED _red;
        LED _green;
        LED _blue;

        LED _leds[LED_NUM] = {_red, _green, _blue};

    public:
        RGB_LED(unsigned int rPin, unsigned int gPin, unsigned int bPin);

        void begin();

        void setPwm(uint8_t pwm, size_t led);

        void setPwm(uint8_t rPwm, uint8_t gPwm, uint8_t bPwm);

        uint8_t getPwm(size_t led) const;

        void setState(bool state);

        void toggle();

        void on();

        void off();
};