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

#include <Arduino.h>
#include "RGB_LED.h"

RGB_LED::RGB_LED(unsigned int rPin, unsigned int gPin, unsigned int bPin)
    : _red(rPin), _green(gPin), _blue(bPin)
{}

void RGB_LED::begin()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].begin();
    }
}

void RGB_LED::setPwm(uint8_t pwm, size_t led)
{
    _leds[led].setPwm(pwm);
}

void RGB_LED::setPwm(uint8_t rPwm, uint8_t gPwm, uint8_t bPwm)
{
    setPwm(rPwm, RED_INDEX);
    setPwm(gPwm, GREEN_INDEX);
    setPwm(bPwm, BLUE_INDEX);
}

uint8_t RGB_LED::getPwm(size_t led) const
{
    return _leds[led].getPwm();
}

void RGB_LED::setState(bool state)
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].setState(state);
    }
}

void RGB_LED::toggle()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].toggle();
    }
}

void RGB_LED::on()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].on();
    }
}

void RGB_LED::off()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].off();
    }
}