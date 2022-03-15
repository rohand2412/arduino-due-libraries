/**
 * Adds modes and other functionality on top of raw button
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
#include "Button.h"

class Button_Wrapper
{
    private:
        Button _button;

        bool _buttonState;
        bool _switchState;

        uint8_t _mode;
    
    public:
        static const uint8_t BUTTON_SWITCH = 0;
        static const uint8_t BUTTON_INVERSE_SWITCH = 1;
        static const uint8_t BUTTON_RAW = 2;
        static const uint8_t BUTTON_INVERSE_RAW = 3;

    public:
        Button_Wrapper(uint8_t pin, uint8_t mode = BUTTON_SWITCH,
                       unsigned long debounceInterval = 30);

        void begin(void (*externalPinISR)());

        void update();

        void setMode(uint8_t mode);

        uint8_t getMode() const;

        void setState(bool state);

        bool getState() const;

        uint8_t getPin() const;

        void pinISR();
};