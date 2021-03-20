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