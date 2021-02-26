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