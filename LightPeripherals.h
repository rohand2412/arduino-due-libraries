#pragma once
#include <Arduino.h>

class LED
{
    private:
        unsigned int _pin;

        bool _state = false;

    public:
        LED(unsigned int pin);

        void begin();

        void setState(bool state);

        void toggle();

        void on();

        void off();
};