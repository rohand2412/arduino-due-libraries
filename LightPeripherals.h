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

class RGB_LED
{
    private:
        LED _red;
        LED _blue;
        LED _green;
    
    public:
        RGB_LED(unsigned int rPin, unsigned int bPin, unsigned int gPin);

        void begin();

        void setState(bool rState, bool bState, bool gState);

        void toggle();

        void on();

        void off();
};