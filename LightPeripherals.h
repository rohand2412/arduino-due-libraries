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