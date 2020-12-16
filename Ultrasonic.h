#pragma once
#include <Arduino.h>

class Ultrasonic
{
    private:
        const unsigned int 
            _trigPin,
            _echoPin,
            _burstFrequencyMS;
        
        unsigned int
            _burstMS,
            _roundTripTime,
            _distance,      //in mm
            _startMS;

    public:
        Ultrasonic(unsigned int trigPin,
                   unsigned int echoPin,
                   unsigned int burstFrequencyMS);

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance() const;

        unsigned int getEchoPin() const;

        unsigned int getTrigPin() const;

        void echoPinISR();
};