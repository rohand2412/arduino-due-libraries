#pragma once
#include <Arduino.h>

class Serial_Wrapper
{
    private:
        static const uint8_t _crc_calculator[0x20];

        static UARTClass _port;

    public:
        static void begin(uint32_t baudRate, UARTClass& port);

        static void setDefault(const UARTClass& port);

        static void send(const uint8_t* buffer, size_t bufferLen, UARTClass& port = _port);

        static size_t receive(uint8_t buffer, size_t bufferLen, UARTClass& port = _port);
    
    private:
        Serial_Wrapper();

        static uint8_t _doCRC(uint8_t incomingByte);

        static uint8_t _undoCRC(uint8_t incomingByte);
};