#pragma once
#include <Arduino.h>

class Serial_Wrapper
{
    private:
        static const uint8_t _CRC_CALCULATOR[0x20];

        static constexpr uint8_t _DELIMITER_BYTE = 0xff;
        static constexpr uint8_t _ESCAPE_BYTE = 0xfe;
        static constexpr uint8_t _CONVERSION = 0x80;

        static UARTClass _port;

        enum class _State
        {
            INIT,
            NORMAL,
            ESCAPE
        };

        static size_t _itemNum;

    public:
        static void begin(uint32_t baudRate, UARTClass& port);

        static void setDefault(const UARTClass& port);

        static void send(const uint8_t* buffer, size_t bufferLen, UARTClass& port = _port);

        static size_t receive(uint8_t* buffer, size_t bufferLen, UARTClass& port = _port);
    
    private:
        Serial_Wrapper();

        static bool _receiveSM(uint8_t *buffer, size_t *i, size_t bufferLen, uint8_t byte_in);

        static uint8_t _doCRC(uint8_t message);

        static uint8_t _undoCRC(uint8_t crc_byte);

        static uint8_t _escape(uint8_t raw);

        static uint8_t _unescape(uint8_t escaped);
};