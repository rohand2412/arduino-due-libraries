#include <Arduino.h>
#include "Serial_Wrapper.h"

UARTClass Serial_Wrapper::_port = Serial;

size_t Serial_Wrapper::_itemNum = 0;

Serial_Wrapper::Serial_Wrapper(){};

void Serial_Wrapper::begin(uint32_t baudRate, UARTClass& port)
{
    if (baudRate > 115200 && baudRate != 750000)
    {
        Serial.begin(115200);
        while (true)
        {
            Serial.println("[ERROR] UART_BRGR NOT SPECIFIED");
        }
    }

    port.begin(baudRate);

    if (baudRate == 750000)
    {
        UART->UART_BRGR = 8;
    }

    while (!port)
        ;
}

void Serial_Wrapper::setDefault(const UARTClass& port)
{
    _port = port;
}

void Serial_Wrapper::send(const uint8_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    port.write(_DELIMITER_BYTE);
    for (size_t item = 0; item < bufferLen; item++)
    {
        if (buffer[item] == _DELIMITER_BYTE || buffer[item] == _ESCAPE_BYTE)
        {
            port.write(_ESCAPE_BYTE);
            port.write(_escape(buffer[item]));
        }
        else
        {
            port.write(buffer[item]);
        }
    }
    port.write(_DELIMITER_BYTE);
}

size_t Serial_Wrapper::receive(uint8_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    while (port.available())
    {
        if (_receiveSM(buffer, &_itemNum, bufferLen, port.read()))
        {
            if (_itemNum == 0)
            {
                continue;
            }

            size_t itemNum = _itemNum;
            _itemNum = 0;
            return itemNum;
        }
    }

    return 0xFFFFFFFF; //return -1
}

bool Serial_Wrapper::_receiveSM(uint8_t *buffer, size_t *itemNum, size_t bufferLen, uint8_t byte_in)
{
    static _State state = _State::INIT;
    switch (state)
    {
        case _State::INIT:
            if (byte_in == _DELIMITER_BYTE)
            {
                state = _State::NORMAL;
            }
            return false;

        case _State::NORMAL:
            if (byte_in == _DELIMITER_BYTE)
            {
                return true;
            }
            if (byte_in == _ESCAPE_BYTE)
            {
                state = _State::ESCAPE;
                return false;
            }
            buffer[(*itemNum)++] = byte_in;
            return false;
        
        case _State::ESCAPE:
            buffer[(*itemNum)++] = _unescape(byte_in);
            state = _State::NORMAL;
            return false;
    }
}

uint8_t Serial_Wrapper::_escape(uint8_t raw)
{
    return raw ^ _CONVERSION;
}

uint8_t Serial_Wrapper::_unescape(uint8_t escaped)
{
    return escaped ^ _CONVERSION;
}