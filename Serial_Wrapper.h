/**
 * Wraps Serial port with a CRC and a state machine for sending packets with items of size int32_t
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

class Serial_Wrapper
{
    private:
        static const uint8_t _CRC_CALCULATOR[0x20];

        static constexpr uint8_t _PACKET_DELIMITER_BYTE = 0x1f;
        static constexpr uint8_t _ITEM_DELIMITER_BYTE = 0x1d;
        static constexpr uint8_t _ESCAPE_BYTE = 0x1e;
        static constexpr uint8_t _CONVERSION = 0x10;

        static constexpr size_t _ITEM_BIT_LEN = 5;
        static constexpr size_t _MAX_ITEM_BYTES = 7;

        static UARTClass _port;

        enum class _State
        {
            INIT,
            NORMAL,
            ESCAPE
        };
        static _State _state;

        static size_t _itemNum;

    public:
        static void begin(uint32_t baudRate, UARTClass& port);

        static void setDefault(const UARTClass& port);

        static void send(const int32_t* buffer, size_t bufferLen, UARTClass& port = _port);

        static size_t receive(int32_t* buffer, size_t bufferLen, UARTClass& port = _port);

    private:
        Serial_Wrapper();

        static void _write(uint8_t item, UARTClass& port);

        static bool _receiveSM(int32_t *buffer, size_t *i, size_t bufferLen, uint8_t byte_in);

        static uint8_t _doCRC(uint8_t message);

        static uint8_t _undoCRC(uint8_t crc_byte);

        static uint8_t _escape(uint8_t raw);

        static uint8_t _unescape(uint8_t escaped);
};