#include "pch.h"
#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

namespace mock
{
MOCK_API uint32_t              gMicros = 0;
MOCK_API std::vector<uint16_t> gAnalogPins(16);
MOCK_API std::vector<uint8_t> gPwmPins(16);
}

MOCK_API MockSerial Serial;
MOCK_API MockWire   Wire;
MOCK_API MockEEPROM EEPROM;
