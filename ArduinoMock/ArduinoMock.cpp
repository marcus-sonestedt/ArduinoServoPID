#include "pch.h"
#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

namespace mock
{
uint32_t              gMicros = 0;
std::vector<uint16_t> gAnalogPins(256);
}

MOCK_API MockSerial Serial;
MOCK_API MockWire   Wire;
MOCK_API MockEEPROM EEPROM;
