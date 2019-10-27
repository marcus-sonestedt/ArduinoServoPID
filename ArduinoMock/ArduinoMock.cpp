#include "pch.h"
#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

namespace mock
{
unsigned long    gMicros = 0;
std::vector<int> gAnalogPins(256);
}

MOCK_API MockSerial Serial;
MOCK_API MockWire   Wire;
MOCK_API MockEEPROM EEPROM;