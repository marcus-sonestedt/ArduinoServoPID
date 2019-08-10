#include "pch.h"
#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

namespace mock
{
unsigned long    gMicros = 0;
std::vector<int> gAnalogPins(256, 0);
}

MockSerial Serial;
MockWire   Wire;
MockEEPROM EEPROM;