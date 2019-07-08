#include "pch.h"
#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

unsigned long gMicros = 0;
std::vector<int> gAnalogPins(256, 0);
MockSerial Serial;
