#include "ArduinoMock.h"

// ReSharper disable CppInconsistentNaming

int gMicros = 0;
std::vector<int> gAnalogPins(256, 0);
MockSerial Serial;
