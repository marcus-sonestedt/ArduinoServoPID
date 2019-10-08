#pragma once
#include "AdafruitPwmServoDriverMock.h"

MOCK_API Adafruit_PWMServoDriver& PwmController(); 
MOCK_API void ArduinoSetup();
MOCK_API void ArduinoLoop();
