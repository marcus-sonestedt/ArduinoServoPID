#include "pch.h"

#include "servopid.ino"

MOCK_API Adafruit_PWMServoDriver& PwmController()
{
  return gPwmController;
}

MOCK_API void ArduinoSetup()
{
  setup();
};

MOCK_API void ArduinoLoop()
{
  loop();
}
