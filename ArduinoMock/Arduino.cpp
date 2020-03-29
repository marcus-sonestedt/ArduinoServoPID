#include "pch.h"

#include "servopid.ino"
#include "AdafruitPwmServoDriverMock.h"

namespace
{
#if (!USE_PCA9685)
  // not defined in servopid.ino, so add here to prevent crashes and stuff
  Adafruit_PWMServoDriver gPwmController;
#endif
}

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
