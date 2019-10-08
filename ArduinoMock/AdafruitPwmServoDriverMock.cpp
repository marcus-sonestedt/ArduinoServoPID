#include "pch.h"

#include "AdafruitPwmServoDriverMock.h"

void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert) {
  // Clamp value between 0 and 4095 inclusive.
  val = std::min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, val);
    }
  }
}

