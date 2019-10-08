#include "pch.h"

#include "Arduino.h"
#include "ArduinoMock.h"
#include "AdafruitPwmServoDriverMock.h"

extern "C" {

MOCK_API void Arduino_setup()
{
  ArduinoSetup();
}

MOCK_API void Arduino_loop()
{
  ArduinoLoop();
}

MOCK_API void Serial_Write(const char* str)
{
  Serial.writeMock(str);
}

MOCK_API void Serial_Read(char* buf, int32_t bufLen, int32_t* available)
{
  std::string str;
  str.reserve(bufLen);

  Serial.readMock(str);

  *available = str.length();
  str.copy(buf, bufLen);
}

MOCK_API void Serial_SetCallback(void (*callback)())
{
  Serial.setCallback(callback);
}

MOCK_API int32_t EEPROM_Size()
{
  return EEPROM.length();
}

MOCK_API void EEPROM_Read(char* buf, int32_t bufLen)
{
  const auto end = EEPROM._mem.begin() + std::min<int>(bufLen, EEPROM.length());
  std::copy(EEPROM._mem.begin(), end, buf);
}

MOCK_API int32_t PWM_NumServos()
{
  return PwmController()._pwmOn.size();
}

MOCK_API void PWM_Read(uint16_t* on, uint16_t* off)
{
  const auto& pwm = PwmController();
  std::copy(pwm._pwmOn.begin(), pwm._pwmOn.end(), on);
  std::copy(pwm._pwmOff.begin(), pwm._pwmOff.end(), off);
}

}
