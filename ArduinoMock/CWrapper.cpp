#include "pch.h"

#include "Arduino.h"
#include "ArduinoMock.h"
#include "AdafruitPwmServoDriverMock.h"

using namespace std;

namespace
{
  recursive_mutex gMutex;

  typedef unique_lock<recursive_mutex> unique_recursive_lock;
}

extern "C" {

MOCK_API void __cdecl Arduino_Setup()
{
  const unique_recursive_lock lock(gMutex);

  ArduinoSetup();
}

MOCK_API void __cdecl Arduino_Loop()
{
  const unique_recursive_lock lock(gMutex);

  ArduinoLoop();
}

MOCK_API void __cdecl Set_Micros(uint32_t micros)
{
  setMockMicros(micros);
}

MOCK_API void __cdecl Serial_Write(const char* str, int32_t strLen)
{
  const unique_recursive_lock lock(gMutex);

  Serial.writeMock(string(str, str + strLen));
}

MOCK_API void __cdecl Serial_Read(char* buf, int32_t bufLen, int32_t* available)
{
  const unique_recursive_lock lock(gMutex);

  const auto str = Serial.readMock();

  *available = str.length();
  str.copy(buf, bufLen);
}

MOCK_API void __cdecl Serial_SetCallback(void (*callback)())
{
  const unique_recursive_lock lock(gMutex);

  Serial.setCallback(callback);
}

MOCK_API int32_t __cdecl EEPROM_Size()
{
  const unique_recursive_lock lock(gMutex);

  return EEPROM.length();
}

MOCK_API void __cdecl EEPROM_Read(char* buf, int32_t bufLen)
{
  const unique_recursive_lock lock(gMutex);

  const auto end = EEPROM._mem.begin() + min<int>(bufLen, EEPROM.length());
  copy(EEPROM._mem.begin(), end, buf);
}

MOCK_API int32_t __cdecl PWM_NumServos()
{
  const unique_recursive_lock lock(gMutex);

  return PwmController()._pwmOn.size();
}

MOCK_API void __cdecl PWM_Read(uint16_t* on, uint16_t* off)
{
  const unique_recursive_lock lock(gMutex);

  const auto& pwm = PwmController();
  copy(pwm._pwmOn.begin(), pwm._pwmOn.end(), on);
  copy(pwm._pwmOff.begin(), pwm._pwmOff.end(), off);
}

MOCK_API void __cdecl AnalogInput_Set(uint8_t pin, uint16_t value)
{
  setMockAnalogRead(pin, value);
}


}
