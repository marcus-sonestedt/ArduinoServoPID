#define USE_PCA9685 1 // set 0 to use Arduino to directly control servos

#ifdef ARDUINO
#if USE_PCA9685 == 1
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#else
#include <Servo.h>
#endif
     #include <EEPROM.h> 
#else
#include "../ArduinoMock/ArduinoMock.h"

#if USE_PCA9685 == 1
#include "../ArduinoMock/AdafruitPwmServoDriverMock.h"
#endif

#endif

#ifdef SERVOPID_TEST
namespace
{
#endif

// PID regulator, incl low-pass lambda filter on D part
class PID
{
public:
  // adjust to control the amount of "energy" that the PID integrator can store
  static int MaxIntegratorStore;

  PID() = default;

  PID(float p, float i, float d, float dLambda)
  {
    _pFactor = p;
    _iFactor = i;
    _dFactor = d;
    _dLambda = dLambda;

    reset(0.0f);
  }

  void reset(float currentValue)
  {
    _integral = 0.0f;
    _prevValue = currentValue;
    _dValueFiltered = 0.0f;
  }

  float regulate(float currentValue, float requestedValue, float dt)
  {
    const auto error = requestedValue - currentValue;
    const auto dValue = (currentValue - _prevValue) / dt;

    _prevValue = currentValue;
    _dValueFiltered = (dValue * _dLambda) + (_dValueFiltered * (1.0f - _dLambda));
    _integral += error * dt;

    // limit "energy" storage in integrator
    _integral = constrain(_integral , -MaxIntegratorStore, MaxIntegratorStore);
    
    return _pFactor * error + _iFactor * _integral - _dFactor * _dValueFiltered;
  }

  float _pFactor = 0;
  float _iFactor = 0;
  float _dFactor = 0;
  float _dLambda = 0;

  float _integral = 0;
  float _dValueFiltered = 0;
  float _prevValue = 0;
};

int PID::MaxIntegratorStore = 5000;

// Analog input, with min/max settings
class AnalogPin
{
public:
  static uint16_t Range;

  AnalogPin() = default;

  AnalogPin(int pin, uint16_t min, uint16_t max)
  {
    _pin = pin;
    _scale = 1.0f / float(max - min);
    _bias = float(-min);
  }

  float read() const
  {
     const auto value = analogRead(_pin);
     const auto angle = ((value * _scale) + _bias) * Range;
     return angle;
  }

  float   _scale = 1;
  float   _bias = 0;
  uint8_t _pin = 0;
};

uint16_t AnalogPin::Range = 320;

class ServoBase
{
public:
  static uint16_t MinAngle;
  static uint16_t MaxAngle;
};

uint16_t ServoBase::MinAngle = 80;
uint16_t ServoBase::MaxAngle = 100;

#if USE_PCA9685 == 1

Adafruit_PWMServoDriver gPwmController = Adafruit_PWMServoDriver();

class PCA9685Servo : public ServoBase
{
public:
  PCA9685Servo() = default;

  void attach(const int pin, const int pwmMin, const int pwmMax)
  {
    _pin = pin;
    _pwmMin = pwmMin;
    _pwmMax = pwmMax;
  }

  void reset()
  {
  }

  // ReSharper disable once CppMemberFunctionMayBeConst
  void write(const float angle)
  {
    const auto cAngle = constrain(angle, float(MinAngle), float(MaxAngle));
    const auto pwm = _pwmMin + (_pwmMax - _pwmMin) * ((cAngle - MinAngle) / (float(MaxAngle) - float(MinAngle)));
    const auto cPwm = constrain(pwm, _pwmMin, _pwmMax);
    gPwmController.setPWM(_pin, 0, uint16_t(cPwm));
  }

  uint8_t  _pin = 0;
  uint16_t _pwmMin = 150;
  uint16_t _pwmMax = 600;
};

#else

class ArduinoServo : public Servo, public ServoBase
{
public:

    void attach(const int pin, const int pwmMin, const int pwmMax)
    {
        _pin = pin;
        _pwmMin = pwmMin;
        _pwmMax = pwmMax;
        reset();
    }

    void write(float angle)
    {
        const auto cAngle = constrain(int(angle), MinAngle, MaxAngle);
        Servo::write(cAngle);
    }

    void reset()
    {
        Servo::attach(_pin, _pwmMin, _pwmMax);
    }

private:
    uint8_t _pin = 0;
    uint16_t _pwmMin = 150;
    uint16_t _pwmMax = 600;
};

#endif


class PidServo
{
public:
  static bool enabled;

  PidServo() = default;

  PidServo(int servoPin, int servoMin, int servoMax, PID pid, AnalogPin analogPin)
  {
    _servo.attach(servoPin, servoMin, servoMax);
    _pid = pid;
    _analogPin = analogPin;
    _setPoint = 90.0f;
  }

  void setPoint(float setPoint)
  {
    _setPoint = setPoint;
  }

  void run(const float dt)
  {
    _input = _analogPin.read();

    if (enabled)
      _output = _pid.regulate(_input, _setPoint, dt);

    _servo.write(_output);
  }

  void reset()
  {
    _input = _analogPin.read();
    _output = _setPoint;
    _servo.reset();
    _pid.reset(_input);
  }

#if USE_PCA9685 == 1
  PCA9685Servo _servo;
#else
    ArduinoServo _servo;
#endif
  PID       _pid;
  AnalogPin _analogPin;

  float _setPoint = 90.0f;
  float _input = 0;
  float _output = 0;
};

bool PidServo::enabled = true;

////////////////////////////////////////////////////////////////////

constexpr int MAX_SERVOS = 8;
unsigned int  numServos = 4;
PidServo      PidServos[MAX_SERVOS];

float prevTime = 0;
float dt = 0;

void initServosFromEeprom();

const int crcAddress = EEPROM.length() - sizeof(unsigned long);

void setup()
{
#if USE_PCA9685
    //Wire.begin();                       // Wire must be started first
    //Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

  gPwmController.begin();
  gPwmController.setPWMFreq(200);
#endif

  initServosFromEeprom();

  // start serial port
  Serial.begin(115200);
  while (Serial.available())
    Serial.read();

  // initiate timer
  delay(10);
  prevTime = 1e-6f * float(micros());
}

int   x = 0;
float maxDt = 0;
float minDt = 100;

void mySerialEvent();

void loop()
{
  // determine time step
  const auto t = 1e-6f * float(micros());
  dt = t - prevTime;
  prevTime = t;

  // regulate servos
  for (auto& PidServo : PidServos)
    PidServo.run(dt);

  maxDt = dt > maxDt ? dt : maxDt;
  minDt = dt < minDt ? dt : minDt;

  if (x++ % 100 == 0)
  {
    Serial.print(F("DT "));
    Serial.print(dt, 6);
    Serial.print(' ');
    Serial.print(minDt, 6);
    Serial.print(' ');
    Serial.print(maxDt, 6);
    Serial.print('\n');

    minDt = 100;
    maxDt = 0;
  }

  if (Serial.available())
    mySerialEvent();
}

enum class Command
{
  NoOp = 0,
  SetServoParamFloat,
  EnableRegulator,
  GetNumServos,
  GetServoParams,
  GetServoData,
  SetGlobalVar,
  GetGlobalVars,
  LoadEeprom,
  SaveEeprom,
  ResetToDefault,
};

enum class ServoParam
{
    None,
  P,
  I,
  D,
  DLambda,
  SetPoint,
  InputScale,
  InputBias
};

enum class GlobalVar
{
  NumServos,
  PidEnabled,
  PidMaxIntegratorStore,
  AnalogInputRange,
  ServoMinAngle,
  ServoMaxAngle,
};

unsigned char serialBuf[128] = {0};
size_t  serialLen = 0;

void handleSerialCommand();
bool loadEeprom();
void saveEeprom();
void resetToDefaultValues();

void mySerialEvent()
{
  while (Serial.available() > 0 && serialLen < sizeof(serialBuf))
  {
    serialBuf[serialLen++] = Serial.read();

    if (serialLen >= 4
      && serialBuf[serialLen - 4] == 'R'
      && serialBuf[serialLen - 3] == 'S'
      && serialBuf[serialLen - 2] == 'T'
      && serialBuf[serialLen - 1] == '\n')
    {
      Serial.print(F("RST ACK\n"));
      Serial.flush();
      serialLen = 0;
      continue;
    }

    if (serialLen >= sizeof(serialBuf))
    {
      Serial.print(F("ERR: Command buffer overflow\n"));
      serialLen = 0;
      continue;
    }

    if (int(serialBuf[0]) == serialLen)
    {
      handleSerialCommand();
      serialLen = 0;
    }
  }
}

void handleSerialCommand()
{
  switch (Command(serialBuf[1]))
  {
    // len, cmd, pid#, param-id, float-value[4]
  case Command::SetServoParamFloat:
    {
      if (serialBuf[2] >= numServos)
      {
        Serial.print(F("ERR: Invalid servo number "));
        Serial.print(int(serialBuf[2]));
        Serial.print('\n');
        return;
      }

      auto& servoPid = PidServos[int(serialBuf[2])];
      const auto value = *reinterpret_cast<float*>(serialBuf + 4);
      switch (ServoParam(serialBuf[3]))
      {
      case ServoParam::P: servoPid._pid._pFactor = value;
        break;
      case ServoParam::I: servoPid._pid._iFactor = value;
        break;
      case ServoParam::D: servoPid._pid._dFactor = value;
        break;
      case ServoParam::DLambda: servoPid._pid._dLambda = value;
        break;
      case ServoParam::SetPoint: servoPid._setPoint = value;
        break;
      case ServoParam::InputScale: servoPid._analogPin._scale = value;
        break;
      case ServoParam::InputBias: servoPid._analogPin._bias = int16_t(value);
        break;
      default:
        Serial.print(F("ERR: Unknown servo parameter "));
        Serial.print(int(serialBuf[3]));
        Serial.print('\n');
        return;
      }
    }
    Serial.print(F("OK"));
    Serial.print('\n');
    break;

  case Command::EnableRegulator:
    PidServo::enabled = serialBuf[1] != 0;
    Serial.print(F("OK"));
    Serial.print('\n');
    break;

  case Command::GetNumServos:
    Serial.print(F("NS "));
    Serial.print(numServos);
    Serial.print('\n');
    break;

  case Command::GetServoParams:
    for (auto i = 0u; i < numServos; ++i) // NOLINT(modernize-loop-convert)
    {
      const auto& servo = PidServos[i];

      Serial.print(F("SP "));
      Serial.print(i);
      Serial.print(' ');
      Serial.print(servo._pid._pFactor);
      Serial.print(' ');
      Serial.print(servo._pid._iFactor);
      Serial.print(' ');
      Serial.print(servo._pid._dFactor);
      Serial.print(' ');
      Serial.print(servo._pid._dLambda);
      Serial.print(' ');
      Serial.print(servo._setPoint);
      Serial.print('\n');
    }
    break;

  case Command::GetServoData:
    for (auto i = 0u; i < numServos; ++i)
    {
      // show only one servo if set
      if (serialBuf[2] != i && serialBuf[2] < numServos)
        continue;

      const auto& servo = PidServos[i];

      Serial.print(F("SD "));
      Serial.print(i);
      Serial.print(' ');
      Serial.print(servo._input);
      Serial.print(' ');
      Serial.print(servo._output);
      Serial.print(' ');
      Serial.print(servo._pid._integral);
      Serial.print(' ');
      Serial.print(servo._pid._dValueFiltered);
      Serial.print('\n');
    }
    break;

  case Command::SetGlobalVar:
    {
      const auto var = static_cast<GlobalVar>(serialBuf[2]);
      const auto value = *reinterpret_cast<float*>(serialBuf + 4);

      switch (var)
      {
      case GlobalVar::NumServos:
        if (int(value) >= MAX_SERVOS)
        {
          Serial.print("ERR: Max supported num servos is: ");
          Serial.println(MAX_SERVOS);
          return;
        }

        numServos = int(value);
        for (auto i = 0u; i < numServos; ++i)
        {
          PidServos[i] = PidServo();
          PidServos[i].reset();
        }
        PidServo::enabled = false;
        break;
      case GlobalVar::PidEnabled:
        PidServo::enabled = value != 0;
        break;
      case GlobalVar::PidMaxIntegratorStore:
        PID::MaxIntegratorStore = uint16_t(value);
        break;
      case GlobalVar::AnalogInputRange:
        AnalogPin::Range = uint16_t(value);
        break;
      case GlobalVar::ServoMinAngle:
        ServoBase::MinAngle = uint16_t(value);
        break;
      case GlobalVar::ServoMaxAngle:
        ServoBase::MaxAngle = uint16_t(value);
        break;
      default:
        Serial.print(F("ERR: Unknown global variable "));
        Serial.print(int(serialBuf[2]));
        Serial.print('\n');
        return;
      }
    }
    // fallthrough on GV update
  case Command::GetGlobalVars:
    {
      Serial.print(F("GV "));
      Serial.print(numServos);
      Serial.print(' ');
      Serial.print(PidServo::enabled ? 1 : 0);
      Serial.print(' ');
      Serial.print(PID::MaxIntegratorStore);
      Serial.print(' ');
      Serial.print(AnalogPin::Range);
      Serial.print(' ');
      Serial.print(ServoBase::MinAngle);
      Serial.print(' ');
      Serial.print(ServoBase::MaxAngle);
      Serial.print('\n');
    }
    break;

  case Command::LoadEeprom:
    loadEeprom();
    Serial.println(F("OK"));
    break;

  case Command::SaveEeprom:
    saveEeprom();
    Serial.println(F("OK"));
    break;

  case Command::NoOp:
    break;

  case Command::ResetToDefault:
    resetToDefaultValues();
    break;

  default:
    Serial.print(F("ERR: Unknown command "));
    Serial.print(serialBuf[1]);
    Serial.print('\n');
    break;
  }
}

unsigned long calcEepromCrc(int start, int end)
{
  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;
  for (auto index = start; index < end; ++index)
  {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

void initServosFromEeprom()
{
  if (loadEeprom())
    return;

  resetToDefaultValues();
  saveEeprom();
}

void resetToDefaultValues()
{
  numServos = 4;

  const auto pK = 0.00f;
  const auto iK = 0.00f;
  const auto dK = 0.00f;
  const auto dL = 0.1f;

  PidServos[0] = PidServo(
    0, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(0, 0, 1023) // potentiometer pin, in min, in max
  );
  PidServos[1] = PidServo(
    1, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(1, 0, 1023) // potentiometer pin, in min, in max
  );
  PidServos[2] = PidServo(
    2, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(2, 0, 1023) // potentiometer pin, in min, in max
  );
  PidServos[3] = PidServo(
    3, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(3, 0, 1023) // potentiometer pin, in min, in max
  );

  // setPoint => where we want servo to be ([80..100])
  PidServos[0].setPoint(90);
  PidServos[1].setPoint(90);
  PidServos[2].setPoint(90);
  PidServos[3].setPoint(90);

  for (auto i = 0u; i < numServos; ++i)
    PidServos[i].reset();

  PidServo::enabled = false;
  PID::MaxIntegratorStore = 50;
  AnalogPin::Range = 320;
  ServoBase::MinAngle = 80;
  ServoBase::MaxAngle = 100;
}

template <typename T>
void eepromPutInc(unsigned int& addr, const T value)
{
  EEPROM.put(addr, value);
  addr += sizeof(T);
}

template <typename T>
void eepromGetInc(unsigned int& addr, T& value)
{
  EEPROM.get(addr, value);
  addr += sizeof(T);
}


bool loadEeprom()
{
  unsigned long storedCrc;
  EEPROM.get(crcAddress, storedCrc);

  const auto computedCrc = calcEepromCrc(0, crcAddress);

  if (computedCrc != storedCrc)
    return false;

  auto addr = 0u;
  eepromGetInc(addr, numServos);
  eepromGetInc(addr, PidServo::enabled);
  eepromGetInc(addr, PID::MaxIntegratorStore);
  eepromGetInc(addr, AnalogPin::Range);
  eepromGetInc(addr, ServoBase::MinAngle);
  eepromGetInc(addr, ServoBase::MaxAngle);

  for (auto i = 0u; i < numServos; ++i)
  {
    eepromGetInc(addr, PidServos[i]);
    PidServos[i].reset();
  }

  return true;
}

void saveEeprom()
{
  auto addr = 0u;
  eepromPutInc(addr, numServos);
  eepromPutInc(addr, PidServo::enabled);
  eepromPutInc(addr, PID::MaxIntegratorStore);
  eepromPutInc(addr, AnalogPin::Range);
  eepromPutInc(addr, ServoBase::MinAngle);
  eepromPutInc(addr, ServoBase::MaxAngle);

  for (auto i = 0u; i < numServos; ++i)
    eepromPutInc(addr, PidServos[i]);

  // clear remaining memory (write if not zero)
  while (addr < crcAddress)
    EEPROM.update(addr++, 0);

  // calc and store crc
  const auto newCrc = calcEepromCrc(0, crcAddress);
  EEPROM.put(crcAddress, newCrc);
}

#ifdef SERVOPID_TEST
} // end anonymous namespace
#endif