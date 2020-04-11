#define USE_PCA9685 0 // set 0 to use Arduino to directly control servos

#ifdef ARDUINO
#include <EEPROM.h> 
#if USE_PCA9685 == 1
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#else
#include <Servo.h>
#endif
#else
#include <cstdint>
#include <corecrt_math.h>
#include "../ArduinoMock/ArduinoMock.h"
#include "../ArduinoMock/AdafruitPwmServoDriverMock.h"
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
    const auto error = _setPoint - currentValue;

    _integral = 0.0f;

    if (_iFactor != 0)
      _integral = error / _iFactor;
    else
      _integral = 0.0f;

    if (_pFactor != 0)
      _integral -= error / _pFactor;

    _prevValue = currentValue;
    _dValueFiltered = 0.0f;
  }

  float regulate(float currentValue, float dt, float minValue, float maxValue)
  {
    constexpr auto offset = 90.0f;

    const auto error = _setPoint - currentValue;
    const auto dValue = (currentValue - _prevValue) / dt;

    _prevValue = currentValue;
    _dValueFiltered = (dValue * _dLambda) + (_dValueFiltered * (1.0f - _dLambda));

    const auto outputEstimate = _pFactor * error + _iFactor * _integral - _dFactor * _dValueFiltered + offset;

    if (_iFactor != 0) {
      // only integrate if we are within bounds
      const auto i = error * dt;
      if (i * _iFactor > 0 && outputEstimate < maxValue)
        _integral += i;
      else if (i * _iFactor < 0 && outputEstimate > minValue)
        _integral += i;

      // limit "energy" storage in integrator
      _integral = constrain(_integral, -MaxIntegratorStore, MaxIntegratorStore);
    }

    return _pFactor * error + _iFactor * _integral - _dFactor * _dValueFiltered + offset;
  }

  void setPoint(float setPoint)
  {
    _integral = 0.0f;
    _setPoint = setPoint;
  }

  float _setPoint = 0;

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
  AnalogPin() = default;

  AnalogPin(uint8_t pin, uint16_t min, uint16_t max)
  {
    _pin = pin;
    _min = min;
    _max = max;
  }

  float read() const
  {
    const auto value = analogRead(_pin);
    const auto angle = map(int(value), 0, 1023, _min, _max);
    return angle;
  }

  float   _min = 0;
  float   _max = 320;
  uint8_t _pin = 0;
};

class Deadband
{
public:
  static int16_t MaxDeviation;

  int _output;

  int apply(int input)
  {
    if (abs(input - _output) > MaxDeviation)
      _output = input;

    return _output;
  }
};

int16_t Deadband::MaxDeviation = 10;

class ServoBase
{
public:
  static uint16_t MinAngle;
  static uint16_t MaxAngle;

  static uint16_t MinAngleRange;
  static uint16_t MaxAngleRange;

  static float servoMidpointAngle()
  {
    return (MaxAngle + MinAngle) / 2.0f;
  };

  int angleToPwm(float angle)
  {
    const auto cAngle = constrain(angle, float(MinAngle), float(MaxAngle));
    const auto pwm = map(cAngle, float(MinAngleRange), float(MaxAngleRange), _pwmMin, _pwmMax);
    const auto cPwm = constrain(pwm, _pwmMin, _pwmMax);
    const auto dPwm = _deadband.apply(cPwm);

    return dPwm;
  }

  uint8_t  _pin = 0;
  uint16_t _pwmMin = 544;
  uint16_t _pwmMax = 2400;
  Deadband _deadband = {};
};

uint16_t ServoBase::MinAngle = 80;
uint16_t ServoBase::MaxAngle = 100;

uint16_t ServoBase::MinAngleRange = 0;
uint16_t ServoBase::MaxAngleRange = 320;


#if USE_PCA9685 == 1

Adafruit_PWMServoDriver gPwmController;

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
    write(servoMidpointAngle()); 
  }

  // ReSharper disable once CppMemberFunctionMayBeConst
  void write(const float angle)
  {
    const auto dPwm = angleToPwm(angle);

    gPwmController.setPWM(_pin, 0, uint16_t(dPwm));
  }

};

#else

class ArduinoServo : public Servo, public ServoBase
{
public:

    void attach(const uint8_t pin, const uint16_t pwmMin, const uint16_t pwmMax)
    {
        _pin = pin;
        _pwmMin = pwmMin;
        _pwmMax = pwmMax;
        reset();
    }

    void write(float angle)
    {
        const auto dPwm = angleToPwm(angle);
        Servo::write(dPwm);
    }

    void reset()
    {
        Servo::attach(_pin, _pwmMin, _pwmMax);
        write(servoMidpointAngle());
    }

    uint8_t _pin = 0;
    uint16_t _pwmMin = 544;
    uint16_t _pwmMax = 2400;
};

#endif



class PidServo
{
public:
  static bool enabled;

  PidServo() = default;

  PidServo(uint8_t servoPin, uint16_t servoMin, uint16_t servoMax, PID pid, AnalogPin analogPin)
  {
    _servo.attach(servoPin, servoMin, servoMax);
    _pid = pid;
    _analogPin = analogPin;
  }

  void setPoint(float value)
  {
    _pid.setPoint(value);
  }

  void run(const float dt)
  {
    _input = _analogPin.read();

    if (enabled) {
      _output = _pid.regulate(_input, dt, _servo.MinAngle, _servo.MaxAngle);
    } else  {
      _output = ServoBase::servoMidpointAngle();
    }
    _servo.write(_output);
  }

  void reset()
  {
    _input = _analogPin.read();
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

  float _input = 0;
  float _output = 0;
};

bool PidServo::enabled = false;

////////////////////////////////////////////////////////////////////

constexpr int MAX_PID_SERVOS = 8;
int  numServos = 1;
PidServo      PidServos[MAX_PID_SERVOS];

float prevTime = 0;
float dt = 0;

void initServosFromEeprom();

int crcAddress;

void setup()
{
  crcAddress = EEPROM.length() - sizeof(unsigned long);

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
  delay(20);
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

  // blink LED
  const auto freq = PidServo::enabled ? 5 : 1;
  const auto ledPwm = uint8_t((sinf(t * 3.14f * freq) + 1) * 127);
  analogWrite(13, ledPwm);

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

enum class Command : uint8_t
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
  CalibrateAnalogInput
};

enum class ServoParam : uint8_t
{
  None,
  P,
  I,
  D,
  DLambda,
  SetPoint,
  InputMin,
  InputMax,
};

enum class GlobalVar : uint8_t
{
  NumServos,
  PidEnabled,
  PidMaxIntegratorStore,
  AnalogInputRange,
  ServoMinAngle,
  ServoMaxAngle,
  DeadbandMaxDeviation
};

unsigned char serialBuf[128] = {0};
unsigned char serialLen = 0;

void handleSerialCommand();
bool loadEeprom();
void saveEeprom();
void resetToDefaultValues();
void calibrateAnalogInputs();

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

    if (serialBuf[0] == serialLen)
    {
      handleSerialCommand();
      serialLen = 0;
    }
  }
}

union FloatAsBytes {
  float floatValue;
  unsigned char byteValue[4];
};

float floatFromBytes(const unsigned char* buf)
{
  FloatAsBytes fab{};
  for (auto i = 0; i < 4; ++i)
    fab.byteValue[i] = buf[i];
  return fab.floatValue;
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
      const auto value = floatFromBytes(serialBuf + 4);
      //Serial.println(value);

      switch (ServoParam(serialBuf[3]))
      {
      case ServoParam::P: 
        servoPid._pid._pFactor = value;
        break;

      case ServoParam::I: 
        servoPid._pid._iFactor = value;
        servoPid.reset();
        break;

      case ServoParam::D: 
        servoPid._pid._dFactor = value;
        break;

      case ServoParam::DLambda:
        servoPid._pid._dLambda = value;
        break;

      case ServoParam::SetPoint:
        servoPid.setPoint(value);
        break;

      case ServoParam::InputMin:
        servoPid._analogPin._min = int16_t(value);
        break;

      case ServoParam::InputMax:
        servoPid._analogPin._max = int16_t(value);
        break;

      default:
        Serial.print(F("ERR: Unknown servo parameter "));
        Serial.print(int(serialBuf[3]));
        Serial.print('\n');
        return;
      }
    }

    Serial.println(F("OK"));
    break;

  case Command::EnableRegulator:
    Serial.println(F("ERR: Deprecated command: EnableRegulator"));
    break;

  case Command::GetNumServos:
    Serial.print(F("NS "));
    Serial.print(numServos);
    Serial.print('\n');
    break;

  case Command::GetServoParams:
    for (auto i = 0; i < numServos; ++i) // NOLINT(modernize-loop-convert)
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
      Serial.print(servo._pid._setPoint);
      Serial.print('\n');
    }
    break;

  case Command::GetServoData:
    for (auto i = 0; i < numServos; ++i)
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
      const auto value = floatFromBytes(serialBuf + 3);
      //Serial.println(value);

      switch (var)
      {
      case GlobalVar::NumServos:
        if (int(value) >= MAX_PID_SERVOS)
        {
          Serial.print(F("ERR: Max supported num servos is: "));
          Serial.println(MAX_PID_SERVOS);
          return;
        }

        numServos = int(value);
        for (auto i = 0; i < numServos; ++i)
        {
          PidServos[i] = PidServo();
          PidServos[i].reset();
        }
        break;

      case GlobalVar::PidEnabled:
        PidServo::enabled = value != 0;
        for (auto i = 0; i < numServos; ++i) // NOLINT(modernize-loop-convert)
        {
          auto& servo = PidServos[i];
          servo.reset();
        }
        break;

      case GlobalVar::PidMaxIntegratorStore:
        PID::MaxIntegratorStore = uint16_t(value);
        break;

      case GlobalVar::AnalogInputRange:
        Serial.println(F("ERR: AnalogInputRange - deprecated global var"));
        //AnalogPin::Range = uint16_t(value);
        break;

      case GlobalVar::ServoMinAngle:
        ServoBase::MinAngle = uint16_t(value);
        break;

      case GlobalVar::ServoMaxAngle:
        ServoBase::MaxAngle = uint16_t(value);
        break;

      case GlobalVar::DeadbandMaxDeviation:
        Deadband::MaxDeviation = uint16_t(value);
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
      Serial.print(0);
      Serial.print(' ');
      Serial.print(ServoBase::MinAngle);
      Serial.print(' ');
      Serial.print(ServoBase::MaxAngle);
      Serial.print('\n');
      Serial.print(Deadband::MaxDeviation);
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

  case Command::CalibrateAnalogInput:
    calibrateAnalogInputs();
    break;


    default:
    Serial.print(F("ERR: Unknown command "));
    Serial.print(serialBuf[1]);
    Serial.print('\n');
    break;
  }
}

constexpr unsigned long crc_table[16] = {
   0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
   0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
   0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
   0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

unsigned long calcEepromCrc(int start, int end)
{ 
  unsigned long crc = 0;
  crc = ~crc;

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
  {
    Serial.println(F("LOG: Loaded PIDs from EEPROM"));
    return;
  }

  resetToDefaultValues();
  saveEeprom();
}

void resetToDefaultValues()
{
  Serial.println(F("LOG: Resetting PIDs to default values"));

  numServos = 1;

  const auto pK = 0.00f;
  const auto iK = 0.00f;
  const auto dK = 0.00f;
  const auto dL = 0.1f;

  PidServos[0] = PidServo(
    9, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(0, 0, 320) // potentiometer pin, in min, in max
  );
  PidServos[1] = PidServo(
    1, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(1, 0, 320) // potentiometer pin, in min, in max
  );
  PidServos[2] = PidServo(
    2, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(2, 0, 320) // potentiometer pin, in min, in max
  );
  PidServos[3] = PidServo(
    3, 544, 2400, // servo pin, pwm min, pwm max
    PID(pK, iK, dK, dL),
    AnalogPin(3, 0, 320) // potentiometer pin, in min, in max
  );

  pinMode(2, OUTPUT);

  // setPoint => where we want servo to be ([80..100])
  PidServos[0].setPoint(ServoBase::servoMidpointAngle());
  PidServos[1].setPoint(ServoBase::servoMidpointAngle());
  PidServos[2].setPoint(ServoBase::servoMidpointAngle());
  PidServos[3].setPoint(ServoBase::servoMidpointAngle());

  for (auto i = 0; i < numServos; ++i)
    PidServos[i].reset();

  PidServo::enabled = false;
  PID::MaxIntegratorStore = 5000;
  //AnalogPin::Range = 320;
  ServoBase::MinAngle = 70;
  ServoBase::MaxAngle = 110;
  Deadband::MaxDeviation = 5;
}

template <typename T>
void eepromPutInc(int& addr, const T value)
{
  EEPROM.put(addr, value);
  addr += sizeof(T);
}

template <typename T>
void eepromGetInc(int& addr, T& value)
{
  EEPROM.get(addr, value);
  addr += sizeof(T);
}


bool loadEeprom()
{
  Serial.println(F("LOG: Loading settings from EEPROM"));

  unsigned long storedCrc;
  EEPROM.get(crcAddress, storedCrc);

  const auto computedCrc = calcEepromCrc(0, crcAddress);

  if (computedCrc != storedCrc) {
    Serial.print(F("ERR: CRC mismatch, stored: "));
    Serial.print(storedCrc);
    Serial.print(F(" vs computed: "));
    Serial.print(storedCrc);
    Serial.println(' ');
    return false;
  }

  auto addr = 0;
  eepromGetInc(addr, numServos);
  eepromGetInc(addr, PidServo::enabled);
  eepromGetInc(addr, PID::MaxIntegratorStore);
  //eepromGetInc(addr, AnalogPin::Range);
  eepromGetInc(addr, ServoBase::MinAngle);
  eepromGetInc(addr, ServoBase::MaxAngle);
  eepromGetInc(addr, Deadband::MaxDeviation);

  Serial.print(F("LOG: CRC match. Initializing "));
  Serial.print(numServos);
  Serial.println(F(" servo(s)."));

  for (auto i = 0; i < numServos; ++i)
  {
    eepromGetInc(addr, PidServos[i]);
    PidServos[i].reset();
  }

  return true;
}

void saveEeprom()
{
  Serial.println(F("LOG: Saving settings to EEPROM"));

  auto addr = 0;
  eepromPutInc(addr, numServos);
  eepromPutInc(addr, PidServo::enabled);
  eepromPutInc(addr, PID::MaxIntegratorStore);
  //eepromPutInc(addr, AnalogPin::Range);
  eepromPutInc(addr, ServoBase::MinAngle);
  eepromPutInc(addr, ServoBase::MaxAngle);
  eepromPutInc(addr, Deadband::MaxDeviation);

  for (auto i = 0; i < numServos; ++i)
    eepromPutInc(addr, PidServos[i]);

  // clear remaining memory (write if not zero)
  while (addr < crcAddress)
    EEPROM.update(addr++, 0);

  // calc and store crc
  const auto newCrc = calcEepromCrc(0, crcAddress);
  EEPROM.put(crcAddress, newCrc);

  Serial.print(F("LOG: Wrote "));
  Serial.print(addr);
  Serial.print(F(" bytes. CRC: "));
  Serial.println(newCrc);
}

void setServosAndPrintInput(float angle)
{
  Serial.print(F("LOG: Moving servos to "));
  Serial.println(angle);

  for (auto i = 0; i < numServos; ++i)
    PidServos[i]._servo.write(angle);

  delay(500);

  for (auto i = 0; i < numServos; ++i) {
    Serial.print(F("Servo #"));
    Serial.print(i);
    Serial.print(F(" at "));
    Serial.print(PidServos[i]._analogPin.read(), 3);
    Serial.println(F(" degrees"));
  }
}

void calibrateAnalogInputs()
{
  Serial.println(F("LOG: Performing calibration maneuver!"));

  setServosAndPrintInput(ServoBase::MinAngle);
  setServosAndPrintInput(ServoBase::MaxAngle);

  for (auto i = 0; i < numServos; ++i) {
    PidServos[i]._servo.write(ServoBase::servoMidpointAngle());
    PidServos[i].reset();
  }

  delay(500);

  Serial.println(F("LOG: Done!"));
}

#ifdef SERVOPID_TEST
} // end anonymous namespace
#endif
