#define USE_PCA9685 0 // set 0 to use Arduino to directly control servos

#ifndef ARDUINO
#include "ArduinoMock.h"

namespace
{
#if USE_PCA9685 == 1
    #include "PCA9685Mock.h"
#endif

#else
#if USE_PCA9685 == 1
     #include <Wire.h>
     #include <PCA9685.h>
#else
     #include <Servo.h>
#endif
#endif

// PID regulator, incl low-pass lambda filter on D part
class PID
{
public:
    // adjust to control the amount of "energy" that the PID integrator can store
    static const int MaxIntegratorStore = 50;

    PID() = default;

    PID(float p, float i, float d, float dLambda)
    {
        _pFactor = p;
        _iFactor = i;
        _dFactor = d;
        _dLambda = dLambda;

        reset();
    }

    void reset()
    {
        _integral = 0.0f;
        _prevError = 0.0f;
        _deltaFiltered = 0.0f;
    }

    float regulate(float currentValue, float requestedValue, float dt)
    {
        const auto error = requestedValue - currentValue;
        const auto errorDelta = (error - _prevError) / dt;

        _prevError = error;
        _deltaFiltered = (errorDelta * _dLambda) + (_deltaFiltered * (1.0f - _dLambda));
        _integral = constrain(_integral + error * dt, float(-MaxIntegratorStore), float(MaxIntegratorStore));
        // limit "energy" storage in integrator

        return _pFactor * error + _iFactor * _integral + _dFactor * _deltaFiltered;
    }

    float _pFactor = 0;
    float _iFactor = 0;
    float _dFactor = 0;
    float _dLambda = 0;

    float _integral = 0;
    float _deltaFiltered = 0;
    float _prevError = 0;
};

// Analog input, with min/max settings
class AnalogPin
{
public:
    static constexpr float Range = 320.0f;

    AnalogPin() = default;

    AnalogPin(int pin, int min, int max)
    {
        _pin = pin;
        _scale = 1.0f / float(max - min);
        _bias = static_cast<float>(-min);
    }

    float read() const
    {
        const auto value = float(analogRead(_pin));
        return ((value * _scale) + _bias) * Range;
    }

    int   _pin = 0;
    float _bias = 0;
    float _scale = 1;
};

class ServoBase
{
public:
    static const int MinAngle = 80;
    static const int MaxAngle = 100;
};

#if USE_PCA9685 == 1
PCA9685 gPwmController;

class PCA9685Servo : public ServoBase
{
public:
    PCA9685Servo() = default;

    void attach(const int pin, const int pwmMin, const int pwmMax)
    {
        _pin = pin;
        _servoEval = PCA9685_ServoEvaluator(pwmMin, pwmMax);
    }

    // ReSharper disable once CppMemberFunctionMayBeConst
    void write(const float angle)
    {
        const auto cAngle = constrain(angle, float(MinAngle), float(MaxAngle));
        const auto pwm = _servoEval.pwmForAngle(cAngle);
        gPwmController.setChannelPWM(_pin, pwm);
    }

private:
    uint8_t _pin = 0;
    PCA9685_ServoEvaluator _servoEval;
};
#else

class ArduinoServo : public Servo, public ServoBase
{
public:
    void write(float angle)
    {
        Servo::write(constrain(int(angle), MinAngle, MaxAngle));
    }
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

constexpr int NUM_SERVOS = 4;
PidServo      PidServos[NUM_SERVOS];

constexpr PidServo& FL = PidServos[0];
constexpr PidServo& FR = PidServos[1];
constexpr PidServo& RL = PidServos[2];
constexpr PidServo& RR = PidServos[3];

float prevTime = 0;
float dt = 0;

void setup()
{
#if USE_PCA9685
    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    gPwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    gPwmController.init(21);              // Address pins A5-A0 set to B010101
    gPwmController.setPWMFrequency(500); // Default is 200Hz, supports 24Hz to 1526Hz
#endif

    // assume servos on pin 3,5,6,9 and potentiometers on analog in 0,1,2,3

    const auto p = 0.00f;
    const auto i = 0.00f;
    const auto d = 0.00f;
    const auto dL = 0.1f;

    FL = PidServo(
        10, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL),
        AnalogPin(0, 0, 1023) // potentiometer pin, in min, in max
    );
    FR = PidServo(
        5, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL),
        AnalogPin(1, 0, 1023) // potentiometer pin, in min, in max
    );
    RL = PidServo(
        6, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL),
        AnalogPin(2, 0, 1023) // potentiometer pin, in min, in max
    );
    RR = PidServo(
        9, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL),
        AnalogPin(3, 0, 1023) // potentiometer pin, in min, in max
    );

    // setPoint => where we want servo to be ([80..100])
    FL.setPoint(90);
    FR.setPoint(90);
    RL.setPoint(90);
    RR.setPoint(90);

    // start serials
    Serial.begin(115200);

    // empty input buffer
    while (Serial.available())
        Serial.read();

    // initiate timer
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
    NoOp,
    SetServoParamFloat,
    EnableRegulator,
    GetNumServos,
    GetServoParams,
    GetServoData
};

enum class ServoParam
{
    P,
    I,
    D,
    DLambda,
    SetPoint,
    InputScale,
    InputBias
};

char         serialBuf[128] = {0};
unsigned int serialLen = 0;

void handleSerialCommand();

void mySerialEvent()
{
    while (Serial.available() > 0 && serialLen < sizeof serialBuf)
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

        if (serialLen >= sizeof serialBuf)
        {
            Serial.print(F("ERR: Command buffer overflow\n"));
            serialLen = 0;
            continue;
        }

        if (serialBuf[0] == char(serialLen))
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
            if (serialBuf[2] >= NUM_SERVOS)
            {
                Serial.print(F("ERR: Invalid servo number "));
                Serial.print(int(serialBuf[2]));
                Serial.print('\n');
                return;
            }

            auto&      servoPid = PidServos[int(serialBuf[2])];
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
            case ServoParam::InputBias: servoPid._analogPin._bias = value;
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
        Serial.print(NUM_SERVOS);
        Serial.print('\n');
        break;

    case Command::GetServoParams:
        for (auto i = 0; i < NUM_SERVOS; ++i) // NOLINT(modernize-loop-convert)
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
        for (auto i = 0; i < NUM_SERVOS; ++i)
        {
            // show only one servo if set
            if (serialBuf[2] != i && serialBuf[2] < NUM_SERVOS)
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
            Serial.print(servo._pid._deltaFiltered);
            Serial.print('\n');
        }
        break;

    default:
        Serial.print(F("ERR: Unknown command "));
        Serial.print(serialBuf[1]);
        Serial.print('\n');
        break;
    }
}


#ifndef ARDUINO
} // end anonymous namespace
#endif
