#ifdef _MSC_VER
#include "ArduinoMock.h"
#else
#include <Servo.h>
#endif

// PID regulator, incl lowpass on D part
class PID
{
public:
    // adjust to control the amount of "energy" that the PID integrator can store
    static const int MaxIntegratorStore = 50;
    static const int MaxOutput = 100;

    PID()
    {
    }

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
        float error = requestedValue - currentValue;
        float errorDelta = (error - _prevError) / dt;

        _prevError = error;
        _deltaFiltered = (errorDelta * _dLambda) + (_deltaFiltered * (1.0f - _dLambda));
        _integral = constrain(_integral + error * dt, -MaxIntegratorStore, MaxIntegratorStore);
        // limit "energy" storage in integrator

        float output = _pFactor * error + _iFactor * _integral + _dFactor * _deltaFiltered;
        return constrain(output, -MaxOutput, MaxOutput);
    }

    float _pFactor;
    float _iFactor;
    float _dFactor;
    float _dLambda;

    float _integral;
    float _deltaFiltered;
    float _prevError;
};

// Analog input, with min/max settings
class AnalogPin
{
public:
    AnalogPin()
    {
    }

    AnalogPin(int pin, int min, int max)
    {
        _pin = pin;
        _scale = 1.0f / (max - min);
        _bias = static_cast<float>(-min);
    }

    float read()
    {
        int value = analogRead(_pin);
        return (value * _scale) + _bias;
    }

    int _pin;
    float _bias;
    float _scale;
};

class PidServo
{
public:
    PidServo()
    {
    }

    PidServo(int servoPin, int servoMin, int servoMax, PID pid, AnalogPin analogPin)
    {
        _servo.attach(servoPin, servoMin, servoMax);
        _pid = pid;
        _analogPin = analogPin;
        _setPoint = 0.5f;
    }

    void setPoint(float setpoint)
    {
        _setPoint = setpoint;
    }

    void run(float dt)
    {
        _input = _analogPin.read();
        _output = _pid.regulate(_input, _setPoint, dt);
        _servo.write(static_cast<int>(_output * 180.0f));
    }

    Servo _servo;
    PID _pid;
    AnalogPin _analogPin;

    float _setPoint;
    float _input;
    float _output;
};

////////////////////////////////////////////////////////////////////

// numbers <-> wheel
#define NUM_SERVOS 4

PidServo PidServos[NUM_SERVOS];

constexpr PidServo& FL = PidServos[0];
constexpr PidServo& FR = PidServos[1];
constexpr PidServo& RL = PidServos[2];
constexpr PidServo& RR = PidServos[3];

float prevTime = 0;
float dt = 0;
bool enabled = true;

void setup() {
    // assume servos on pin 3,5,6,9 and potentiometers on analog in 0,1,2,3
    
    float p = 3.0f;
    float i = 1.0f;
    float d = 0.05f;
    float dL = 0.1f;

    FL = PidServo(
        3, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL), // p, i, d, d-lowpass
        AnalogPin(0, 0, 1023) // potentiometer pin, in min, in max
    );
    FR = PidServo(
        5, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL), // p, i, d, d-lowpass
        AnalogPin(1, 0, 1023) // potentiometer pin, in min, in max
    );
    RL = PidServo(
        6, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL), // p, i, d, d-lowpass
        AnalogPin(2, 0, 1023) // potentiometer pin, in min, in max
    );
    RR = PidServo(
        9, 544, 2400, // servo pin, pwm min, pwm max
        PID(p, i, d, dL), // p, i, d, d-lowpass
        AnalogPin(3, 0, 1023) // potentiometer pin, in min, in max
    );

    // setPoint => where we want servo to be on relative scale [0..1] (min..max)
    // set front wheels lower than rear
    FL.setPoint(0.3f);
    FR.setPoint(0.3f);
    RL.setPoint(0.5f);
    RR.setPoint(0.5f);

    // start serials
    Serial.begin(115200);

    // initiate timer
    prevTime = micros() * 1e-6f;
}

void loop()
{
    // determine time step
    float t = micros() * 1e-6f;
    dt = t - prevTime;
    prevTime = t;

    // regulate servos
    if (enabled)
        for (int i = 0; i < NUM_SERVOS; ++i)
            PidServos[i].run(dt);

    Serial.println(dt, 3);
}

enum class Command
{
    NoOp,
    SetServoParamFloat,
    EnableRegulator,
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

char serialBuf[16] = {0};
int serialLen = 0;

void HandleSerialCommand();

void serialEvent()
{
    while (Serial.available() > 0 && serialLen < sizeof(serialBuf))
        serialBuf[serialLen++] = Serial.read();

    if (serialLen == 0)
        return;

    if (serialLen >= sizeof(serialBuf))
    {
        Serial.println(F("ERR: Command buffer overflow"));
        serialLen = 0;
        return;
    }

    auto cmdLen = serialBuf[serialLen - 1];

    // more data
    if (cmdLen != serialLen)
        return;

    HandleSerialCommand();
    serialLen = 0;
}

void HandleSerialCommand()
{
    switch ((Command)serialBuf[1])
    {
        // len, cmd, pid#, param-id, float-value[4]
    case Command::SetServoParamFloat:
        {
            if (serialBuf[2] >= NUM_SERVOS)
            {
                Serial.print(F("ERR: Invalid servo "));
                Serial.println(serialBuf[2]);
                return;
            }

            auto& servoPid = PidServos[serialBuf[2]];
            float value = *reinterpret_cast<float*>(serialBuf + 4);
            switch ((ServoParam)serialBuf[3])
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
                Serial.println(serialBuf[3]);
                return;
            }
        }
        Serial.println(F("OK"));
        break;

    case Command::EnableRegulator:
        enabled = serialBuf[1] != 0;
        break;

    case Command::GetServoParams:
        for (auto i = 0; i < NUM_SERVOS; ++i)
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
            Serial.println('\n');
        }
        break;

    case Command::GetServoData:
        for (auto i = 0; i < NUM_SERVOS; ++i)
        {
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
            Serial.println(servo._pid._deltaFiltered);
        }
        break;

    default:
        Serial.print(F("ERR: Unknown command "));
        Serial.println(serialBuf[1]);
        break;
    }
}
