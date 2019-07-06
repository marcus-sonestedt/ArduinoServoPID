#ifdef _MSC_VER
#include "ArduinoMock.h"
#else
#include <Servo.h>
#endif

// PID regulator, incl low-pass lambda filter on D part
class PID
{
public:
    // adjust to control the amount of "energy" that the PID integrator can store
    static const int MaxIntegratorStore = 50;
    static const int MaxOutput = 100;

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
        _integral = constrain(_integral + error * dt, -MaxIntegratorStore, MaxIntegratorStore);
        // limit "energy" storage in integrator

        const auto output = _pFactor * error + _iFactor * _integral + _dFactor * _deltaFiltered;
        return constrain(output, -MaxOutput, MaxOutput);
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
        return (value * _scale) + _bias;
    }

    int _pin = 0;
    float _bias = 0;
    float _scale = 1;
};

class PidServo
{
public:
    PidServo() = default;

    PidServo(int servoPin, int servoMin, int servoMax, PID pid, AnalogPin analogPin)
    {
        _servo.attach(servoPin, servoMin, servoMax);
        _pid = pid;
        _analogPin = analogPin;
        _setPoint = 0.5f;
    }

    void setPoint(float setPoint)
    {
        _setPoint = setPoint;
    }

    void run(const float dt)
    {
        _input = _analogPin.read();
        _output = _pid.regulate(_input, _setPoint, dt);
        _servo.write(int(_output * 180.0f));
    }

    Servo _servo;
    PID _pid;
    AnalogPin _analogPin;

    float _setPoint = 0.5f;
    float _input = 0;
    float _output = 0;
};

////////////////////////////////////////////////////////////////////

// numbers <-> wheel
constexpr int NUM_SERVOS = 4;

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
    
    const auto p = 3.0f;
    const auto i = 1.0f;
    const auto d = 0.05f;
    const auto dL = 0.1f;

    FL = PidServo(
        3, 544, 2400, // servo pin, pwm min, pwm max
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

    // setPoint => where we want servo to be on relative scale [0..1] (min..max)
    // set front wheels lower than rear
    FL.setPoint(0.3f);
    FR.setPoint(0.3f);
    RL.setPoint(0.5f);
    RR.setPoint(0.5f);

    // start serials
    Serial.begin(115200);

    // initiate timer
    prevTime = 1e-6f * float(micros());
}

void loop()
{
    // determine time step
    const auto t = 1e-6f * float(micros());
    dt = t - prevTime;
    prevTime = t;

    // regulate servos
    if (enabled)
        for (auto& PidServo : PidServos)
            PidServo.run(dt);

    Serial.print("DT ");
    Serial.println(dt, 3);
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

    const auto cmdLen = serialBuf[serialLen - 1];

    // more data
    if (cmdLen != serialLen)
        return;

    HandleSerialCommand();

    serialLen = 0;
}

void HandleSerialCommand()
{
    switch (Command(serialBuf[1]))
    {
        // len, cmd, pid#, param-id, float-value[4]
    case Command::SetServoParamFloat:
        {
            if (serialBuf[2] >= NUM_SERVOS)
            {
                Serial.print(F("ERR: Invalid servo number "));
                Serial.println(serialBuf[2]);
                return;
            }

            auto& servoPid = PidServos[serialBuf[2]];
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
                Serial.println(serialBuf[3]);
                return;
            }
        }
        Serial.println(F("OK"));
        break;

    case Command::EnableRegulator:
        enabled = serialBuf[1] != 0;
        break;

    case Command::GetNumServos:
        Serial.print(F("NS "));
        Serial.print(NUM_SERVOS);
        break;

    case Command::GetServoParams:
        for (auto i = 0; i < NUM_SERVOS; ++i)  // NOLINT(modernize-loop-convert)
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
            Serial.println(servo._pid._deltaFiltered);
        }
        break;

    default:
        Serial.print(F("ERR: Unknown command "));
        Serial.println(serialBuf[1]);
        break;
    }
}
