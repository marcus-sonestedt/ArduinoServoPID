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

    PID() {}
    PID(float p, float i, float d, float dLambda) {
        _pFactor = p;
        _iFactor = i;
        _dFactor = d;
        _dLambda = dLambda;

        reset();
    }

    void reset() {
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
        _integral = constrain(_integral + error * dt, -MaxIntegratorStore, MaxIntegratorStore); // limit "energy" storage in integrator

        float output = _pFactor * error + _iFactor * _integral + _dFactor * _deltaFiltered;
        return constrain(output, -MaxOutput, MaxOutput);
    }

private:
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
    AnalogPin() {}
    AnalogPin(int pin, int min, int max)
    {
        _pin = pin;
        _scale = 1.0f / (max - min);
        _bias = -min;
    }

    float read()
    {
        int value = analogRead(_pin);
        return (value + _bias) * _scale;
    }

private:
    int _pin;
    int _bias;
    float _scale;
};

class PidServo
{
public:
    PidServo() {}
    PidServo(int servoPin, int servoMin, int servoMax, PID pid, AnalogPin analogPin, float offset)
    {
        _servo.attach(servoPin, servoMin, servoMax);
        _pid = pid;
        _analogPin = analogPin;
        _offset = offset;
        _setPoint = 0.5f;
    }

    void setPoint(float setpoint)
    {
        _setPoint = setpoint;
    }

    void run(float dt)
    {
        float currentValue = _analogPin.read();
        float input = currentValue + _offset;
        float output = _pid.regulate(input, _setPoint, dt);
        _servo.write(static_cast<int>(output * 180.0f));
    }

private:
    Servo _servo;
    PID _pid;
    AnalogPin _analogPin;

    float _offset;
    float _setPoint;
};

////////////////////////////////////////////////////////////////////

// numbers <-> wheel
#define FL 0
#define FR 1
#define RL 2
#define RR 3
#define NUM_SERVOS 4

PidServo PidServos[NUM_SERVOS];
float prevTime;

void setup() {
    // assume servos on pin 0..3 and potentiometers on 10..13

    // unroll loop to set separate values for each servo
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
        PidServos[i] = PidServo(
            i, 544, 2400, // servo pin, pwm min, pwm max
            PID(3.0f, 1.0f, 0.05f, 0.2f), // p, i, d, d-lowpass
            AnalogPin(i + 10, 0, 1023), // potentiometer pin, in min, in max
            0 // offset between pot and servo [0..1]
        );
    }

    // setPoint => where we want servo to be on relative scale [0..1] (min..max)

    // set front wheels lower than rear
    PidServos[FL].setPoint(0.3f);
    PidServos[FR].setPoint(0.3f);
    PidServos[RL].setPoint(0.5f);
    PidServos[RR].setPoint(0.5f);

    prevTime = micros() * 1e-6f;
}

void loop() {
    // determine time step
    float t = micros() * 1e-6f;
    float dt = t - prevTime;
    prevTime = t;

    // regulate servos
    for (int i = 0; i < NUM_SERVOS; ++i)
        PidServos[i].run(dt);
}
