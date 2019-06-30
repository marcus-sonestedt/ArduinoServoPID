#include <Servo.h>

// adjust to control the amount of "energy" that the PID integrator can store
#define PID_I_MAX_STORE 10

// PID regulator, incl lowpass on D part
class PID
{
  public:
    PID() {}
    PID(float p, float i, float d, float dLambda) {
      _pFactor = p;
      _iFactor = i;
      _dFactor = d;
      _dLambda = dLambda;

      reset();
    }

    void reset() {
      _iSum = 0;
      _prevValue = 0;
      _dFiltered = 0;
    }

    // returns output [0..1]
    float regulate(float currentValue, float requestedValue, float dt)
    {    
      float error = requestedValue - currentValue;          
      float errorD = (currentValue - _prevValue) / dt;      

      _prevValue = currentValue;
      _dFiltered = errorD * _dLambda + (_dFiltered * (1.0 - _dLambda));
      _iSum = constrain(_iSum + error * dt, -PID_I_MAX_STORE, PID_I_MAX_STORE); // limit "energy" storage in integrator

      return _pFactor * error + _iFactor * _iSum + _dFactor * _dFiltered; 
    }

  private:
    float _pFactor;
    float _iFactor;
    float _dFactor;
    float _dLambda;
    
    float _iSum;
    float _dFiltered;
    float _prevValue;
};

// Analog input, with min/max settings
class AnalogPin
{
  public: 
    AnalogPin() {}
    AnalogPin(int pin, int min, int max)
    {
      _pin = pin;
      _scale = 1.0 / (max - min);
      _bias = -min;
    }

    float read()
    {
      int value = analogRead(_pin);
      return (value + _bias) * _scale;
    }

  private:
    int _pin;
    float _bias;
    float _scale;
};

class ServoPid
{
  public:
    ServoPid() {}
    ServoPid(int servoPin, int servoMin, int servoMax, PID pid, AnalogPin analogPin, float offset) 
    {
      _servo.attach(servoPin, servoMin, servoMax);
      _pid = pid;
      _analogPin = analogPin;
      _offset = offset;
      _setPoint = 0.5;
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
      _servo.write(output * 180.0);
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

ServoPid servoPids[NUM_SERVOS];
float prevTime;

void setup() {
  // assume servos on pin 0..3 and potentiometers on 10..13

  // unroll loop to set separate values for each servo
  for(int i=0; i<NUM_SERVOS; ++i) 
  {
    servoPids[i] = ServoPid(
      i, 544, 2400, // servo pin, pwm min, pwm max
      PID(3, 1, 0.05, 0.2), // p, i, d, d-lowpass
      AnalogPin(i+10, 0, 1023), // potentiometer pin, in min, in max
      0 // offset between pot and servo [0..1]
    );     
  }

  // setPoint => where we want servo to be on relative scale [0..1] (min..max)
  
  // set front wheels lower than rear
  servoPids[FL].setPoint(0.3);
  servoPids[FR].setPoint(0.3);
  servoPids[RL].setPoint(0.5);
  servoPids[RR].setPoint(0.5);

  prevTime = micros() * 1e6;
}

void loop() {
  float t = micros() * 1e6;
  float dt = t - prevTime;
  prevTime = t;

  for(int i=0; i<NUM_SERVOS; ++i)
  {
    servoPids[i].run(dt);
  }
}
