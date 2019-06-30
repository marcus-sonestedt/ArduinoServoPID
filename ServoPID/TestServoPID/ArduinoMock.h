#pragma once
#include <vector>

// Mock globals

extern int gMicros; 
inline int micros() { return gMicros; }
inline void setMockMicros(const int value) { gMicros = value;}

extern std::vector<int> gAnalogPins;
inline int analogRead(const int pin) { return gAnalogPins.at(pin);}
inline void setMockAnalogRead(const int pin, const int value)
{
    gAnalogPins.at(pin) = value;
}

inline float constrain(const float v, const float min, const float max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

// Mock Servo

class Servo
{
public:
    void attach(const int pin, const int min, const int max)
    {
        _pin = pin;
        _min = min;
        _max = max;
    }

    void write(const int angle) { _angle = angle; }

    int angle() const { return _angle; }

private:
    int _pin = 0;
    int _min = 0;
    int _max = 0;

    int _angle = 0;
};
