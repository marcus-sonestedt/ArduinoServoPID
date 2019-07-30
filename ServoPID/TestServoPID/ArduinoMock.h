#pragma once

// ReSharper disable CppInconsistentNaming
// ReSharper disable CppMemberFunctionMayBeStatic
// ReSharper disable CppParameterNeverUsed

constexpr const char* F(const char* x) { return x; }

// Mock globals

namespace mock {

extern unsigned long gMicros;
extern std::vector<int> gAnalogPins;

}
inline unsigned long micros() { return mock::gMicros; }
inline void setMockMicros(const unsigned long value) { mock::gMicros = value;}

inline int analogRead(const int pin) { return mock::gAnalogPins.at(pin);}
inline void setMockAnalogRead(const int pin, const int value)
{
    mock::gAnalogPins.at(pin) = value;
}

template<typename T, typename U>
T constrain(const T v, const U min, const U max)
{
    if (v < T(min))
        return T(min);
    if (v > T(max))
        return T(max);
    return v;
}

// Mock Servo

class MockServo
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

typedef MockServo Servo;


class MockSerial
{
public:
    void begin(int baudRate) const {}
    void end() const {}

    int available() const { return dataRead.size(); }

    char read()
    {
        if (dataRead.empty())
            return 0;
        
        const auto value = dataRead.back();
        dataRead.pop_back();
        return value;
    }

    void setMockData(const std::string& str)
    {
        dataRead.assign(str.rbegin(), str.rend());
    }

    void setMockData(const std::vector<char>& data)
    {
        dataRead.assign(data.rbegin(), data.rend());
    }

    template<typename T>
    void println(T value, int format = 3) { dataWrite << value; }

    template<typename T>
    void print(T value, int format = 3) { dataWrite << value; }

    void flush() {}

    void resetMock()
    {
        dataRead.clear();
        dataWrite.clear();
        dataWrite.str({});
    }

    std::vector<char> dataRead;
    std::stringstream dataWrite;
};

extern MockSerial Serial;

class MockWire
{
public:
    void begin() const {}
    void setClock(const int baudRate) const {}
};

extern MockWire Wire;

// ReSharper restore CppInconsistentNaming
// ReSharper restore CppMemberFunctionMayBeStatic
// ReSharper restore CppParameterNeverUsed
