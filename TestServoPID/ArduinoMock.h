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
inline void setMockMicros(const unsigned long value) { mock::gMicros = value; }

inline void delay(int ms) {};

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


class MockEEPROM
{
public:
    MockEEPROM()
    {
        _mem.fill(0);
    }

    uint8_t read(int addr) { return _mem[addr];}

    void write(int addr, uint8_t value) { _mem[addr] = value; }

    template<typename T>
    void put(int addr, T value) { *reinterpret_cast<T*>(_mem[addr]) = t; }

    template<typename T>
    void get(int addr, T& value) { value = *reinterpret_cast<T*>(_mem[addr]); }

    uint8_t& operator[](int addr) { return _mem[addr]; }

    std::array<uint8_t, 4096> _mem;
};

MockEEPROM EEPROM;

// ReSharper restore CppInconsistentNaming
// ReSharper restore CppMemberFunctionMayBeStatic
// ReSharper restore CppParameterNeverUsed
