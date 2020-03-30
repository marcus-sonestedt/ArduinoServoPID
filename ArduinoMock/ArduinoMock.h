#pragma once

// ReSharper disable CppInconsistentNaming
// ReSharper disable CppMemberFunctionMayBeStatic
// ReSharper disable CppParameterNeverUsed

constexpr const char* F(const char* x) { return x; }

// whatever
#define OUTPUT 1
#define INPUT 2
#define INPUT_PULLUP 3

// Mock globals
namespace mock
{
MOCK_API extern uint32_t              gMicros;
MOCK_API extern std::vector<uint16_t> gAnalogPins;
MOCK_API extern std::vector<uint8_t> gPwmPins;
}

inline void pinMode(int pin, int mode)
{
  // bah
}

inline void analogWrite(int pin, uint8_t pwm)
{
  mock::gPwmPins.at(pin) = pwm;
}

inline uint32_t micros()
{
  return mock::gMicros;
}

inline void setMockMicros(const uint32_t value)
{
  mock::gMicros = value;
}

inline void delay(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
};

inline uint16_t analogRead(const int pin)
{
  return mock::gAnalogPins.at(pin);
}

inline void setMockAnalogRead(const int pin, const uint16_t value)
{
  mock::gAnalogPins.at(pin) = value;
}

template <typename T, typename U>
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
  void begin(int baudRate) const
  {
  }

  void end() const
  {
  }

  int available() const
  {
    return int(_dataToArduino.size());
  }

  unsigned char read()
  {
    if (_dataToArduino.empty())
      return 0;

    const auto value = _dataToArduino.front();
    _dataToArduino.pop();
    return value;
  }

  void setMockData(const std::string& str)
  {
    while (!_dataToArduino.empty())
      _dataToArduino.pop();

    for (const auto c : str)
      _dataToArduino.push(c);
  }

  void setMockData(const std::vector<char>& data)
  {
    while (!_dataToArduino.empty())
      _dataToArduino.pop();

    for (const auto c : data)
      _dataToArduino.push(c);
  }

  template <typename T>
  void println(T value, int format = 3)
  {
    print(value, format, false);
    _dataFromArduino.push('\n');
    notify();
  }

  template <typename T>
  void print(T value, int format = 3, bool callNotify = true)
  {
    std::stringstream ss;
    ss.precision(format);
    ss << value;

    for (const auto c : ss.str())
      _dataFromArduino.push(c);

    if (callNotify)
      notify();
  }

  void flush()
  {
  }

  void resetMock()
  {
    while (!_dataToArduino.empty())
      _dataToArduino.pop();

    while (!_dataFromArduino.empty())
      _dataFromArduino.pop();
  }

  void setCallback(void (*callback)())
  {
    _callback = callback;
  }
  
  void writeMock(const unsigned char* str, const int32_t strLen)
  {
    for (auto p = str; p < str + strLen; p++)
      _dataToArduino.push(*p);
  }

  std::string readMock()
  {
    std::string str;
    while (!_dataFromArduino.empty())
    {
      const auto value = _dataFromArduino.front();
      _dataFromArduino.pop();
      str.push_back(value);
    }

    return str;
  }

  std::string readMockLine()
  {
    std::string str;

    while (!_dataFromArduino.empty())
    {
      const auto value = _dataFromArduino.front();
      _dataFromArduino.pop();
      if (value == '\n')
        break;
      str.push_back(value);
    }

    return str;
  }

private:
  void notify() const
  {
    if (_callback)
      _callback();
  }

  std::queue<unsigned char> _dataToArduino;
  std::queue<unsigned char> _dataFromArduino;
  void (*          _callback)() = nullptr;
};

MOCK_API extern MockSerial Serial;

class MockWire
{
public:
  void begin() const
  {
  }

  void setClock(const int baudRate) const
  {
  }
};

MOCK_API extern MockWire Wire;


class MockEEPROM
{
public:
  MockEEPROM()
  {
    _mem.fill(0xff);
  }

  uint8_t read(int addr) { return _mem[addr]; }

  void write(int addr, uint8_t value) { _mem[addr] = value; }

  void update(int addr, uint8_t value) { if (read(addr) != value) write(addr, value); }

  template <typename T>
  void put(int addr, const T value)
  {
    memcpy(_mem.data() + addr, &value, sizeof(T));
  }

  template <typename T>
  void get(int addr, T& value)
  {
    memcpy(&value, _mem.data() + addr, sizeof(T));
  }

  uint8_t& operator[](int addr) { return _mem[addr]; }

  constexpr unsigned int length() const { return static_cast<unsigned int>(_mem.size()); }

  std::array<uint8_t, 4096> _mem{};
};

MOCK_API extern MockEEPROM EEPROM;

// ReSharper restore CppInconsistentNaming
// ReSharper restore CppMemberFunctionMayBeStatic
// ReSharper restore CppParameterNeverUsed
