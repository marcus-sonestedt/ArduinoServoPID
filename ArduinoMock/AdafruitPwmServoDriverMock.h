#pragma once

#define PCA9685_SUBADR1 0x2 /**< i2c bus address 1 */
#define PCA9685_SUBADR2 0x3 /**< i2c bus address 2 */
#define PCA9685_SUBADR3 0x4 /**< i2c bus address 3 */

#define PCA9685_MODE1 0x0 /**< Mode Register 1 */
#define PCA9685_MODE2 0x1 /**< Mode Register 2 */
#define PCA9685_PRESCALE 0xFE /**< Prescaler for PWM output frequency */

#define LED0_ON_L 0x6 /**< LED0 output and brightness control byte 0 */
#define LED0_ON_H 0x7 /**< LED0 output and brightness control byte 1 */
#define LED0_OFF_L 0x8 /**< LED0 output and brightness control byte 2 */
#define LED0_OFF_H 0x9 /**< LED0 output and brightness control byte 3 */

#define ALLLED_ON_L 0xFA /**< load all the LEDn_ON registers, byte 0 */
#define ALLLED_ON_H 0xFB /**< load all the LEDn_ON registers, byte 1 */
#define ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, byte 0 */
#define ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers, byte 1 */

// ReSharper disable CppMemberFunctionMayBeStatic

#pragma warning(push)
#pragma warning(disable: 4251) // std::vector needs to have DLL interface to be used by clients...

class MOCK_API Adafruit_PWMServoDriver
{
public:
  explicit Adafruit_PWMServoDriver(uint8_t addr = 0x40, void* I2C = nullptr)
    : _pwmOn(16), _pwmOff(16), _i2c(I2C), _i2caddr(addr)
  {
  }  

  void begin(uint8_t prescale = 0)
  {
    _prescale = prescale;
  }

  void reset()
  {
  }

  void sleep()
  {
  }

  void wakeup()
  {
  }

  void setExtClk(uint8_t prescale)
  {
    _prescale = prescale;
  }

  void    setPWMFreq(float freq) { _freq = freq; }
  void    setOutputMode(bool totempole) { _outputMode = totempole; }
  uint8_t getPWM(uint8_t num) { return static_cast<uint8_t>((_pwmOff.at(num) - _pwmOn.at(num)) / (4096 / 256)); }

  void setPWM(uint8_t num, uint16_t on, uint16_t off)
  {
    _pwmOn.at(num) = on;
    _pwmOff.at(num) = off;
  }

  void setPin(uint8_t num, uint16_t val, bool invert = false);

public:
  std::vector<uint16_t> _pwmOn;
  std::vector<uint16_t> _pwmOff;
  float                 _freq = 60;
  uint8_t               _prescale = 0;
  bool                  _outputMode = false;

private:
  void*   _i2c = nullptr;
  uint8_t _i2caddr = 0x40;
};

#pragma warning(pop)

// ReSharper restore CppMemberFunctionMayBeStatic
