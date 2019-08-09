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

class Adafruit_PWMServoDriver {
 public:
  Adafruit_PWMServoDriver(uint8_t addr = 0x40, void *I2C = nullptr) : _i2caddr(addr), _i2c(I2C) {}
  ~Adafruit_PWMServoDriver();

    
    
  void begin(uint8_t prescale = 0) {}
  void reset() {}
  void sleep() {}
  void wakeup() {}
  void setExtClk(uint8_t prescale) {}
  void setPWMFreq(float freq) { _freq = freq; }
  void setOutputMode(bool totempole) { _outputMode = totempole; }
  uint8_t getPWM(uint8_t num) { return (_pwmOff[num] - _pwmOn[num]) / (4096/256); }
  void setPWM(uint8_t num, uint16_t on, uint16_t off) { _pwmOn[num] = on; _pwmOff[num] = off; }
  void setPin(uint8_t num, uint16_t val, bool invert=false);

 public:
  std::array<uint16_t, 16> _pwmOn;
  std::array<uint16_t, 16> _pwmOff;
  float _freq;
  bool _outputMode;

private:
  void *_i2c;
  uint8_t _i2caddr;
};