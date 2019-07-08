#pragma once

// ReSharper disable CppMemberFunctionMayBeStatic

// ReSharper disable IdentifierTypo
constexpr int PCA9685_MODE_INVRT = 0x10;  // Inverts polarity of channel output signal
constexpr int PCA9685_MODE_OUTPUT_ONACK = 0x08;  // Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
constexpr int PCA9685_MODE_OUTPUT_TPOLE = 0x04;  // Use a totem-pole (push-pull) style output, typical for boards using this chipset
constexpr int PCA9685_MODE_OUTNE_HIGHZ = 0x02;  // For active low output enable, sets channel output to high-impedance state
constexpr int PCA9685_MODE_OUTNE_LOW = 0x01;  // Similarly, sets channel output to high if in totem-pole mode, otherwise high-impedance state

constexpr int PCA9685_MIN_CHANNEL = 0;
constexpr int PCA9685_MAX_CHANNEL = 15;
constexpr int PCA9685_CHANNEL_COUNT = 16;
// ReSharper restore IdentifierTypo

typedef uint8_t byte;

class PCA9685  
{
public:
    // Should be called only once in setup(), before any init()'s, but after Wire.begin().
    // Only should be called once on any Wire instance to do a software reset, which
    // will affect all devices on that line. This helps when you're constantly rebuilding
    // and re-uploading to ensure all the devices on that line are reset properly.
    void resetDevices() {}

    // Called in setup(). The i2c address here is the value of the A0, A1, A2, A3, A4 and
    // A5 pins ONLY, as the class takes care of its internal base address. i2cAddress
    // should be a value between 0 and 61, since only 62 boards can be addressed.
    void init(byte i2cAddress = 0, byte mode = PCA9685_MODE_OUTPUT_ONACK | PCA9685_MODE_OUTPUT_TPOLE) {}

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz (resolution widens as Hz goes higher)
    void setPWMFrequency(float pwmFrequency) { _pwmFreq = pwmFrequency;  }

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, uint16_t pwmAmount) { _pwmValues.at(channel) = pwmAmount; }
    void setChannelsPWM(int begChannel, int numChannels, const uint16_t* pwmAmounts)
    {
        for (auto i = 0; i < numChannels; ++i)
            _pwmValues.at(i + begChannel) = pwmAmounts[i];
    }

private:
    float _pwmFreq = 200.0F;
    std::array<uint16_t, 16> _pwmValues = {};
};

// ReSharper restore CppMemberFunctionMayBeStatic

class PCA9685_ServoEvaluator sealed {  // NOLINT 
public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90.
    explicit PCA9685_ServoEvaluator(uint16_t n90PWMAmount = 102, uint16_t p90PWMAmount = 512);
    PCA9685_ServoEvaluator(const PCA9685_ServoEvaluator&) = delete;

    // Uses a cubic spline to interpolate due to an offset zero angle that isn't
    // exactly between -90/+90. This takes more time to compute, but gives a more
    // accurate PWM output value along the entire range.
    PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount);

    ~PCA9685_ServoEvaluator();

    // Returns the PWM value to use given the angle (-90 to +90)
    uint16_t pwmForAngle(float angle);

private:
    float* _coeff;      // a,b,c,d coefficient values
    bool _isCSpline = false;    // Cubic spline tracking, for _coeff length
};
