#include "pch.h"

#include "ArduinoMock.h"
#include "PCA9685Mock.h"

constexpr uint16_t PCA9685_PWM_FULL = 0x01000; // Special value for full on/full off LEDx modes

// ReSharper disable CppMemberFunctionMayBeConst

PCA9685_ServoEvaluator::PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t p90PWMAmount)
{
    n90PWMAmount = constrain(n90PWMAmount, uint16_t(0), PCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, n90PWMAmount, PCA9685_PWM_FULL);

    _coeff = new float[2];
    _isCSpline = false;

    _coeff[0] = n90PWMAmount;
    _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f;
}

PCA9685_ServoEvaluator::PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount)
{
    n90PWMAmount = constrain(n90PWMAmount, uint16_t(0), PCA9685_PWM_FULL);
    zeroPWMAmount = constrain(zeroPWMAmount, n90PWMAmount, PCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, zeroPWMAmount, PCA9685_PWM_FULL);

    if (p90PWMAmount - zeroPWMAmount != zeroPWMAmount - n90PWMAmount)
    {
        _coeff = new float[8];
        _isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        * this notice you can do whatever you want with this stuff. If we meet some day, and you
        * think this stuff is worth it, you can buy me a beer in return. */

        float x[3] = {0, 90, 180};
        float y[3] = {float(n90PWMAmount), float(zeroPWMAmount), static_cast<float>(p90PWMAmount)};
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i)
        {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i)
        {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            _coeff[4 * i + 0] = y[i]; // a
            _coeff[4 * i + 1] = b[i]; // b
            _coeff[4 * i + 2] = c[i]; // c
            _coeff[4 * i + 3] = d[i]; // d
        }
    }
    else
    {
        _coeff = new float[2];
        _isCSpline = false;

        _coeff[0] = n90PWMAmount;
        _coeff[1] = float(p90PWMAmount - n90PWMAmount) / 180.0f;
    }
}

PCA9685_ServoEvaluator::~PCA9685_ServoEvaluator()
{
    delete[] _coeff;
}

uint16_t PCA9685_ServoEvaluator::pwmForAngle(float angle)
{
    float retVal;
    angle = constrain(angle + 90, 0.0f, 180.0f);

    if (!_isCSpline)
    {
        retVal = _coeff[0] + (_coeff[1] * angle);
    }
    else
    {
        if (angle <= 90)
        {
            retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle
            );
        }
        else
        {
            angle -= 90;
            retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle
            );
        }
    }

    return constrain(uint16_t(roundf(retVal)), uint16_t(0), PCA9685_PWM_FULL);
};
