#include "gtest/gtest.h"

#include "../servopid.ino"

TEST(TestPID, StaysOnZero)
{
    auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);

    for (auto i = 0; i < 10; ++i) {
        const auto output = pid.regulate(42.0f, 42.0f, 1.0f);
        ASSERT_FLOAT_EQ(output, 0.0f);
    }
}

TEST(TestPID, NoStartFlutter)
{
    auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
    const auto output = pid.regulate(42.0f, 42.0f, 1.0f);
    ASSERT_FLOAT_EQ(output, 0.0f);
}

TEST(TestPID, RegulateScaled)
{
    auto pid = PID(1.0f, 2.0f, 0.0f, 0.5f);

    auto process = 0.0f;
    for (auto i = 0; i < 1000; ++i) {
        const auto output = pid.regulate(process, 42.0f, 0.1f);
        process = output * 0.5f;
    }
    
    ASSERT_FLOAT_EQ(process, 42.0f);
}

TEST(TestPID, RegulateOffset)
{
    auto pid = PID(0.1f, 10.0f, 0.1f, 0.2f);

    auto process = 0.0f;
    for (auto i = 0; i < 1000; ++i) {
        const auto output = pid.regulate(process, 42.0f, 0.1f);
        process = output + 10.0f;
    }
    
    ASSERT_FLOAT_EQ(process, 42.0f);
}
