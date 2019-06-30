#include "gtest/gtest.h"

#include "../servopid.ino"

TEST(TestPID, StaysOnZero)
{
    auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
    auto output = 0.0f;
    for (auto i = 0; i < 10; ++i)
        output = pid.regulate(42.0f, 42.0f, 1.0f);
    ASSERT_FLOAT_EQ(output, 0.0f);
}

TEST(TestPID, NoStartFlutter)
{
    auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
    const auto output = pid.regulate(42.0f, 42.0f, 1.0f);
    ASSERT_FLOAT_EQ(output, 0.0f);
}
