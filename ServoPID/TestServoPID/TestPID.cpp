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
