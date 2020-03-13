#include "pch.h"

#include "../ServoPid/servopid.ino"

TEST(TestPID, StaysOnZero)
{
    auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);

    for (auto i = 0; i < 10; ++i)
    {
        const auto output = pid.regulate(42.0f, 42.0f, 1.0f);
        ASSERT_FLOAT_EQ(output, 0.0f);
    }
}

TEST(TestPID, NoStartFlutter)
{
    auto       pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
    const auto output = pid.regulate(42.0f, 42.0f, 1.0f);
    ASSERT_FLOAT_EQ(output, 0.0f);
}

TEST(TestPID, RegulateScaled)
{
    auto pid = PID(1.0f, 2.0f, 0.0f, 0.5f);

    auto input = 0.0f;
    for (auto i = 0; i < 1000; ++i)
    {
        const auto output = pid.regulate(input, 42.0f, 0.1f);
        input = output * 0.5f;
    }

    ASSERT_FLOAT_EQ(input, 42.0f);
}

TEST(TestPID, RegulateOffset)
{
    auto pid = PID(0.1f, 10.0f, 0.1f, 0.2f);

    auto input = 0.0f;
    for (auto i = 0; i < 1000; ++i)
    {
        const auto output = pid.regulate(input, 42.0f, 0.1f);
        input = output + 10.0f;
    }

    ASSERT_FLOAT_EQ(input, 42.0f);
}

TEST(TestPID, RegulateMassSpringBounce)
{
    auto pid = PID(-2.0f, 2.0f, 0.0f, 0.5f);

    PID::MaxIntegratorStore = 50;

    const auto m = 2.0f;
    const auto k = 5.0f;
    const auto dt = 0.05f;

    auto pos = 20.0f;
    auto vel = 0.0f;

    std::ofstream csv;
    csv.open("pid_ms.csv", std::ios::out);
    ASSERT_TRUE(csv.is_open());

    csv << "Requested" << ',' << "Output" << ',' << "Actual" << ',' << "Integral" << ',' << "DeltaF" << '\n';

    for (auto i = 0; i < 400; ++i)
    {
        const auto setPos = ((i / 100) % 3) * 10.0f;
        const auto attachPos = constrain(pid.regulate(pos, setPos,dt), -100, 100);
        
        const auto force = (pos - attachPos) * -k;
        const auto acc = force / m;
        
        vel += acc * dt;
        pos += vel * dt;

        csv << setPos << "," << attachPos << ',' << pos << ',' << pid._integral << ',' << pid._deltaFiltered << std::endl;
    }

    ASSERT_NEAR(pos, 0, 1);
}
