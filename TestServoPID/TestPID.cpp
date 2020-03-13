#include "pch.h"

#include "../ServoPid/servopid.ino"

TEST(TestPID, StaysOnZero)
{
  auto pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
  const auto input = 42.0f;
  pid.reset(input);
  for (auto i = 0; i < 10; ++i)
  {
    const auto output = pid.regulate(input, input, 1.0f);
    ASSERT_FLOAT_EQ(output, 0.0f);
  }
}

TEST(TestPID, NoStartFlutter)
{
  auto       pid = PID(1.0f, 2.0f, 3.0f, 0.5f);
  const auto input = 42.0f;
  pid.reset(input);
  const auto output = pid.regulate(input, input, 1.0f);
  ASSERT_FLOAT_EQ(output, 0.0f);
}

TEST(TestPID, NoDeltaRelatedSpikeOnSetPosChange)
{
  auto       pid = PID(0.0f, 0.0f, 3.0f, 0.5f);
  const auto input = 42.0f;
  pid.reset(input);
  const auto output1 = pid.regulate(input, input, 0.001f);
  const auto output2 = pid.regulate(input, input*2, 0.001f);
  ASSERT_NEAR(output1, output2, 1);
}


TEST(TestPID, RegulateScaled)
{
  auto pid = PID(1.0f, 2.0f, 0.0f, 0.5f);
  auto input = 0.0f;
  pid.reset(input);

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
  pid.reset(input);

  for (auto i = 0; i < 1000; ++i)
  {
    const auto output = pid.regulate(input, 42.0f, 0.1f);
    input = output + 10.0f;
  }

  ASSERT_FLOAT_EQ(input, 42.0f);
}

TEST(TestPID, RegulateMassSpringBounce)
{
  auto pid = PID(1.5f, 2.0f, 1.0f, 0.5f);

  PID::MaxIntegratorStore = 500;

  const auto m = 1.0f;
  const auto k = 10.0f;
  const auto dt = 0.05f;

  auto pos = -2.0f;
  auto vel = 0.0f;
  auto setPos = 0.0f;

  pid.reset(pos);

  std::ofstream csv;
  csv.open("pid_ms.csv", std::ios::out);
  ASSERT_TRUE(csv.is_open());

  csv << "Requested" << ',' << "Output" << ',' << "Actual" << ',' << "Integral" << ',' << "DeltaF" << '\n';
  csv.precision(3);
  csv << setPos << "," << '0' << ',' << pos << ',' << pid._integral << ',' << pid._dValueFiltered << std::endl;

  for (auto i = 0; i < 300; ++i)
  {
    setPos = ((i / 100) % 3) * 10.0f;
    const auto attachPos = constrain(pid.regulate(pos, setPos, dt), -100, 100);

    const auto force = (pos - attachPos) * -k;
    const auto acc = force / m;

    vel += acc * dt;
    //vel *= 0.95f; // damping
    pos += vel * dt;

    csv << setPos << "," << attachPos << ',' << pos << ',' << pid._integral << ',' << pid._dValueFiltered << std::endl;
  }

  ASSERT_NEAR(pos, setPos, 1);
}
