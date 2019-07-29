#include "pch.h"

#include "../servopid.ino"

class SerialTest : public testing::Test
{
public:
    SerialTest() = default;

    void SetUp() override
    {
        Serial.resetMock();
        serialLen = 0;
        memset(serialBuf, 0, sizeof serialBuf);
    }

    std::string GetSerialLine()
    {
        std::string resp;
        std::getline(Serial.dataWrite, resp);
        return resp;
    }

    void TearDown() override
    {
    }
};

TEST_F(SerialTest, CheckFixture)
{
    EXPECT_EQ(serialLen, 0);
    EXPECT_EQ(Serial.available(), 0);
}

TEST_F(SerialTest, CheckReset)
{
    Serial.setMockData("RST\n");
    EXPECT_GE(Serial.available(), 0);

    mySerialEvent();
    EXPECT_EQ(serialLen, 0);

    EXPECT_EQ("RST ACK", GetSerialLine());
}

TEST_F(SerialTest, CheckGetNumServos)
{
    const std::vector<char> data{2, char(Command::GetNumServos)};

    Serial.setMockData(data);
    mySerialEvent();
    EXPECT_EQ(serialLen, 0);

    EXPECT_EQ("NS 4", GetSerialLine());
    EXPECT_EQ(GetSerialLine().length(), 0);
}


TEST_F(SerialTest, CheckGetNumServosTwice)
{
    const std::vector<char> data{2, char(Command::GetNumServos), 2, char(Command::GetNumServos)};

    Serial.setMockData(data);
    mySerialEvent();
    EXPECT_EQ(serialLen, 0);

    EXPECT_EQ("NS 4", GetSerialLine());
    EXPECT_EQ("NS 4", GetSerialLine());
    EXPECT_EQ(GetSerialLine(), "");
}



TEST_F(SerialTest, GetServoParams)
{
    const std::vector<char> data{2, char(Command::GetServoParams)};

    Serial.setMockData(data);
    EXPECT_GE(Serial.available(), 0);

    mySerialEvent();
    EXPECT_EQ(serialLen, 0);

    using ::testing::StartsWith;

    EXPECT_THAT(GetSerialLine(), StartsWith("SP 0 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SP 1 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SP 2 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SP 3 "));
    ASSERT_EQ(GetSerialLine(), "");
}

TEST_F(SerialTest, GetServoData)
{
    const std::vector<char> data{3, char(Command::GetServoData), 5};

    Serial.setMockData(data);
    EXPECT_GE(Serial.available(), 0);

    mySerialEvent();
    EXPECT_EQ(serialLen, 0);

    using ::testing::StartsWith;

    EXPECT_THAT(GetSerialLine(), StartsWith("SD 0 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SD 1 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SD 2 "));
    EXPECT_THAT(GetSerialLine(), StartsWith("SD 3 "));
    ASSERT_EQ(GetSerialLine(), "");
}
