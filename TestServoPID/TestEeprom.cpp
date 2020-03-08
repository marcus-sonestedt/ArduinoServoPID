#include "pch.h"

#include "../ServoPid/servopid.ino"

namespace
{
    bool operator==(const PidServo &a, const PidServo &b)
    {
        return memcmp(&a, &b, sizeof(PidServo)) == 0;        
    }

    std::ostream& operator<<(std::ostream& os, const PidServo &s)
    {
        return os 
            << "Servo pin: " << s._servo._pin << " pwm min: " << s._servo._pwmMin << " pwm max: " <<s._servo._pwmMax << std::endl
            << "Pot pin: " << s._analogPin._pin << " bias: " << s._analogPin._bias << " scale " << s._analogPin._scale << std::endl
            << "P: " << s._pid._pFactor << " I: " << s._pid._iFactor << " D: " << s._pid._dFactor << " DL: " << s._pid._dLambda  << std::endl
            << "Integral: " << s._pid._integral << " PE: " << s._pid._prevError << " DF: " << s._pid._deltaFiltered << std::endl
            << "Input: " << s._input << " Output: " << s._output << std::endl;
    }
}

TEST(TestEEPROM, TestSaveComputesCrc)
{
    EEPROM._mem.fill(0xFF);
    numServos = 0;
    saveEeprom();

    unsigned int storedCrc;
    const auto dataEndAddr = EEPROM.length() - sizeof(storedCrc);
    EEPROM.get(dataEndAddr, storedCrc);
    EXPECT_NE(storedCrc, 0xFFFFFFFFu);

    const auto computedCrc = calcEepromCrc(0, dataEndAddr);
    ASSERT_EQ(storedCrc, computedCrc);
}

TEST(TestEEPROM, TestDefaultInit)
{
    resetToDefaultValues();
    const std::vector<PidServo> defaultServos(&PidServos[0], &PidServos[numServos]);

    EEPROM._mem.fill(0xFF);
    initServosFromEeprom(); // should init with default values

    std::vector<PidServo> currentServos(numServos);
    std::copy(std::begin(PidServos), std::begin(PidServos) + numServos, std::begin(currentServos));

    ASSERT_THAT(currentServos, testing::ElementsAreArray(defaultServos));
}

TEST(TestEEPROM, TestInitSaves)
{
    EEPROM._mem.fill(0xFF);
    initServosFromEeprom();
    EXPECT_NE(EEPROM.read(0), 0xFF);
    EXPECT_NE(EEPROM.read(EEPROM.length() - 1), 0xFF);
}

TEST(TestEEPROM, TestLoadAfterSave)
{
    EEPROM._mem.fill(0xFF);
    initServosFromEeprom();
    saveEeprom(); // should be done by above
    const std::vector<PidServo> defaultServos(std::begin(PidServos), std::begin(PidServos) + numServos);

    // corrupt current data
    numServos = 42;
    std::fill(&PidServos[0], &PidServos[MAX_SERVOS], PidServo());

    loadEeprom();
    const std::vector<PidServo> currentServos(std::begin(PidServos), std::begin(PidServos) + numServos);
    
    ASSERT_THAT(currentServos, testing::ElementsAreArray(defaultServos));
}