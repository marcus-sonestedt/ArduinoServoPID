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
            << "Servo pin: " << int(s._servo._pin) << " pwm min: " << s._servo._pwmMin << " pwm max: " <<s._servo._pwmMax << std::endl
            << "Pot pin: " << int(s._analogPin._pin) << " min: " << s._analogPin._min << " max " << s._analogPin._max << std::endl
            << "P: " << s._pid._pFactor << " I: " << s._pid._iFactor << " D: " << s._pid._dFactor << " DL: " << s._pid._dLambda  << std::endl
            << "Integral: " << s._pid._integral << " PV: " << s._pid._prevValue << " DF: " << s._pid._dValueFiltered << std::endl
            << "SetPoint: " << s._pid._setPoint << " Input: " << s._input << " Output: " << s._output << std::endl;
    }
}

TEST(TestEEPROM, TestSaveComputesCrc)
{
    EEPROM._mem.fill(0xFF);
    numServos = 0;
    saveEeprom();

    uint32_t storedCrc;
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
    ASSERT_GE(numServos, 0);

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
    setMockAnalogRead(0, 0);
    setMockAnalogRead(1, 0); 
    setMockAnalogRead(2, 0); 
    setMockAnalogRead(3, 0); 

    EEPROM._mem.fill(0xFF);
    initServosFromEeprom();

    // change default values
    PID::MaxIntegratorStore = 420;
    ServoBase::MinAngle = 3;
    ServoBase::MaxAngle = 93;
    Deadband::MaxDeviation = 9;

    numServos = 4;
    PidServos[0]._analogPin._pin = 7;
    PidServos[1]._pid._pFactor = 2;
    PidServos[2]._pid._iFactor = 3;
    PidServos[3]._pid._dFactor = 4;

    // run one step
    for (auto& ps : PidServos)
      ps.run(0.5f);

    // save
    saveEeprom();
    const auto savedServos = std::vector(&PidServos[0], &PidServos[numServos]);

    // corrupt current data
    numServos = 42;
    PID::MaxIntegratorStore = 666;
    ServoBase::MinAngle = -1;
    ServoBase::MaxAngle = -2;
    Deadband::MaxDeviation = -3;
    std::fill(std::begin(PidServos), std::end(PidServos), PidServo());
    const auto corruptServos = std::vector(&PidServos[0], &PidServos[4]);
    ASSERT_THAT(corruptServos, testing::Not(testing::ElementsAreArray(savedServos)));

    // load back, check size
    loadEeprom();

    ASSERT_EQ(numServos, savedServos.size());
    ASSERT_EQ(PID::MaxIntegratorStore, 420);
    ASSERT_EQ(ServoBase::MinAngle, 3);
    ASSERT_EQ(ServoBase::MaxAngle, 93);
    ASSERT_EQ(Deadband::MaxDeviation, 9);
    
    const auto currentServos = std::vector<PidServo> (&PidServos[0], PidServos + numServos);  
    ASSERT_THAT(currentServos, testing::ElementsAreArray(savedServos));
}