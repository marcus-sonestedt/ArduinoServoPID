#include "pch.h"

#include "../servopid.ino"

TEST(TestSetupAndLoop, TestSetup)
{
    setup();
}

TEST(TestSetupAndLoop, TestSetupAndLoop)
{
    setup();
    loop();
}
