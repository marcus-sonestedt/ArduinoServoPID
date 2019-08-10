#include "pch.h"

#include "../ServoPid/servopid.ino"

TEST(TestSetupAndLoop, TestSetup)
{
    setup();
}

TEST(TestSetupAndLoop, TestSetupAndLoop)
{
    setup();
    loop();
}
