// TestServoPID.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"

int main(int argc, char* argv[])
{
  ::SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
