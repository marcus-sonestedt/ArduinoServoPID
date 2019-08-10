@echo on
setlocal enabledelayedexpansion

set ARDUINO=%ProgramFiles(x86)%\Arduino
set TargetDir=%~dp0\bin
set ArduinoLibs=-libraries "%ARDUINO%\lib" 
set UserLibs1=%USERPROFILE%\Documents\Arduino\libraries
set UserLibs2=%USERPROFILE%\OneDrive\Documents\Arduino\libraries
if exist "%USERLIBS1%" set ArduinoLibs=%ArduinoLibs% -libraries "%USERLIBS1%"
if exist "%USERLIBS2%" set ArduinoLibs=%ArduinoLibs% -libraries "%USERLIBS2%"
if not exist "%Targetdir%" mkdir "%Targetdir%"

"%ARDUINO%\arduino-builder.exe" -compile ^
  -hardware "%ARDUINO%\hardware" ^
  -tools "%ARDUINO%\hardware/tools"  ^
  -tools "%ARDUINO%\tools-builder" ^
  %ArduinoLibs% ^
  -fqbn arduino:avr:uno  ^
  -build-path "%TargetDir%"  ^
  servopid.ino
