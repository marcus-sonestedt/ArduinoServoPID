@echo off
REM FROM: https://stackoverflow.com/questions/302393/including-all-dependencies
setlocal

set SEARCHDIR=%1
set OUTPUTFILE=%2
set TARGETINSTALLDIR=%3

echo Starting Dependency check...
echo ^<?xml version="1.0" encoding="UTF-8"?^> > %OUTPUTFILE%
echo ^<Wix xmlns="http://schemas.microsoft.com/wix/2006/wi"^> >> %OUTPUTFILE%
echo   ^<Fragment^> >> %OUTPUTFILE%
echo     ^<ComponentGroup Id="GeneratedDependencies" Directory="%TARGETINSTALLDIR%"^> >> %OUTPUTFILE%

for %%F in (%SEARCHDIR%\*.dll) do (
   echo "-- Adding %%~nxF" 
    echo       ^<Component Id="%%~nF"^> >> %OUTPUTFILE%
    echo                     ^<File  Id="%%~nF" Name="%%~nxF" Source="%%~dpnxF" Vital="yes" KeyPath="yes" DiskId="1"/^> >> %OUTPUTFILE%
    echo       ^</Component^> >> %OUTPUTFILE%
)
echo     ^</ComponentGroup^> >> %OUTPUTFILE%
echo   ^</Fragment^> >> %OUTPUTFILE%
echo ^</Wix^> >> %OUTPUTFILE%
echo Dependency check done.
