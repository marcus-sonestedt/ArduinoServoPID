cd "$PSScriptRoot"

$ArduinoRoot="${env:ProgramFiles(x86)}\Arduino"
$TargetDir="$PSScriptRoot\bin"

$ArduinoLibs=@('-libraries', "$ArduinoRoot\lib")
$UserLibs1="$($Env:USERPROFILE)\Documents\Arduino\libraries"
$UserLibs2="$($Env:USERPROFILE)\OneDrive\Documents\Arduino\libraries"

if (Test-Path $UserLibs1) { $ArduinoLibs += @("-libraries", $UserLibs1) }
if (Test-Path $UserLibs2) { $ArduinoLibs += @("-libraries", $UserLibs2) }
if (-not (Test-Path $TargetDir)) { mkdir $TargetDir }

$srcTime = Get-ItemProperty -Path servopid.ino -Name LastWriteTime
$dstTime = Get-ItemProperty -Path "$TargetDir/servopid.ino.elf" -Name LastWriteTime -ErrorAction SilentlyContinue

if ($null -eq $dstTime -or $srcTime.LastWriteTime -gt $dstTime.LastWriteTime) {
    & "$ArduinoRoot\arduino-builder.exe" '-compile' `
      '-hardware' "$ArduinoRoot\hardware" `
      '-tools' "$ArduinoRoot\hardware/tools"  `
      '-tools' "$ArduinoRoot\tools-builder" `
      @ArduinoLibs `
      '-fqbn' 'arduino:avr:uno'  `
      '-build-path' $TargetDir  `
      'servopid.ino'
} else {
    Write-Output "Target is up to date."
}