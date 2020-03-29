$TargetDir="$PSScriptRoot\bin"
$SourceName='servopid.ino'
$Source="$PSScriptRoot\$SourceName"
$Target="$TargetDir\$SourceName.elf"
if (-not (Test-Path $TargetDir)) { mkdir $TargetDir }

$ArduinoRoot="${env:ProgramFiles(x86)}\Arduino"
$ArduinoLibs=@('-libraries', "$ArduinoRoot\lib")
$UserLibs1="$($Env:USERPROFILE)\Documents\Arduino\libraries"
$UserLibs2="$($Env:USERPROFILE)\OneDrive\Documents\Arduino\libraries"

if (Test-Path $UserLibs1) { $ArduinoLibs += @("-libraries", $UserLibs1) }
if (Test-Path $UserLibs2) { $ArduinoLibs += @("-libraries", $UserLibs2) }

$srcTime = Get-ItemProperty -Path $Source -Name LastWriteTime
$dstTime = Get-ItemProperty -Path $Target -Name LastWriteTime -ErrorAction SilentlyContinue
$scriptTime = Get-ItemProperty -Path $PSCommandPath -Name LastWriteTime -ErrorAction SilentlyContinue

if ($null -eq $dstTime `
    -or $srcTime.LastWriteTime -gt $dstTime.LastWriteTime `
    -or $scriptTime.LastWriteTime -gt $dstTime.LastWriteTime) 
{
   Write-Output "Compiling $Source -> $Target ..."
    
   & "$ArduinoRoot\arduino-builder.exe" '-compile' `
      '-hardware' "$ArduinoRoot\hardware" `
      '-tools' "$ArduinoRoot\hardware/tools"  `
      '-tools' "$ArduinoRoot\tools-builder" `
      @ArduinoLibs `
      '-fqbn' 'arduino:avr:uno'  `
      '-build-path' $TargetDir  `
      $SourceName
} else {
    Write-Output "$Source -> $Target (already up to date)"
}