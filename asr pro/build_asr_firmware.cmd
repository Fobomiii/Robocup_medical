@echo off
setlocal
set "BUILD_SCRIPT="
for %%F in ("%~dp0*.ps1") do set "BUILD_SCRIPT=%%~fF"
if not defined BUILD_SCRIPT (
    echo ERROR: PowerShell build script was not found.
    pause
    exit /b 1
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%BUILD_SCRIPT%" > "%~dp0build_asr_firmware.log" 2>&1
set "BUILD_EXIT=%ERRORLEVEL%"
type "%~dp0build_asr_firmware.log"
if not "%BUILD_EXIT%"=="0" (
    echo.
    echo Firmware build failed. See build_asr_firmware.log.
    pause
    exit /b 1
)

echo.
echo Firmware generated: %~dp0niulaipro_final.bin
echo Disconnect STM32 PD1 from ASR PB6 before flashing.
pause
