@echo off
setlocal
set "BUILD_SCRIPT="
for %%F in ("%~dp0*.ps1") do set "BUILD_SCRIPT=%%~fF"
if not defined BUILD_SCRIPT (
    echo ERROR: PowerShell preparation script was not found.
    pause
    exit /b 1
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%BUILD_SCRIPT%" -FlashNow
if errorlevel 1 (
    echo.
    echo Audio preparation failed. Do not flash.
    pause
    exit /b 1
)

echo.
echo Custom niulai audio is ready in the official flashing tool.
echo Keep this console open until flashing finishes.
pause
