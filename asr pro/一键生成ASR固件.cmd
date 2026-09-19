@echo off
setlocal
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0准备自定义开机音.ps1" > "%~dp0固件生成日志.txt" 2>&1
set "BUILD_EXIT=%ERRORLEVEL%"
type "%~dp0固件生成日志.txt"
if not "%BUILD_EXIT%"=="0" (
    echo.
    echo 生成失败，请保留本窗口并查看上方错误。
    pause
    exit /b 1
)
echo.
echo 已生成：%~dp0niulaipro_final.bin
echo 烧录 ASR Pro 时请先断开 STM32 PD1 到 ASR PB6 的连线。
pause
