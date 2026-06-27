@echo off
cd /d "%~dp0"

:start
echo 正在启动 AEB.exe...
start  ..\bin\Release\AEB.exe

:loop
timeout /t 1 >nul

tasklist | findstr /i "AEB.exe" >nul
if %ERRORLEVEL% == 0 (
    goto loop
) else (
    echo AEB.exe 已关闭，正在重新启动...
timeout /t 3 >nul
    goto start
)