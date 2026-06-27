@echo off
cd /d "%~dp0"

:start
echo 正在启动 AVP.exe...
start  ..\bin\Release\AVP.exe

:loop
timeout /t 1 >nul

tasklist | findstr /i "AVP.exe" >nul
if %ERRORLEVEL% == 0 (
    goto loop
) else (
    echo AVP.exe 已关闭，正在重新启动...
timeout /t 3 >nul
    goto start
)