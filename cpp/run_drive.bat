@echo off
chcp 65001 >nul
cd /d "%~dp0"
call build_run.bat drive
echo.
echo [done] press any key to close
pause >nul
