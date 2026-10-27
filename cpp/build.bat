@echo off
rem Build drive.exe and nav.exe (no run).
cd /d "%~dp0"
rem Find g++ even if VS Code was opened before WinLibs was installed (PATH not refreshed yet)
set "PATH=%PATH%;%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
g++ -std=c++17 -O2 -Wall -static drive.cpp -o drive.exe || goto :fail
g++ -std=c++17 -O2 -Wall -static nav.cpp -o nav.exe || goto :fail
echo Build OK: drive.exe nav.exe
exit /b 0
:fail
echo Build FAILED. Check that g++ is installed: g++ --version ^(need GCC 9 or newer^)
exit /b 1
