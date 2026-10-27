@echo off
rem Build one program (drive or nav) with g++ and run it.
rem Usage: build_run.bat nav      or      build_run.bat drive
cd /d "%~dp0"
rem Find g++ even if VS Code was opened before WinLibs was installed (PATH not refreshed yet)
set "PATH=%PATH%;%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
set NAME=%~1
if /i not "%NAME%"=="drive" if /i not "%NAME%"=="nav" set NAME=nav
echo [build] g++ %NAME%.cpp
g++ -std=c++17 -O2 -Wall -static "%NAME%.cpp" -o "%NAME%.exe"
if errorlevel 1 (
  echo [build] FAILED. Check that g++ is installed: g++ --version ^(need GCC 9 or newer^)
  exit /b 1
)
echo [build] OK, running %NAME%.exe
echo.
"%NAME%.exe"
