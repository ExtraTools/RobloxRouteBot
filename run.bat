@echo off
setlocal
cd /d "%~dp0"

echo ============================================
echo   Roblox Route Bot - launcher
echo ============================================

rem -- kill old instance if running --
taskkill /IM RobloxRouteBot.exe /F >nul 2>&1

rem -- check .NET SDK --
where dotnet >nul 2>&1
if errorlevel 1 (
  echo [ERROR] .NET SDK not found.
  echo Install .NET 8 SDK from: https://dotnet.microsoft.com/download
  echo.
  pause
  exit /b 1
)

echo Building (Release, x64)...
dotnet build -c Release -p:Platform=x64 -v minimal
if errorlevel 1 (
  echo.
  echo [ERROR] Build failed. See messages above.
  pause
  exit /b 1
)

set "EXE=%~dp0bin\x64\Release\net8.0-windows10.0.19041.0\RobloxRouteBot.exe"
if not exist "%EXE%" (
  echo [ERROR] Executable not found: %EXE%
  pause
  exit /b 1
)

echo Starting...
start "" "%EXE%"
endlocal
