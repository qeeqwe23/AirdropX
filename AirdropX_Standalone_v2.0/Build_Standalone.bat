@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Build_Standalone.ps1" %*
if errorlevel 1 (
  echo.
  echo BUILD FAILED. See the error above.
  pause
  exit /b 1
)
echo.
echo Build completed successfully.
pause
