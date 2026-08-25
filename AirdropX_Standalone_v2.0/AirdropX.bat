@echo off
setlocal
cd /d "%~dp0"
if exist "%~dp0dist\AirdropX\AirdropX.exe" (
  start "" "%~dp0dist\AirdropX\AirdropX.exe"
  exit /b 0
)
if exist "%~dp0.venv_standalone\Scripts\python.exe" (
  "%~dp0.venv_standalone\Scripts\python.exe" "%~dp0main.py"
  exit /b %errorlevel%
)
echo AirdropX is not built yet. Run Build_Standalone.bat first.
pause
exit /b 1
