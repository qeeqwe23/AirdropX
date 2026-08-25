@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

rem Prefer the packaged Windows executable when available.
if exist "%~dp0dist\AirdropX\AirdropX.exe" (
  "%~dp0dist\AirdropX\AirdropX.exe"
  exit /b %errorlevel%
)

set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=%~dp0..\offline_gui_v136p\.venv\Scripts\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=%~dp0..\offline_gui_v140\.venv\Scripts\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=C:\Users\Administrator\AppData\Local\Programs\Python\Python312\python.exe"
if not exist "%PYTHON_EXE%" set "PYTHON_EXE=python"
"%PYTHON_EXE%" main.py
if errorlevel 1 pause