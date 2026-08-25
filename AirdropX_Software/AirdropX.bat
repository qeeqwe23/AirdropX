@echo off
setlocal
set "APPDIR=%~dp0"
set "AIRDROPX_PROJECT_ROOT=%APPDIR%.."
cd /d "%APPDIR%"
call "%APPDIR%launch_gui.bat"
