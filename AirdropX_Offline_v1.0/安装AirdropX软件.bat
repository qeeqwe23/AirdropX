@echo off
chcp 65001 >nul
setlocal
set "ROOT=D:\vscode project\AirdropX"
if not exist "%ROOT%\matlab" (
  echo 璇疯緭鍏?AirdropX 椤圭洰鏍圭洰褰曪紝渚嬪 D:\vscode project\AirdropX
  set /p ROOT=ProjectRoot:
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install_into_airdropx.ps1" -ProjectRoot "%ROOT%"
if errorlevel 1 (
  echo.
  echo 瀹夎澶辫触锛岃鏌ョ湅涓婇潰鐨勯敊璇俊鎭€?  pause
  exit /b 1
)
echo.
echo 瀹夎瀹屾垚銆傚惎鍔ㄤ綅缃細%ROOT%\AirdropX_Software\AirdropX.bat
pause
