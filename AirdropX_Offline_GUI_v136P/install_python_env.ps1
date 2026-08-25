param([string]$Python = "")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (-not $Python) {
    $known = "C:\Users\Administrator\AppData\Local\Programs\Python\Python312\python.exe"
    if (Test-Path $known) { $Python = $known } else { $Python = "python" }
}
if (-not (Test-Path ".venv\Scripts\python.exe")) {
    & $Python -m venv .venv
}
& ".venv\Scripts\python.exe" -m pip install --upgrade pip
& ".venv\Scripts\python.exe" -m pip install -r requirements.txt
& ".venv\Scripts\python.exe" -c "import PyQt6; print('PyQt6 import PASS')"
Write-Host "Ready: $PSScriptRoot\launch_gui.bat"
