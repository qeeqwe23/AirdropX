param([string]$Python = "")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (-not $Python) {
    $venv = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
    if (Test-Path $venv) { $Python = $venv } else { $Python = "python" }
}
& $Python -m pip install -r requirements.txt pyinstaller
& $Python -m PyInstaller --noconfirm --clean --windowed --name "AirdropX_Offline_GUI_v136P" `
    --add-data "config;config" `
    main.py
Write-Host "GUI executable: $PSScriptRoot\dist\AirdropX_Offline_GUI_v136P\AirdropX_Offline_GUI_v136P.exe"
