param([string]$Python = "python")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
& $Python -m pip install -r requirements.txt pyinstaller
& $Python -m PyInstaller --noconfirm --clean --windowed --name "AirdropX_Offline_GUI_v140" `
    --add-data "config;config" `
    main.py
Write-Host "GUI executable: $PSScriptRoot\dist\AirdropX_Offline_GUI_v140\AirdropX_Offline_GUI_v140.exe"
