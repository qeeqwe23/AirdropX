param([string]$Python = "")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not $Python) {
    $candidates = @(
        (Join-Path $PSScriptRoot ".venv\Scripts\python.exe"),
        (Join-Path $PSScriptRoot "..\offline_gui_v136p\.venv\Scripts\python.exe"),
        (Join-Path $PSScriptRoot "..\offline_gui_v140\.venv\Scripts\python.exe"),
        "C:\Users\Administrator\AppData\Local\Programs\Python\Python312\python.exe",
        "python"
    )
    foreach ($c in $candidates) {
        if ($c -eq "python" -or (Test-Path $c)) { $Python = $c; break }
    }
}

& $Python -m pip install -r requirements.txt pyinstaller
if ($LASTEXITCODE -ne 0) { throw "Python dependency install failed." }

$Dist = Join-Path $PSScriptRoot "dist"
$Work = Join-Path $PSScriptRoot "build"
$Spec = Join-Path $PSScriptRoot "build_spec"
& $Python -m PyInstaller --noconfirm --clean --windowed --name "AirdropX" `
    --distpath $Dist --workpath $Work --specpath $Spec main.py
if ($LASTEXITCODE -ne 0) { throw "PyInstaller build failed." }

Write-Host ""
Write-Host "AirdropX.exe built successfully:" -ForegroundColor Green
Write-Host (Join-Path $Dist "AirdropX\AirdropX.exe")