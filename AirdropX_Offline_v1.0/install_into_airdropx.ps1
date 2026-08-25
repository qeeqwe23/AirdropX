param(
    [Parameter(Mandatory=$true)][string]$ProjectRoot,
    [switch]$SkipExeBuild
)
$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = (Resolve-Path $ProjectRoot).Path
if (-not (Test-Path (Join-Path $ProjectRoot "matlab"))) { throw "Not an AirdropX root: $ProjectRoot" }

Write-Host "[1/6] Installing frozen Physics-MPC v1.3.6-Paper overlay..."
$PaperInstaller = Join-Path $Here "v136p_overlay\install_paper_validation_v136p.ps1"
& $PaperInstaller -ProjectRoot $ProjectRoot
if (-not $?) { throw "v1.3.6-Paper overlay install failed." }

Write-Host "[2/6] Installing the single v1.3.6-Paper GUI bridge..."
$BridgeDst = Join-Path $ProjectRoot "matlab\gui_bridge"
New-Item -ItemType Directory -Force -Path $BridgeDst | Out-Null
Copy-Item (Join-Path $Here "matlab\gui_bridge\airdropx_gui_backend_entry_v136p.m") $BridgeDst -Force
Copy-Item (Join-Path $Here "matlab\gui_bridge\airdropx_gui_prepare_model_v136p.m") $BridgeDst -Force

Write-Host "[3/6] Installing GUI custom-wind adapter without changing formal paper scenarios..."
$WindDir = Join-Path $ProjectRoot "matlab\wind"
$WindDst = Join-Path $WindDir "airdropx_wind_profile_v136.m"
if (Test-Path $WindDst) {
    $Backup = "$WindDst.pre_airdropx_software_v1.bak"
    if (-not (Test-Path $Backup)) { Copy-Item $WindDst $Backup }
}
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_v136.m") $WindDst -Force
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_gui.m") (Join-Path $WindDir "airdropx_wind_profile_gui.m") -Force

Write-Host "[4/6] Installing AirdropX software UI..."
$GuiDst = Join-Path $ProjectRoot "AirdropX_Software"
New-Item -ItemType Directory -Force -Path $GuiDst | Out-Null
foreach ($name in @(
    "main.py","requirements.txt","launch_gui.bat","AirdropX.bat",
    "build_python_exe.ps1","install_python_env.ps1","README_SOFTWARE.md","VERSION.txt"
)) {
    Copy-Item (Join-Path $Here $name) $GuiDst -Force
}
foreach ($dir in @("core","ui","config","docs","tests")) {
    $dst = Join-Path $GuiDst $dir
    if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
    Copy-Item (Join-Path $Here $dir) $GuiDst -Recurse -Force
}

Write-Host "[5/6] Sanity checks..."
$required = @(
    (Join-Path $ProjectRoot "matlab\airdrop\airdropx_wind_airdrop_mission_v136p.m"),
    (Join-Path $ProjectRoot "matlab\gui_bridge\airdropx_gui_backend_entry_v136p.m"),
    (Join-Path $ProjectRoot "matlab\gui_bridge\airdropx_gui_prepare_model_v136p.m"),
    (Join-Path $ProjectRoot "matlab\wind\airdropx_wind_profile_gui.m"),
    (Join-Path $GuiDst "main.py")
)
foreach ($p in $required) { if (-not (Test-Path $p)) { throw "Install incomplete, missing: $p" } }

Write-Host "[6/6] Building Windows GUI executable..."
if (-not $SkipExeBuild) {
    Push-Location $GuiDst
    try {
        & (Join-Path $GuiDst "build_python_exe.ps1")
    }
    finally {
        Pop-Location
    }
} else {
    Write-Host "EXE build skipped. launch_gui.bat can run the software directly."
}

Write-Host ""
Write-Host "AirdropX software installed successfully." -ForegroundColor Green
Write-Host "Controller: Physics-MPC v1.3.6-Paper (fixed)"
Write-Host "Software folder: $GuiDst"
Write-Host "Launcher: $GuiDst\AirdropX.bat"
if (Test-Path (Join-Path $GuiDst "dist\AirdropX\AirdropX.exe")) {
    Write-Host "Executable: $GuiDst\dist\AirdropX\AirdropX.exe" -ForegroundColor Green
}
