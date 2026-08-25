param(
    [Parameter(Mandatory=$true)][string]$ProjectRoot
)
$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = (Resolve-Path $ProjectRoot).Path
if (-not (Test-Path (Join-Path $ProjectRoot "matlab"))) { throw "Not an AirdropX root: $ProjectRoot" }

Write-Host "[1/5] Installing Physics-MPC v1.3.6-Paper overlay..."
$PaperInstaller = Join-Path $Here "v136p_overlay\install_paper_validation_v136p.ps1"
& $PaperInstaller -ProjectRoot $ProjectRoot

Write-Host "[2/5] Installing GUI MATLAB bridges (v1.3.6-Paper default + v1.4.0 optional)..."
$BridgeDst = Join-Path $ProjectRoot "matlab\gui_bridge"
New-Item -ItemType Directory -Force -Path $BridgeDst | Out-Null
Copy-Item (Join-Path $Here "matlab\gui_bridge\*.m") $BridgeDst -Force

Write-Host "[3/5] Installing isolated gui_custom wind extension..."
$WindDir = Join-Path $ProjectRoot "matlab\wind"
$WindDst = Join-Path $WindDir "airdropx_wind_profile_v136.m"
if (Test-Path $WindDst) {
    $Backup = "$WindDst.pre_offline_gui_v136p.bak"
    if (-not (Test-Path $Backup)) { Copy-Item $WindDst $Backup }
}
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_v136.m") $WindDst -Force
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_gui.m") (Join-Path $WindDir "airdropx_wind_profile_gui.m") -Force
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_v140_gui.m") (Join-Path $WindDir "airdropx_wind_profile_v140_gui.m") -Force

Write-Host "[4/5] Installing Python GUI into offline_gui_v136p..."
$GuiDst = Join-Path $ProjectRoot "offline_gui_v136p"
New-Item -ItemType Directory -Force -Path $GuiDst | Out-Null
foreach ($name in @("main.py","requirements.txt","launch_gui.bat","build_python_exe.ps1","install_python_env.ps1","README_OFFLINE_GUI.md")) {
    Copy-Item (Join-Path $Here $name) $GuiDst -Force
}
foreach ($dir in @("core","ui","config","docs","tests")) {
    $dst = Join-Path $GuiDst $dir
    if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
    Copy-Item (Join-Path $Here $dir) $GuiDst -Recurse -Force
}

Write-Host "[5/5] Sanity checks..."
$required = @(
    (Join-Path $ProjectRoot "matlab\airdrop\airdropx_wind_airdrop_mission_v136p.m"),
    (Join-Path $ProjectRoot "matlab\gui_bridge\airdropx_gui_backend_entry_v136p.m"),
    (Join-Path $ProjectRoot "matlab\gui_bridge\airdropx_gui_prepare_model_v136p.m"),
    (Join-Path $ProjectRoot "matlab\wind\airdropx_wind_profile_gui.m")
)
foreach ($p in $required) { if (-not (Test-Path $p)) { throw "Install incomplete, missing: $p" } }

Write-Host ""
Write-Host "Installed AirdropX Offline GUI with v1.3.6-Paper as the DEFAULT backend." -ForegroundColor Green
Write-Host "GUI: $GuiDst\launch_gui.bat"
Write-Host "If no usable PyQt6 environment is found, run: $GuiDst\install_python_env.ps1"
Write-Host "v1.4.0 remains selectable as an EXPERIMENTAL backend when its mission files are present."
Write-Host "New arbitrary H/V points still require the existing Physics Oracle MEX and airdropx_phys_build_bank.m."
