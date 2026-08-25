param(
    [Parameter(Mandatory=$true)][string]$ProjectRoot
)
$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = (Resolve-Path $ProjectRoot).Path
if (-not (Test-Path (Join-Path $ProjectRoot "matlab"))) { throw "Not an AirdropX root: $ProjectRoot" }

Write-Host "[1/4] Installing v1.4.0 overlay with its official backup-aware installer..."
$V140Installer = Join-Path $Here "v140_overlay\install_wind_disturbance_airdrop_v140.ps1"
& $V140Installer -ProjectRoot $ProjectRoot

Write-Host "[2/4] Installing GUI MATLAB bridge..."
$BridgeDst = Join-Path $ProjectRoot "matlab\gui_bridge"
New-Item -ItemType Directory -Force -Path $BridgeDst | Out-Null
Copy-Item (Join-Path $Here "matlab\gui_bridge\*.m") $BridgeDst -Force

Write-Host "[3/4] Extending v1.3.6 wind profile with isolated gui_custom branch..."
$WindDst = Join-Path $ProjectRoot "matlab\wind\airdropx_wind_profile_v136.m"
if (Test-Path $WindDst) {
    $Backup = "$WindDst.pre_offline_gui_v140.bak"
    if (-not (Test-Path $Backup)) { Copy-Item $WindDst $Backup }
}
Copy-Item (Join-Path $Here "matlab\wind\airdropx_wind_profile_v136.m") $WindDst -Force
Copy-Item (Join-Path $Here "matlab\gui_bridge\airdropx_wind_profile_v140_gui.m") (Join-Path $ProjectRoot "matlab\wind\airdropx_wind_profile_v140_gui.m") -Force

Write-Host "[4/4] Installing Python GUI into offline_gui_v140..."
$GuiDst = Join-Path $ProjectRoot "offline_gui_v140"
New-Item -ItemType Directory -Force -Path $GuiDst | Out-Null
foreach ($name in @("main.py","requirements.txt","launch_gui.bat","build_python_exe.ps1")) { Copy-Item (Join-Path $Here $name) $GuiDst -Force }
foreach ($dir in @("core","ui","config","docs","tests")) { Copy-Item (Join-Path $Here $dir) $GuiDst -Recurse -Force }

Write-Host "Installed. Launch: $GuiDst\launch_gui.bat"
Write-Host "Note: new arbitrary H/V points require the existing Physics Oracle MEX and airDropX physics builder in this project."
