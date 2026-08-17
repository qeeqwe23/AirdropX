param(
  [string]$ProjectRoot='D:\vscode project\AirdropX'
)
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
$src=Join-Path $here 'matlab\phys_mpc'
$dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if (-not (Test-Path $ProjectRoot)) { throw "AirdropX project root not found: $ProjectRoot" }
if (-not (Test-Path $dst)) { throw "Install validated Physics-MPC v0.3.3 first: $dst" }
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64')
foreach($r in $required) { if (-not (Test-Path (Join-Path $dst $r))) { throw "v0.3.3 prerequisite missing: $r" } }
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
if (-not (Test-Path $bank)) { throw "Validated 95-point bank is missing: $bank" }
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
$backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v040_"+$stamp)
New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {
  $old=Join-Path $dst $_.Name
  if (Test-Path $old) { Copy-Item -Force $old (Join-Path $backup $_.Name) }
  Copy-Item -Force $_.FullName $old
}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_closed_loop_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_closed_loop_D.ps1')
Write-Host "Installed Physics-MPC v0.4.0 controller overlay into $ProjectRoot"
Write-Host "No C++ source, Oracle MEX, S-Function MEX, bank, Q/R, or v0.3.3 certification file was modified."
Write-Host "Run: .\run_phys_mpc_closed_loop_D.ps1 -Mode Smoke"
