param(
  [string]$ProjectRoot='D:\vscode project\AirdropX'
)
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
$src=Join-Path $here 'matlab\phys_mpc'
$dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if (-not (Test-Path $ProjectRoot)) { throw "AirdropX project root not found: $ProjectRoot" }
if (-not (Test-Path $dst)) { throw "Install validated Physics-MPC v0.3.3/v0.4.0 first: $dst" }
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_mpc_condense.m','airdropx_phys_mpc_solve.m')
foreach($r in $required) { if (-not (Test-Path (Join-Path $dst $r))) { throw "Prerequisite missing: $r" } }
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
if (-not (Test-Path $bank)) { throw "Validated 95-point bank is missing: $bank" }
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
$backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v050_"+$stamp)
New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {
  $old=Join-Path $dst $_.Name
  if (Test-Path $old) { Copy-Item -Force $old (Join-Path $backup $_.Name) }
  Copy-Item -Force $_.FullName $old
}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_four_drop_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_four_drop_D.ps1')
Write-Host "Installed Physics-MPC v0.5.0 four-drop mission overlay into $ProjectRoot"
Write-Host "No C++ source, Oracle MEX, S-Function MEX, physics_bank.mat, unified Q/R, or v0.3.3 certification output was modified."
Write-Host "Run: .\run_phys_mpc_four_drop_D.ps1 -Mode Mission"
