param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
$src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}
if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r"}}
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not (Test-Path $bank)){throw "Validated 95-point bank missing: $bank"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v061_"+$stamp)
New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {
  $old=Join-Path $dst $_.Name
  if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}
  Copy-Item -Force $_.FullName $old
}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_preview_altitude_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_preview_altitude_D.ps1')
Write-Host "Installed Physics-MPC v0.6.1 altitude-validation overlay into $ProjectRoot"
Write-Host 'Controller remains PreviewOnly v0.6.0: Q/R, scales, Np/Nc, input bounds and 0.2 s drop schedule are unchanged.'
Write-Host 'No Oracle MEX, S-Function MEX, C++ source or physics bank was modified.'
Write-Host 'Run: .\run_phys_mpc_preview_altitude_D.ps1'
