param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path; $src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}; if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_preview_four_drop_closed_loop.m','airdropx_phys_mpc_build_cfg_schedule.m','airdropx_phys_mpc_preview_condense.m')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r. Install/run v0.8.2 first."}}
$master=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $master)){throw "v0.8.2 475-point usable master bank missing: $master"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v090_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_continuous_interval_v090_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_continuous_interval_v090_D.ps1')
Copy-Item -Force (Join-Path $here 'run_phys_mpc_interval_point_v090_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_interval_point_v090_D.ps1')
Write-Host "Installed Physics-MPC v0.9.0 continuous HxV interval overlay into $ProjectRoot"
Write-Host 'No MEX/C++ is replaced. Formal Richardson certification is not a runtime gate in this interval-validation release.'
Write-Host 'Runtime interpolation: bilinear A/B/xref/uref; unified Q/R; DARE P/K recomputed; fixed Np=Nc=100.'
Write-Host 'Run full interval: .\run_phys_mpc_continuous_interval_v090_D.ps1'
Write-Host 'Run arbitrary point: .\run_phys_mpc_interval_point_v090_D.ps1 -H 137.4 -V 58.2'
