param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path; $src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}; if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_mpc_state_error.m','airdropx_phys_mpc_preview_solve.m','airdropx_phys_mpc_preview_shift_warmstart.m','airdropx_phys_oracle_selftest.m')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r. Install the validated v0.9.0 stack first."}}
$master=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $master)){throw "v0.8.2 475-point usable master bank missing: $master"}
$interval=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v090_continuous_interval_validation\interval_validation.mat'; if(-not (Test-Path $interval)){Write-Warning "v0.9 interval evidence not found; v1.0 can run but final report cannot cite/recheck that reference."}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v100_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_runtime_command_v100_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_runtime_command_v100_D.ps1')
Copy-Item -Force (Join-Path $here 'run_phys_mpc_runtime_command_point_v100_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_runtime_command_point_v100_D.ps1')
Write-Host "Installed Physics-MPC v1.0.0 runtime-command overlay into $ProjectRoot"
Write-Host 'No MEX/C++ is replaced. Q/R and fixed Np=Nc=100 remain unchanged.'
Write-Host 'Run: .\run_phys_mpc_runtime_command_v100_D.ps1'
