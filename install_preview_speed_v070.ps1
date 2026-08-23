param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path; $src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}; if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_build_vertex.m','airdropx_phys_preflight.m','airdropx_phys_math_selftest.m')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r"}}
$baseBank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not (Test-Path $baseBank)){throw "Validated V50 95-point bank missing: $baseBank"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v070_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_preview_speed_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_preview_speed_D.ps1')
Write-Host "Installed Physics-MPC v0.7.0 speed-pilot overlay into $ProjectRoot"
Write-Host 'Altitude acceptance basis: v0.6.1 produced 19/19 nonlinear PreviewOnly PASS at V=50 m/s.'
Write-Host 'This package does NOT alter Q/R, gates, Oracle MEX, S-Function MEX, or the existing V50 physics bank.'
Write-Host 'It builds only new V40/V45/V55/V60 pilot vertices, merges V50 certified vertices, then runs 15 nonlinear missions.'
Write-Host 'Run: .\run_phys_mpc_preview_speed_D.ps1'
