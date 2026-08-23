param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path; $src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}; if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_build_vertex.m','airdropx_phys_preflight.m','airdropx_phys_math_selftest.m')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r"}}
$baseBank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not (Test-Path $baseBank)){throw "Validated V50 95-point bank missing: $baseBank"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v080_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_preview_envelope_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_preview_envelope_D.ps1')
Write-Host "Installed Physics-MPC v0.8.0 full HxV envelope overlay into $ProjectRoot"
Write-Host 'Formal range: H=20:10:200 m, V=45:5:65 m/s, cfg0:4.'
Write-Host 'This package does NOT alter Q/R, controller gates, Oracle MEX, S-Function MEX, or the validated V50 physics bank.'
Write-Host 'V50 95 vertices are reused; V45/V55/V60/V65 each build a complete 95-vertex speed slice.'
Write-Host 'Then 95 nonlinear PreviewOnly missions run with the fixed 0.2 s four-drop schedule.'
Write-Host 'Run: .\run_phys_mpc_preview_envelope_D.ps1'
