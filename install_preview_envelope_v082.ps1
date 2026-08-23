param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path; $src=Join-Path $here 'matlab\phys_mpc'; $dst=Join-Path $ProjectRoot 'matlab\phys_mpc'
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}; if(-not (Test-Path $dst)){throw "Physics-MPC base not found: $dst"}
$required=@('airdropx_phys_step.m','airdropx_phys_oracle_init.m','airdropx_jsbsim_oracle_mex.mexw64','airdropx_phys_build_vertex.m','airdropx_phys_preflight.m','airdropx_phys_math_selftest.m')
foreach($r in $required){if(-not (Test-Path (Join-Path $dst $r))){throw "Prerequisite missing: $r"}}
$baseBank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not (Test-Path $baseBank)){throw "Validated V50 95-point bank missing: $baseBank"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_controller_pre_v082_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_phys_mpc_preview_envelope_diagnostic_v082_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_preview_envelope_diagnostic_v082_D.ps1')
Write-Host "Installed Physics-MPC v0.8.2 fixed-horizon diagnostic full-flow overlay into $ProjectRoot"
Write-Host 'This version DOES NOT relax Richardson/Q/R/mission gates. Runtime MPC uses fixed Np=Nc=100 everywhere.'
Write-Host 'Per-vertex certification FAIL is recorded; all 475 rows are attempted.'
Write-Host 'Uncertified-but-usable finite vertices may be used only by the explicit diagnostic runner.'
Write-Host 'Hard-unusable vertices are NOT replaced by neighboring models; affected H/V missions are marked MODEL_UNAVAILABLE.'
Write-Host 'Run: .\run_phys_mpc_preview_envelope_diagnostic_v082_D.ps1'
