param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
if(-not (Test-Path $ProjectRoot)){throw "AirdropX project root not found: $ProjectRoot"}
$matlab=Join-Path $ProjectRoot 'matlab'; $dst=Join-Path $matlab 'wind'; New-Item -ItemType Directory -Force -Path $dst | Out-Null
$required=@(
  (Join-Path $ProjectRoot 'matlab\airdropx_sim_params.m'),
  (Join-Path $ProjectRoot 'matlab\sfunc_jsbsim\sfun_airdropx_jsbsim.mexw64')
)
foreach($r in $required){if(-not (Test-Path $r)){throw "Prerequisite missing: $r"}}
$src=Join-Path $here 'matlab\wind'
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $matlab ("wind_pre_v110_"+$stamp); New-Item -ItemType Directory -Force -Path $backup | Out-Null
Get-ChildItem $src -File | ForEach-Object {$old=Join-Path $dst $_.Name; if(Test-Path $old){Copy-Item -Force $old (Join-Path $backup $_.Name)}; Copy-Item -Force $_.FullName $old}
Copy-Item -Force (Join-Path $here 'run_wind_estimator_validation_v110_D.ps1') (Join-Path $ProjectRoot 'run_wind_estimator_validation_v110_D.ps1')
Copy-Item -Force (Join-Path $here 'run_wind_estimator_point_v110_D.ps1') (Join-Path $ProjectRoot 'run_wind_estimator_point_v110_D.ps1')
Write-Host "Installed AirdropX v1.1.0 longitudinal wind estimator into $dst"
Write-Host "No MEX/C++/Physics-MPC files were replaced."
Write-Host "Run: .\run_wind_estimator_validation_v110_D.ps1"
