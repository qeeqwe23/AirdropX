param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'; $Here=Split-Path -Parent $MyInvocation.MyCommand.Path
if(-not(Test-Path $ProjectRoot)){throw "ProjectRoot not found: $ProjectRoot"}
function SamePath([string]$a,[string]$b){return [System.IO.Path]::GetFullPath($a).TrimEnd('\') -ieq [System.IO.Path]::GetFullPath($b).TrimEnd('\')}
function CopySafe([string]$src,[string]$dst){if(SamePath $src $dst){Write-Host "Skip self-copy: $src"; return}; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dst)|Out-Null; Copy-Item $src $dst -Force}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\_backup_v132_"+$stamp)
$matlabSrc=Join-Path $Here 'matlab'; $files=Get-ChildItem -Path $matlabSrc -Recurse -File
foreach($f in $files){
  $rel=$f.FullName.Substring($matlabSrc.Length).TrimStart('\','/'); $dst=Join-Path (Join-Path $ProjectRoot 'matlab') $rel
  if(-not(SamePath $f.FullName $dst) -and (Test-Path $dst)){$bak=Join-Path $backup $rel; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $bak)|Out-Null; Copy-Item $dst $bak -Force}
  CopySafe $f.FullName $dst
}
CopySafe (Join-Path $Here 'run_wind_disturbance_airdrop_v132_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_v132_D.ps1')
CopySafe (Join-Path $Here 'run_wind_disturbance_airdrop_point_v132_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_point_v132_D.ps1')
Write-Host "Installed Physics-MPC v1.3.2 into $ProjectRoot"
if(Test-Path $backup){Write-Host "Backed up replaced MATLAB files to $backup"}
Write-Host 'v1.1.3 wind estimator and v1.3.0 physical Gw calibration remain unchanged.'
Write-Host 'v1.3.2 adds confidence gating, causal release-state filtering, unified gust recovery, and actuator diagnostics.'
Write-Host 'Run: .\run_wind_disturbance_airdrop_v132_D.ps1'
