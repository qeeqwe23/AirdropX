param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'; $Here=Split-Path -Parent $MyInvocation.MyCommand.Path
if(-not(Test-Path $ProjectRoot)){throw "ProjectRoot not found: $ProjectRoot"}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\_backup_v130_"+$stamp)
$files=Get-ChildItem -Path (Join-Path $Here 'matlab') -Recurse -File
foreach($f in $files){$rel=$f.FullName.Substring((Join-Path $Here 'matlab').Length).TrimStart('\','/'); $dst=Join-Path (Join-Path $ProjectRoot 'matlab') $rel; if(Test-Path $dst){$bak=Join-Path $backup $rel; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $bak)|Out-Null; Copy-Item $dst $bak -Force}; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dst)|Out-Null; Copy-Item $f.FullName $dst -Force}
Copy-Item (Join-Path $Here 'run_wind_disturbance_airdrop_v130_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_v130_D.ps1') -Force
Copy-Item (Join-Path $Here 'run_wind_disturbance_airdrop_point_v130_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_point_v130_D.ps1') -Force
Write-Host "Installed Physics-MPC v1.3.0 wind-disturbance-aware airdrop into $ProjectRoot"
if(Test-Path $backup){Write-Host "Backed up replaced MATLAB files to $backup"}
Write-Host 'Wind estimator v1.1.3 parameters are unchanged.'
Write-Host 'Existing v0.3.3 Oracle MEX is untouched; v1.3.0 reuses the separate v1.2.1 continuous-wind Oracle.'
Write-Host 'Run: .\run_wind_disturbance_airdrop_v130_D.ps1'
