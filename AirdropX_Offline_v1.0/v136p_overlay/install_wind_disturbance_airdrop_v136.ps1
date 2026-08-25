param([string]$ProjectRoot='D:\vscode project\AirdropX')
$ErrorActionPreference='Stop'; $Here=Split-Path -Parent $MyInvocation.MyCommand.Path
if(-not(Test-Path $ProjectRoot)){throw "ProjectRoot not found: $ProjectRoot"}
function SamePath([string]$a,[string]$b){return [System.IO.Path]::GetFullPath($a).TrimEnd('\') -ieq [System.IO.Path]::GetFullPath($b).TrimEnd('\')}
function CopySafe([string]$src,[string]$dst){if(SamePath $src $dst){Write-Host "Skip self-copy: $src"; return}; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dst)|Out-Null; Copy-Item $src $dst -Force}
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'; $backup=Join-Path $ProjectRoot ("matlab\_backup_v136_"+$stamp)
$matlabSrc=Join-Path $Here 'matlab'; $files=Get-ChildItem -Path $matlabSrc -Recurse -File
foreach($f in $files){
  $rel=$f.FullName.Substring($matlabSrc.Length).TrimStart('\','/'); $dst=Join-Path (Join-Path $ProjectRoot 'matlab') $rel
  if(-not(SamePath $f.FullName $dst) -and (Test-Path $dst)){$bak=Join-Path $backup $rel; New-Item -ItemType Directory -Force -Path (Split-Path -Parent $bak)|Out-Null; Copy-Item $dst $bak -Force}
  CopySafe $f.FullName $dst
}
CopySafe (Join-Path $Here 'run_wind_disturbance_airdrop_v136_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_v136_D.ps1')
CopySafe (Join-Path $Here 'run_wind_disturbance_airdrop_point_v136_D.ps1') (Join-Path $ProjectRoot 'run_wind_disturbance_airdrop_point_v136_D.ps1')
CopySafe (Join-Path $Here 'run_base_equivalence_audit_v136_D.ps1') (Join-Path $ProjectRoot 'run_base_equivalence_audit_v136_D.ps1')
# v1.3.6 reuses the already-validated v1.3.5 mass-refresh Oracle unchanged.
# Do not delete/rebuild the mex unnecessarily; the runners self-heal only if it
# or the v1.3.5 selftest marker is genuinely missing.
Write-Host "Installed Physics-MPC v1.3.6 into $ProjectRoot"
if(Test-Path $backup){Write-Host "Backed up replaced MATLAB files to $backup"}
Write-Host 'v1.3.6 reuses the validated v1.3.5 JSBSim wind Oracle; child MATLAB windows remain hidden by default.'
Write-Host 'Recommended first: .\run_base_equivalence_audit_v136_D.ps1'
Write-Host 'Then key points: calm, headwind_12, sine_longitudinal with run_wind_disturbance_airdrop_point_v136_D.ps1'
