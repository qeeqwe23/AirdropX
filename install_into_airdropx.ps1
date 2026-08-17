param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [switch]$InstallSFunctionSource
)
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
$srcPhys=Join-Path $here 'matlab\phys_mpc'
$dstPhys=Join-Path $ProjectRoot 'matlab\phys_mpc'
if (-not (Test-Path $ProjectRoot)) { throw "AirdropX project root not found: $ProjectRoot" }
if (-not (Test-Path $srcPhys)) { throw "Package phys_mpc source missing: $srcPhys" }

# Source-level rollback point. Existing files are copied, not moved/deleted.
if (Test-Path $dstPhys) {
  $stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
  $backup=Join-Path $ProjectRoot ("matlab\phys_mpc_pre_v033_"+$stamp)
  Copy-Item -Recurse -Force $dstPhys $backup
  Write-Host "Backed up current phys_mpc sources to $backup"
} else {
  New-Item -ItemType Directory -Force -Path $dstPhys | Out-Null
}
Copy-Item -Force (Join-Path $srcPhys '*') $dstPhys
Copy-Item -Force (Join-Path $here 'run_phys_mpc_D.ps1') (Join-Path $ProjectRoot 'run_phys_mpc_D.ps1')

# v0.3 does NOT need a new S-Function to run the Physics Oracle. Protect the
# current working S-Function source unless explicitly requested.
if ($InstallSFunctionSource) {
  $dstSfun=Join-Path $ProjectRoot 'matlab\sfunc_jsbsim'
  $srcCpp=Join-Path $here 'matlab\sfunc_jsbsim\sfun_airdropx_jsbsim.cpp'
  $srcBuild=Join-Path $here 'matlab\sfunc_jsbsim\build_sfun_airdropx_jsbsim.m'
  $dstCpp=Join-Path $dstSfun 'sfun_airdropx_jsbsim.cpp'
  if (Test-Path $dstCpp) {
    $stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
    Copy-Item $dstCpp ($dstCpp+'.pre_phys_v033_'+$stamp+'.bak')
  }
  Copy-Item -Force $srcCpp $dstCpp
  Copy-Item -Force $srcBuild (Join-Path $dstSfun 'build_sfun_airdropx_jsbsim.m')
}
Write-Host "Installed Physics-MPC v0.3.3 sources into $ProjectRoot"
Write-Host 'First run: .\run_phys_mpc_D.ps1 -Mode Smoke'
Write-Host 'Do NOT use -SkipBuild on the first v0.3.3 run because the Oracle MEX must be rebuilt.'
