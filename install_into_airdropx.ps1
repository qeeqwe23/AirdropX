param(
  [string]$ProjectRoot = 'D:\vscode project\AirdropX'
)
$ErrorActionPreference='Stop'
$here=Split-Path -Parent $MyInvocation.MyCommand.Path
$dstPhys=Join-Path $ProjectRoot 'matlab\phys_mpc'
$dstSfun=Join-Path $ProjectRoot 'matlab\sfunc_jsbsim'
New-Item -ItemType Directory -Force -Path $dstPhys | Out-Null
Copy-Item -Force (Join-Path $here 'matlab\phys_mpc\*') $dstPhys
$srcSfun=Join-Path $here 'matlab\sfunc_jsbsim\sfun_airdropx_jsbsim.cpp'
$dstCpp=Join-Path $dstSfun 'sfun_airdropx_jsbsim.cpp'
if (Test-Path $dstCpp) {
  $bak=$dstCpp+'.pre_phys_v02.bak'
  if (-not (Test-Path $bak)) { Copy-Item $dstCpp $bak }
}
Copy-Item -Force $srcSfun $dstCpp
Write-Host "Installed Physics-MPC v0.2 sources into $ProjectRoot"
Write-Host 'Next: build_sfun_airdropx_jsbsim; build_airdropx_jsbsim_oracle; airdropx_phys_smoke(pwd)'
