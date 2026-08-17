param(
  [ValidateSet('Smoke')][string]$Mode='Smoke',
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [double]$Duration=30
)
$ErrorActionPreference='Stop'
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'
if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
if (-not (Test-Path $phys)) { throw "Physics MPC folder not installed: $phys" }
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
if (-not (Test-Path $bank)) { throw "Validated v0.3.3 bank missing: $bank" }
$mex=Join-Path $phys 'airdropx_jsbsim_oracle_mex.mexw64'
if (-not (Test-Path $mex)) { throw "Validated Oracle MEX missing: $mex" }
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v040_closed_loop'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
$log=Join-Path $out ("closed_loop_terminal_"+$stamp+".txt")
$rootEsc=$ProjectRoot.Replace("'","''")
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_closed_loop_smoke(string(pwd),Duration_s=$Duration); disp(r.gate); disp(r.metrics);"
Write-Host "Logging to $log"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath $log
$code=$LASTEXITCODE
exit $code
