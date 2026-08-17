param(
  [ValidateSet('Smoke','Build')][string]$Mode='Smoke',
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [string]$Heights='200:-10:20',
  [string]$Speeds='50',
  [int]$Workers=1,
  [switch]$SkipBuild
)
$ErrorActionPreference='Stop'
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'
$sfun=Join-Path $ProjectRoot 'matlab\sfunc_jsbsim'
if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
if (-not (Test-Path $phys)) { throw "Physics MPC folder not installed: $phys" }
$build=""
if (-not $SkipBuild) {
  $build="build_sfun_airdropx_jsbsim; build_airdropx_jsbsim_oracle;"
}
if ($Mode -eq 'Smoke') {
  $cmd="cd('$($ProjectRoot.Replace("'","''"))'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); $build r=airdropx_phys_smoke(pwd); disp(r);"
} else {
  $out="matlab/results/physics_mpc_v30"
  $cmd="cd('$($ProjectRoot.Replace("'","''"))'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); $build r=airdropx_phys_build_bank(string(pwd),Heights=$Heights,Speeds=$Speeds,OutputRoot=fullfile(pwd,'$out')); disp(r.rows);"
}
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
