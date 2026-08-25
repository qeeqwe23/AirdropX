param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [ValidateSet('calm','tailwind_5','headwind_5','tailwind_12','headwind_12','step_bidirectional','ramp_minus10_plus10','sine_longitudinal')][string]$Scenario='calm',
  [ValidateSet('wind_aware','no_wind_baseline')][string]$Mode='wind_aware',
  [int]$SensorNoiseSeed=101
)
$ErrorActionPreference='Stop'
function E([string]$s){return $s.Replace("'","''")}
$rootEsc=E $ProjectRoot; $aware=if($Mode -eq 'wind_aware'){'true'}else{'false'}
$mex=Join-Path $ProjectRoot 'matlab\phys_mpc\airdropx_jsbsim_wind_oracle_mex.mexw64'
if(-not(Test-Path $mex)){
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); build_airdropx_jsbsim_wind_oracle_v121;"
  & $MatlabExe -batch $cmd; if($LASTEXITCODE -ne 0){throw 'Wind Oracle build failed.'}
}
$out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v121_wind_airdrop_point\${Scenario}_${Mode}"); New-Item -ItemType Directory -Force -Path $out|Out-Null; $outEsc=E $out
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); r=airdropx_wind_airdrop_entry_v121(string(pwd),OutputRoot=string('$outEsc'),ScenarioName=string('$Scenario'),UseWindCompensation=$aware,Duration_s=55,TargetStart_m=1200,TargetSpacing_m=80,SensorNoiseSeed=$SensorNoiseSeed); disp(r);"
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
