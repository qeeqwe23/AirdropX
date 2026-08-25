param(
 [string]$ProjectRoot='D:\vscode project\AirdropX',
 [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
 [ValidateSet('calm','tailwind_5','headwind_5','tailwind_12','headwind_12','step_bidirectional','ramp_minus10_plus10','sine_longitudinal')][string]$Scenario='step_bidirectional',
 [ValidateSet('wind_mpc_aware','legacy_mpc_aware','legacy_mpc_nowind')][string]$Mode='wind_mpc_aware'
)
$ErrorActionPreference='Stop'; function E([string]$s){$s.Replace("'","''")}
$root=E $ProjectRoot; $cal=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_calibration\wind_disturbance_model_v130.mat'; if(-not(Test-Path $cal)){throw 'Run full v1.3.0 runner once to create wind disturbance calibration.'}
$out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v130_wind_disturbance_point\${Scenario}_${Mode}"); New-Item -ItemType Directory -Force -Path $out|Out-Null
$aware=if($Mode -eq 'legacy_mpc_nowind'){'false'}else{'true'}; $wm=if($Mode -eq 'wind_mpc_aware'){'true'}else{'false'}; $outEsc=E $out; $calEsc=E $cal
$cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); r=airdropx_wind_airdrop_entry_v130(string(pwd),OutputRoot=string('$outEsc'),ScenarioName=string('$Scenario'),UseWindCompensation=$aware,UseWindDisturbanceMPC=$wm,WindCalibrationPath=string('$calEsc')); disp(r);"
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
