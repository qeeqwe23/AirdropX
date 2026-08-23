param(
 [string]$ProjectRoot='D:\vscode project\AirdropX',
 [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
 [ValidateSet('calm','tailwind_5','headwind_5','tailwind_12','headwind_12','step_bidirectional','ramp_minus10_plus10','sine_longitudinal')]
 [string]$Scenario='step_bidirectional'
)
$ErrorActionPreference='Stop'
function E([string]$s){return $s.Replace("'","''")}
$dur=@{calm=30;tailwind_5=30;headwind_5=30;tailwind_12=30;headwind_12=30;step_bidirectional=35;ramp_minus10_plus10=35;sine_longitudinal=45}[$Scenario]
$out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v112_wind_point\"+$Scenario); New-Item -ItemType Directory -Force -Path $out | Out-Null
$cmd="cd('$(E $ProjectRoot)'); addpath('matlab'); addpath('matlab/wind'); addpath('matlab/sfunc_jsbsim'); r=airdropx_wind_estimation_entry_v111(string(pwd),OutputRoot=string('$(E $out)'),ScenarioName=string('$Scenario'),Duration_s=$dur); disp(r);"
& $MatlabExe -batch $cmd
