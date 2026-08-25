param(
 [string]$ProjectRoot='D:\vscode project\AirdropX',
 [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
 [ValidateSet('calm','tailwind_5','headwind_5','tailwind_12','headwind_12','step_bidirectional','ramp_minus10_plus10','sine_longitudinal')][string]$Scenario='calm',
 [ValidateSet('wind_mpc_aware','legacy_mpc_aware','legacy_mpc_nowind')][string]$Mode='wind_mpc_aware',
 [int]$Seed=0,
 [switch]$SampledRelease,
 [switch]$NoRecovery,
 [switch]$NoConfidence,
 [switch]$IdealStateFeedback,
 [switch]$IndependentCargoTruth,
 [switch]$ShowChildWindows
)
$ErrorActionPreference='Stop'
function E([string]$s){$s.Replace("'","''")}
function StartMatlab([string]$cmd,[string]$stdout,[string]$stderr){
  $sp=@{FilePath=$MatlabExe;ArgumentList="-batch `"$cmd`"";Wait=$true;PassThru=$true;RedirectStandardOutput=$stdout;RedirectStandardError=$stderr}
  if(-not $ShowChildWindows){$sp.WindowStyle='Hidden'}
  return Start-Process @sp
}
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$root=E $ProjectRoot
$cal=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_calibration\wind_disturbance_model_v130.mat'
if(-not(Test-Path $cal)){throw 'Run the full v1.3.6-Paper runner once, or provide the existing v1.3.0 wind-disturbance calibration.'}
# Point tests now use the SAME deterministic seeds as the formal manifest unless explicitly overridden.
$seedMap=@{calm=101;tailwind_5=102;headwind_5=103;tailwind_12=104;headwind_12=105;step_bidirectional=106;ramp_minus10_plus10=107;sine_longitudinal=108}
if($Seed -eq 0){$Seed=[int]$seedMap[$Scenario]}
$duration=if($Scenario -eq 'sine_longitudinal'){60}else{55}

# v1.3.6-Paper reuses the validated v1.3.5 mass-refresh Oracle. Reuse it;
# only rebuild if the MEX itself is missing; a missing marker triggers selftest only.
$mex=Join-Path $ProjectRoot 'matlab\phys_mpc\airdropx_jsbsim_wind_oracle_mex.mexw64'
$oracleMarker=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v135_oracle_selftest.ok'
if(-not(Test-Path $mex)){
  $tmp=Join-Path $ProjectRoot 'matlab\results\v135_oracle_build'; New-Item -ItemType Directory -Force -Path $tmp|Out-Null
  $so=Join-Path $tmp 'stdout.txt'; $se=Join-Path $tmp 'stderr.txt'
  $cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); build_airdropx_jsbsim_wind_oracle_v121;"
  $p=StartMatlab $cmd $so $se; if($p.ExitCode -ne 0){if(Test-Path $so){Get-Content $so};if(Test-Path $se){Get-Content $se};throw 'v1.3.5-compatible wind Oracle build failed.'}
}
if(-not(Test-Path $oracleMarker)){
  $tmp=Join-Path $ProjectRoot 'matlab\results\v135_oracle_selftest'; New-Item -ItemType Directory -Force -Path $tmp|Out-Null
  $so=Join-Path $tmp 'stdout.txt'; $se=Join-Path $tmp 'stderr.txt'
  $cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_wind_oracle_selftest_v121(string(pwd)); assert(contains(string(r.version),'mass-refresh-v135')); disp(r);"
  $p=StartMatlab $cmd $so $se; if($p.ExitCode -ne 0){if(Test-Path $so){Get-Content $so};if(Test-Path $se){Get-Content $se};throw 'v1.3.5-compatible wind Oracle selftest failed; rebuild is required.'}
  Set-Content -Encoding ASCII -Path $oracleMarker -Value ("pass "+(Get-Date -Format o))
}

$variant='formal'
if($SampledRelease){$variant+='_sampled'}
if($NoRecovery){$variant+='_noRecovery'}
if($NoConfidence){$variant+='_noConfidence'}
if($IdealStateFeedback){$variant+='_idealState'}
if($IndependentCargoTruth){$variant+='_independentCargo'}
$out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v136p_paper_point\${Scenario}_${Mode}_${variant}")
New-Item -ItemType Directory -Force -Path $out|Out-Null
$aware=if($Mode -eq 'legacy_mpc_nowind'){'false'}else{'true'}; $wm=if($Mode -eq 'wind_mpc_aware'){'true'}else{'false'}; $frac=if($SampledRelease){'false'}else{'true'}; $rec=if($NoRecovery){'false'}else{'true'}; $conf=if($NoConfidence){'false'}else{'true'}; $outEsc=E $out; $calEsc=E $cal; $paperSensor=if($IdealStateFeedback){'false'}else{'true'}; $indCargo=if($IndependentCargoTruth){'true'}else{'false'}
$cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); addpath('matlab/avionics'); r=airdropx_wind_airdrop_entry_v136p(string(pwd),OutputRoot=string('$outEsc'),ScenarioName=string('$Scenario'),UseWindCompensation=$aware,UseWindDisturbanceMPC=$wm,UseFractionalRelease=$frac,UseWindConfidenceGate=$conf,UseUnifiedGustRecovery=$rec,UsePaperSensorModel=$paperSensor,UseIndependentCargoTruth=$indCargo,WindCalibrationPath=string('$calEsc'),Duration_s=$duration,SensorNoiseSeed=$Seed); disp(r);"
$stdout=Join-Path $out 'point_stdout.txt'; $stderr=Join-Path $out 'point_stderr.txt'; Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
$p=StartMatlab $cmd $stdout $stderr
if(Test-Path $stdout){Get-Content $stdout}; if(Test-Path $stderr){Get-Content $stderr | Write-Host}
exit $p.ExitCode
