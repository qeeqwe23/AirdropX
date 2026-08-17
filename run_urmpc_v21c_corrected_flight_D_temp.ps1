param(
    [string]$ProjectRoot = 'D:\vscode project\AirdropX',
    [double]$SpeedMps = 50,
    [double]$TargetAltitudeM = 200
)
$ErrorActionPreference='Stop'
$env:TEMP='D:\MATLAB_TEMP\AirdropX_urmpc_v21c\temp'
$env:TMP=$env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
$matlab='D:\MATLAB R2026a\matlab\bin\matlab.exe'
if (!(Test-Path $matlab)) { throw "MATLAB not found: $matlab" }
$pr=$ProjectRoot.Replace("'","''")
$cmd="cd('$pr'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); r=airdropx_urmpc_v21_corrected_flight_ablation('ProjectRoot','$pr','SpeedsMps',$SpeedMps,'TargetAltitudeM',$TargetAltitudeM); disp(r.summary);"
& $matlab -batch $cmd
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
