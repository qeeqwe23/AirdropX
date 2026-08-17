param(
    [string]$ProjectRoot = "D:\vscode project\AirdropX",
    [string]$MatlabExe = "D:\MATLAB R2026a\matlab\bin\matlab.exe",
    [string]$TempRoot = "D:\MATLAB_TEMP",
    [string]$ShortRoot = "D:\AXC"
)
$ErrorActionPreference = "Stop"
if (-not (Test-Path -LiteralPath $ProjectRoot)) { throw "ProjectRoot not found: $ProjectRoot" }
if (-not (Test-Path -LiteralPath $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
New-Item -ItemType Directory -Force -Path $TempRoot,$ShortRoot | Out-Null
$runId = "v31_3_dynamic_" + (Get-Date -Format "yyyyMMdd_HHmmss") + "_" + ([guid]::NewGuid().ToString("N").Substring(0,6))
$runShort = Join-Path $ShortRoot $runId
$cache = Join-Path $runShort "m\c"; $codegen = Join-Path $runShort "m\g"
New-Item -ItemType Directory -Force -Path $cache,$codegen | Out-Null
$env:TEMP=$TempRoot; $env:TMP=$TempRoot; $env:AIRDROPX_SHORT_FILEGEN_ROOT=$runShort; $env:AIRDROPX_PARALLEL_WORKERS="1"
Write-Host "AirdropX v31.3 dynamic H/V validation (cfg0, no training)"
$root=$ProjectRoot.Replace("'","''"); $c=$cache.Replace("'","''"); $g=$codegen.Replace("'","''")
$cmd=@"
cd('$root'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); if isfolder('matlab/sfunc_jsbsim'),addpath('matlab/sfunc_jsbsim');end
Simulink.fileGenControl('set','CacheFolder','$c','CodeGenFolder','$g','createDir',true);
P=[0 200 50; 30 190 50; 70 190 45; 110 180 55; 160 200 48; 215 200 50];
r=airdropx_v31_3_dynamic_reference_validation('ProjectRoot',pwd, ...
 'ContextRoot','matlab/results/mpc_auto_v31/reference_contexts/H200p000_V50p000', ...
 'OutputRoot','matlab/results/mpc_auto_v31/dynamic_reference_validation/cfg0_demo', ...
 'ReferenceProfile',P,'FixedConfigId',0,'EnableDrops',false,'UseScheduler',true);
disp(r);
"@
try { & $MatlabExe -batch $cmd; $exitCode=$LASTEXITCODE }
finally { if (Test-Path -LiteralPath $runShort) { Remove-Item -LiteralPath $runShort -Recurse -Force -ErrorAction SilentlyContinue } }
exit $exitCode
