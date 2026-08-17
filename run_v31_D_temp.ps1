param(
    [string]$ProjectRoot = "D:\vscode project\AirdropX",
    [string]$MatlabExe = "D:\MATLAB R2026a\matlab\bin\matlab.exe",
    [string]$TempRoot = "D:\MATLAB_TEMP",
    [string]$ShortRoot = "D:\AXC",
    [int]$Workers = 3,
    [int]$MaxTaskCalls = 1
)

$ErrorActionPreference = "Stop"
$Workers = [Math]::Max(1,[Math]::Min(3,$Workers))
if (-not (Test-Path -LiteralPath $ProjectRoot)) { throw "ProjectRoot not found: $ProjectRoot" }
if (-not (Test-Path -LiteralPath $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
if (-not (Test-Path -LiteralPath $TempRoot)) { New-Item -ItemType Directory -Force -Path $TempRoot | Out-Null }
if (-not (Test-Path -LiteralPath $ShortRoot)) { New-Item -ItemType Directory -Force -Path $ShortRoot | Out-Null }

$drive = Get-PSDrive -Name ([IO.Path]::GetPathRoot($TempRoot).Substring(0,1))
if ($drive.Free -lt 15GB) { throw "Less than 15 GB free on TEMP drive." }

$runId = "v31_3_" + (Get-Date -Format "yyyyMMdd_HHmmss") + "_" + ([guid]::NewGuid().ToString("N").Substring(0,6))
$runShort = Join-Path $ShortRoot $runId
$mainCache = Join-Path $runShort "m\c"
$mainCodegen = Join-Path $runShort "m\g"
New-Item -ItemType Directory -Force -Path $mainCache,$mainCodegen | Out-Null

$env:TEMP = $TempRoot
$env:TMP = $TempRoot
$env:AIRDROPX_SHORT_FILEGEN_ROOT = $runShort
$env:AIRDROPX_PARALLEL_WORKERS = "$Workers"

Write-Host "============================================================"
Write-Host "AirdropX v31.3 dynamic-reference + continuous speed-scheduler architecture"
Write-Host "ProjectRoot : $ProjectRoot"
Write-Host "Workers     : $Workers (hard cap 3)"
Write-Host "TEMP        : $TempRoot"
Write-Host "Short root  : $runShort"
Write-Host "Task calls  : $MaxTaskCalls per invocation"
Write-Host "============================================================"

$escapedRoot = $ProjectRoot.Replace("'","''")
$escapedCache = $mainCache.Replace("'","''")
$escapedCodegen = $mainCodegen.Replace("'","''")
$cmd = @"
cd('$escapedRoot');
addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto');
if isfolder('matlab/sfunc_jsbsim'), addpath('matlab/sfunc_jsbsim'); end
Simulink.fileGenControl('set','CacheFolder','$escapedCache','CodeGenFolder','$escapedCodegen','createDir',true);
r = airdropx_v31_train_envelope( ...
    'ProjectRoot',pwd, ...
    'OutputRoot','matlab/results/mpc_auto_v31', ...
    'KnowledgeBankRoot','matlab/results/mpc_auto_v31_knowledge_bank', ...
    'AnchorVerifiedRoot','matlab/results/mpc_auto_200m_all_cfg_v16', ...
    'ReferenceAltitudeM',200, ...
    'AnchorAirspeedMps',50, ...
    'QualificationHeightsM',[200;150;100;50;20], ...
    'SpeedCurriculumMps',[45;55;40;60;35;65;30;70], ...
    'UseParallel',true,'ParallelWorkers',$Workers, ...
    'MaxTaskCallsPerInvocation',$MaxTaskCalls);
disp(r);
"@

try {
    & $MatlabExe -batch $cmd
    $exitCode = $LASTEXITCODE
} finally {
    # The v30.2/v31 worker setup can create subdirectories beneath this short
    # run root. Removing only this runId prevents stale slprj/cache pollution.
    if (Test-Path -LiteralPath $runShort) {
        Remove-Item -LiteralPath $runShort -Recurse -Force -ErrorAction SilentlyContinue
    }
}
exit $exitCode
