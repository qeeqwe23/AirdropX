param(
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$MatlabExe = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
if (-not (Test-Path $MatlabExe)) {
    throw "MATLAB not found: $MatlabExe"
}

$rootEsc = $ProjectRoot.Replace("'", "''")
$parts = @(
    "cd('$rootEsc')"
    "addpath('matlab')"
    "addpath('matlab/mpc')"
    "addpath('matlab/mpc_auto')"
    "addpath('matlab/sfunc_jsbsim')"
    "set(groot,'defaultFigureVisible','off')"
    "setenv('AIRDROPX_FILEGEN_ROOT','D:\AXC\v55audit')"
)

if (-not $SkipBuild) {
    $parts += "bdclose('all')"
    $parts += "clear mex"
    $parts += "build_sfun_airdropx_jsbsim"
}
$parts += "r=audit_v55_cfg3_cfg4_longitudinal('ProjectRoot','$rootEsc')"
$parts += "disp(r.report_txt)"

$batch = ($parts -join '; ') + ';'
Write-Host '=== AirdropX V55 physics audit ==='
Write-Host "Project: $ProjectRoot"
if ($SkipBuild) {
    Write-Host 'MEX rebuild: skipped by request'
} else {
    Write-Host 'MEX rebuild: enabled (required after applying this patch)'
}
& $MatlabExe -batch $batch
if ($LASTEXITCODE -ne 0) {
    throw "MATLAB audit failed with exit code $LASTEXITCODE"
}
