param(
    [int]$Workers = 3,
    [switch]$ResetLearning
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Workers = [Math]::Max(1,[Math]::Min(3,$Workers))
$Matlab = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
if (-not (Test-Path -LiteralPath $Matlab)) { throw "MATLAB not found: $Matlab" }
$TempRoot = 'D:\MATLAB_TEMP\AirdropX_v32'
$ShortRoot = 'D:\AXC\v32'
New-Item -ItemType Directory -Force -Path $TempRoot,$ShortRoot | Out-Null
$env:TEMP = $TempRoot
$env:TMP = $TempRoot
Write-Host "=== AirdropX v32.1.4 JOINT-EQUILIBRIUM PERSISTENT AUTO-LEARNING MPC ==="
Write-Host "Project : $ProjectRoot"
Write-Host "Workers : $Workers"
Write-Host "Legacy  : DISABLED (v29/v30/v31 data are never imported)"
if ($ResetLearning) {
    Write-Host "Memory  : RESET REQUESTED - v32 memory will be erased explicitly" -ForegroundColor Yellow
} else {
    Write-Host "Memory  : RESUME - v32 knowledge/checkpoints persist across starts" -ForegroundColor Green
}
Write-Host "Policy  : stale Physics -> one trim revalidation; unchanged ID reused; changed trim -> fresh ID"
Write-Host "Trim    : joint elevator x throttle equilibrium map -> BO -> long verification"
Write-Host "Mass    : runtime context mass kept MEX-compatible; XML fuel tracked separately"
Write-Host "Output  : matlab/results/mpc_auto_v32_clean"
Write-Host "Short   : $ShortRoot"
$escaped = $ProjectRoot.Replace("'","''")
$reset = 'false'
if ($ResetLearning) { $reset = 'true' }
$cmd = "cd('$escaped'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); addpath('matlab/sfunc_jsbsim'); " +
       "r=airdropx_v32_clean_train('ProjectRoot',pwd,'ResetLearning',$reset,'Workers',$Workers,'ShortFileGenRoot','D:\AXC\v32'); " +
       "disp(r);"
& $Matlab -batch $cmd
exit $LASTEXITCODE
