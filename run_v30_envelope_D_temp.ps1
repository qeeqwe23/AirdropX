param(
    [string]$ProjectRoot = 'D:\vscode project\AirdropX',
    [string]$MatlabExe = 'D:\MATLAB R2026a\matlab\bin\matlab.exe',
    [string]$TempRoot = 'D:\MATLAB_TEMP',
    [string]$WorkerRoot = 'D:\AXC',
    [int]$Workers = 3
)

$ErrorActionPreference = 'Stop'
$Workers = [Math]::Max(1, [Math]::Min(3, $Workers))

if (-not (Test-Path -LiteralPath $MatlabExe)) { throw "MATLAB executable not found: $MatlabExe" }
if (-not (Test-Path -LiteralPath $ProjectRoot)) { throw "Project root not found: $ProjectRoot" }
if (-not (Test-Path -LiteralPath $TempRoot)) { New-Item -ItemType Directory -Path $TempRoot -Force | Out-Null }
if (-not (Test-Path -LiteralPath $WorkerRoot)) { New-Item -ItemType Directory -Path $WorkerRoot -Force | Out-Null }

# v30.2: use a very short per-launch Simulink file-generation root. This
# avoids Windows MAX_PATH failures inside slprj/_sfprj on process workers.
$sessionTag = ([Guid]::NewGuid().ToString('N')).Substring(0, 6)
$FileGenSessionRoot = Join-Path $WorkerRoot ("r" + $sessionTag)
New-Item -ItemType Directory -Path $FileGenSessionRoot -Force | Out-Null
$env:AIRDROPX_FILEGEN_ROOT = $FileGenSessionRoot
Write-Host "Short Simulink filegen root: $FileGenSessionRoot"

$driveName = [System.IO.Path]::GetPathRoot($TempRoot).TrimEnd('\').TrimEnd(':')
$drive = Get-PSDrive -Name $driveName -ErrorAction SilentlyContinue
if ($null -ne $drive) {
    $freeGB = [Math]::Round($drive.Free / 1GB, 2)
    Write-Host "TEMP drive free space: $freeGB GB"
    if ($drive.Free -lt 15GB) { throw "TEMP drive has less than 15 GB free: $freeGB GB" }
}

$env:TEMP = $TempRoot
$env:TMP = $TempRoot

$proj = $ProjectRoot.Replace("'", "''")
$temp = $TempRoot.Replace("'", "''")
$filegen = $FileGenSessionRoot.Replace("'", "''")
$matlabCmd = @"
cd('$proj');
addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); if isfolder('matlab/sfunc_jsbsim'), addpath('matlab/sfunc_jsbsim'); end;
setenv('AIRDROPX_FILEGEN_ROOT','$filegen');
cacheDir=fullfile('$filegen','m','c'); codegenDir=fullfile('$filegen','m','g');
if ~isfolder(cacheDir), mkdir(cacheDir); end; if ~isfolder(codegenDir), mkdir(codegenDir); end;
Simulink.fileGenControl('set','CacheFolder',cacheDir,'CodeGenFolder',codegenDir,'createDir',true);
r = airdropx_auto_envelope_train( ...
    'ProjectRoot',pwd, ...
    'AnchorOutputRoot','matlab/results/mpc_auto_200m_all_cfg_v16', ...
    'EnvelopeRoot','matlab/results/mpc_auto_flight_envelope_v30', ...
    'LearningBankRoot','matlab/results/mpc_auto_global_learning_bank', ...
    'PlantBankRoot','matlab/results/mpc_auto_global_plant_bank', ...
    'AltitudeMinM',20,'AltitudeMaxM',200, ...
    'NominalAirspeedMps',50, ...
    'SpeedSearchMinMps',30,'SpeedSearchMaxMps',70, ...
    'UseParallel',true,'ParallelWorkers',$Workers);
disp(r);
"@

Write-Host "Starting AirdropX v30.6.2 baseline-safe mission recovery envelope training..."
try {
    & $MatlabExe -batch $matlabCmd
    $code = $LASTEXITCODE
}
finally {
    # MATLAB has exited, so this launch-specific cache can be removed safely.
    if (Test-Path -LiteralPath $FileGenSessionRoot) {
        Remove-Item -LiteralPath $FileGenSessionRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
}
exit $code
