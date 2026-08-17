param(
    [string]$ProjectRoot = 'D:\vscode project\AirdropX',
    [string]$MatlabExe = 'D:\MATLAB R2026a\matlab\bin\matlab.exe',
    [string]$TempRoot = 'D:\MATLAB_TEMP',
    [string]$OutputRoot = 'matlab/results/mpc_auto_200m_all_cfg_v16',
    [string]$LearningBankRoot = 'matlab/results/mpc_auto_global_learning_bank',
    [double]$TargetAltitudeM = 200,
    [double]$TargetAirspeedMps = 50,
    [double]$ReferenceMassKg = 3423,
    [double]$CargoMassKg = 300,
    [int]$Workers = 5
)

$ErrorActionPreference = 'Stop'
$Workers = [Math]::Max(1,[Math]::Min(5,$Workers))
New-Item -ItemType Directory -Force -Path $TempRoot | Out-Null

# Refuse to start a long multi-worker simulation if the TEMP drive is nearly
# full. Large Simulink .dmr databases are expected under this drive.
$tempDriveName = ([System.IO.Path]::GetPathRoot($TempRoot)).TrimEnd('\\').TrimEnd(':')
$tempDrive = Get-PSDrive -Name $tempDriveName -ErrorAction Stop
$freeGB = [Math]::Round($tempDrive.Free / 1GB, 2)
Write-Host "TEMP drive free space: $freeGB GB"
if ($tempDrive.Free -lt 15GB) {
    throw "TEMP drive has less than 15 GB free. Clean $TempRoot or choose another TempRoot before running."
}

# Critical: set these BEFORE MATLAB starts. Simulink/parallel DMR temporary
# databases then go to D: instead of filling C:\Users\...\AppData\Local\Temp.
$env:TEMP = $TempRoot
$env:TMP  = $TempRoot

# Quote strings for MATLAB single-quoted literals.
function MatlabQuote([string]$s) { return $s.Replace("'", "''") }
$projectQ = MatlabQuote $ProjectRoot
$outputQ = MatlabQuote $OutputRoot
$bankQ = MatlabQuote $LearningBankRoot

$batch = @"
cd('$projectQ');
addpath('matlab');
addpath('matlab/mpc');
addpath('matlab/mpc_auto');
set(groot,'defaultFigureVisible','off');
fprintf('V29 MATLAB tempdir = %s\n', tempdir);
clientCache = fullfile(tempdir,'airdropx_v29_client_cache');
clientCodegen = fullfile(tempdir,'airdropx_v29_client_codegen');
Simulink.fileGenControl('set','CacheFolder',clientCache,'CodeGenFolder',clientCodegen,'createDir',true);
fprintf('V29 Simulink client cache = %s\n', clientCache);
r = airdropx_auto_mpc_unified_learning( ...
    'IdentifiedMat','matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat', ...
    'OutputRoot','$outputQ', ...
    'LearningBankRoot','$bankQ', ...
    'TargetAltitudeM',$TargetAltitudeM, ...
    'TargetAirspeedMps',$TargetAirspeedMps, ...
    'ReferenceMassKg',$ReferenceMassKg, ...
    'CargoMassKg',$CargoMassKg, ...
    'ConfigIds',(0:4).', ...
    'UseParallel',true, ...
    'ParallelWorkers',$Workers);
"@

Write-Host "TEMP/TMP for MATLAB: $TempRoot"
Write-Host "OutputRoot: $OutputRoot"
Write-Host "LearningBankRoot: $LearningBankRoot"
Write-Host "Mission: H=$TargetAltitudeM m, V=$TargetAirspeedMps m/s, cargo=$CargoMassKg kg"

& $MatlabExe -batch $batch
exit $LASTEXITCODE
