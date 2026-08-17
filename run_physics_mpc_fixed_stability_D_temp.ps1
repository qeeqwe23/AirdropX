param(
    [int]$Workers = 3,
    [string]$Speeds = '45,50,55'
)
$ErrorActionPreference = 'Stop'

$ProjectRoot = 'D:\vscode project\AirdropX'
$MatlabExe   = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
$TempRoot    = 'D:\MATLAB_TEMP\AirdropX_physics_mpc_cl17'
$ShortRoot   = 'D:\AXC\phys_cl17'
$OutputRoot  = 'matlab/results/mpc_physics_v1/fixed_stability_cl17'

$TempDir     = Join-Path $TempRoot 'temp'
$JobDir      = Join-Path $TempRoot 'jobs'
$MainCache   = Join-Path $TempRoot 'main_cache'
$MainCodegen = Join-Path $TempRoot 'main_codegen'

if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }

# IMPORTANT: TEMP/TMP must be changed BEFORE MATLAB starts. Parallel process
# workers inherit these environment variables, so their DMR/temp files also
# go to D: instead of C:\Users\...\AppData\Local\Temp.
New-Item -ItemType Directory -Force -Path `
    $TempRoot,$TempDir,$JobDir,$MainCache,$MainCodegen,$ShortRoot | Out-Null
$env:TEMP = $TempDir
$env:TMP  = $TempDir

function Escape-MatlabString([string]$s) { return $s.Replace("'","''") }
$ProjectRootM = Escape-MatlabString $ProjectRoot
$TempRootM    = Escape-MatlabString $TempRoot
$TempDirM     = Escape-MatlabString $TempDir
$JobDirM      = Escape-MatlabString $JobDir
$MainCacheM   = Escape-MatlabString $MainCache
$MainCodegenM = Escape-MatlabString $MainCodegen
$ShortRootM   = Escape-MatlabString $ShortRoot
$OutputRootM  = Escape-MatlabString $OutputRoot

# Parse a comma-separated speed list. Example: -Speeds 50 -Workers 1
$SpeedVals = @($Speeds -split ',' | ForEach-Object { [double]($_.Trim()) })
if ($SpeedVals.Count -lt 1) { throw 'At least one speed is required.' }
$SpeedMatlab = '[' + (($SpeedVals | ForEach-Object { $_.ToString('0.############',[System.Globalization.CultureInfo]::InvariantCulture) }) -join ';') + ']'
if ($Workers -gt $SpeedVals.Count) { $Workers = $SpeedVals.Count }

$cmd = @"
cd('$ProjectRootM');
addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); addpath('matlab/sfunc_jsbsim');

% ---- hard redirect of all large MATLAB/Simulink/parallel caches to D: ----
cacheRoot = '$TempRootM';
jobRoot   = '$JobDirM';
mainCache = '$MainCacheM';
mainCode  = '$MainCodegenM';

Simulink.fileGenControl('set', ...
    'CacheFolder',mainCache, ...
    'CodeGenFolder',mainCode, ...
    'createDir',true);

c = parcluster('Processes');
if ~exist(jobRoot,'dir'), mkdir(jobRoot); end
c.JobStorageLocation = jobRoot;

% ---- fail-fast path verification; CL-1.7 must not write large cache to C: ----
td = char(tempdir);
fg = Simulink.fileGenControl('getConfig');
c2 = parcluster('Processes');
c2.JobStorageLocation = jobRoot;
sdiSource = '';
try
    sdiSource = char(Simulink.sdi.getSource);
catch ME
    fprintf('[CL1.7 CACHE] SDI source probe warning: %s\n',ME.message);
end
fprintf('\n=== CL-1.7 CACHE PREFLIGHT ===\n');
fprintf('tempdir            = %s\n',td);
fprintf('CacheFolder        = %s\n',fg.CacheFolder);
fprintf('CodeGenFolder      = %s\n',fg.CodeGenFolder);
fprintf('JobStorageLocation = %s\n',c2.JobStorageLocation);
fprintf('SDI source         = %s\n',sdiSource);

pathsToCheck = {td, char(fg.CacheFolder), char(fg.CodeGenFolder), char(c2.JobStorageLocation)};
labels = {'tempdir','CacheFolder','CodeGenFolder','JobStorageLocation'};
for kk=1:numel(pathsToCheck)
    pp = lower(char(pathsToCheck{kk}));
    if ~startsWith(pp,'d:')
        error('AirdropX:PhysicsMPC:CacheStillOnC','%s is not on D: %s',labels{kk},pathsToCheck{kk});
    end
end
if ~isempty(sdiSource)
    pp = lower(char(sdiSource));
    if ~startsWith(pp,'d:')
        error('AirdropX:PhysicsMPC:SdiStillOnC','SDI/DMR source is not on D: %s',sdiSource);
    end
end
fprintf('[CL1.7 CACHE] PASS: large cache/temp/job/DMR paths are on D:.\n\n');

% ---- hard MPC API smoke test before any 255-s flight ----
airdropx_physics_mpc_mpcmove_preflight( ...
    'ProjectRoot',pwd, ...
    'BankMat','matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat', ...
    'OutputRoot','$OutputRootM');

r=airdropx_physics_mpc_fixed_stability_scan( ...
    'ProjectRoot',pwd, ...
    'ParallelWorkers',$Workers, ...
    'SpeedsMps',$SpeedMatlab, ...
    'ShortFileGenRoot','$ShortRootM', ...
    'JobStorageRoot',jobRoot, ...
    'RequireDDriveTemp',true, ...
    'OutputRoot','$OutputRootM');

disp(r.summary(:,{'speed_mps','cfg_id','hard_pass','formal_pass','h_rms_m','va_rms_mps','vz_rms_mps','q_rms_dps','pitch_std_deg','h_slope_mps','va_slope_mps2','recovery_fraction','recovery_enter_increment','mpc_gate_reject_increment','authority_limit_fraction','tracking_loss_fraction','tracking_loss_count_increment','q_est_rmse_dps','settling_time_s','reason'}));
"@

Write-Host '=== Physics MPC CL-1.7: early tracking-loss recovery + TECS-style energy sharing ==='
Write-Host "Workers=$Workers  Speeds=$Speeds  Output=$OutputRoot"
Write-Host "TEMP/TMP=$TempDir"
Write-Host "ParallelJobs=$JobDir"
Write-Host "MainCache=$MainCache"
Write-Host "MainCodegen=$MainCodegen"
Write-Host "WorkerFileGen=$ShortRoot"

& $MatlabExe -batch $cmd
exit $LASTEXITCODE
