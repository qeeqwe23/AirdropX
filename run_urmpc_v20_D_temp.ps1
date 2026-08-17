param(
    [int]$Workers = 1,
    [string]$Speeds = '50',
    [switch]$SkipBuild
)
$ErrorActionPreference='Stop'
$ProjectRoot='D:\vscode project\AirdropX'
$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
$TempRoot='D:\MATLAB_TEMP\AirdropX_urmpc_v20'
$ShortRoot='D:\AXC\urmpc_v20'
$BuildOut='matlab/results/mpc_physics_v1/unified_robust_mpc_v2'
$RunOut='matlab/results/mpc_physics_v1/fixed_stability_urmpc_v20'
$TempDir=Join-Path $TempRoot 'temp';$JobDir=Join-Path $TempRoot 'jobs';$MainCache=Join-Path $TempRoot 'main_cache';$MainCodegen=Join-Path $TempRoot 'main_codegen'
New-Item -ItemType Directory -Force -Path $TempRoot,$TempDir,$JobDir,$MainCache,$MainCodegen,$ShortRoot | Out-Null
$env:TEMP=$TempDir;$env:TMP=$TempDir
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
function E([string]$s){$s.Replace("'","''")}
$pr=E $ProjectRoot;$jr=E $JobDir;$mc=E $MainCache;$mg=E $MainCodegen;$sr=E $ShortRoot;$bo=E $BuildOut;$ro=E $RunOut
$vals=@($Speeds -split ','|%{[double]($_.Trim())});if($vals.Count-lt 1){throw 'At least one speed required.'};if($Workers-gt$vals.Count){$Workers=$vals.Count}
$sm='['+(($vals|%{$_.ToString('0.############',[Globalization.CultureInfo]::InvariantCulture)})-join ';')+']';$skip=if($SkipBuild){'true'}else{'false'}
$cmd=@"
cd('$pr');addpath('matlab');addpath('matlab/mpc');addpath('matlab/mpc_auto');addpath('matlab/sfunc_jsbsim');
Simulink.fileGenControl('set','CacheFolder','$mc','CodeGenFolder','$mg','createDir',true);
c=parcluster('Processes');if ~exist('$jr','dir'),mkdir('$jr');end;c.JobStorageLocation='$jr';
td=char(tempdir);fg=Simulink.fileGenControl('getConfig');fprintf('\n=== UR-MPC v2.0 CACHE PREFLIGHT ===\n');fprintf('tempdir=%s\nCache=%s\nCodeGen=%s\nJobStorage=%s\n',td,fg.CacheFolder,fg.CodeGenFolder,c.JobStorageLocation);
pp={td,char(fg.CacheFolder),char(fg.CodeGenFolder),char(c.JobStorageLocation)};for k=1:numel(pp),if ~startsWith(lower(char(pp{k})),'d:'),error('AirdropX:URMPC:CacheStillOnC','Large cache path is not D: %s',char(pp{k}));end,end
if ~$skip
    b=airdropx_urmpc_build('ProjectRoot',pwd,'OutputRoot','$bo');
end
p=airdropx_urmpc_preflight('ProjectRoot',pwd,'BuildOutputRoot','$bo');
r=airdropx_urmpc_fixed_stability_scan('ProjectRoot',pwd,'ParallelWorkers',$Workers,'SpeedsMps',$sm,'ShortFileGenRoot','$sr','JobStorageRoot','$jr','RequireDDriveTemp',true,'OutputRoot','$ro');
disp(r.summary(:,{'speed_mps','cfg_id','hard_pass','formal_pass','h_rms_m','va_rms_mps','vz_rms_mps','q_rms_dps','pitch_std_deg','mpc_fail_increment','max_elevator_deviation','max_throttle_deviation','reason'}));
"@
Write-Host '=== AirdropX UR-MPC v2.0.5: input-estimator static-white ablation ==='
Write-Host "Speeds=$Speeds Workers=$Workers SkipBuild=$($SkipBuild.IsPresent)"
Write-Host "TEMP/TMP=$TempDir"
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
