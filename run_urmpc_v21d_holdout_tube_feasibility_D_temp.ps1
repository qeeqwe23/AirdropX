param(
    [switch]$FeasibilityOnly,
    [string]$ProjectRoot = 'D:\vscode project\AirdropX'
)
$ErrorActionPreference='Stop'
$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
if (!(Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
$TempRoot='D:\MATLAB_TEMP\AirdropX_urmpc_v21d'
$TempDir=Join-Path $TempRoot 'temp';$Cache=Join-Path $TempRoot 'cache';$Codegen=Join-Path $TempRoot 'codegen'
New-Item -ItemType Directory -Force -Path $TempRoot,$TempDir,$Cache,$Codegen | Out-Null
$env:TEMP=$TempDir;$env:TMP=$TempDir
function E([string]$s){$s.Replace("'","''")}
$pr=E $ProjectRoot;$ca=E $Cache;$cg=E $Codegen;$fo=if($FeasibilityOnly){'true'}else{'false'}
$cmd=@"
cd('$pr');addpath('matlab');addpath('matlab/mpc');addpath('matlab/mpc_auto');addpath('matlab/sfunc_jsbsim');
Simulink.fileGenControl('set','CacheFolder','$ca','CodeGenFolder','$cg','createDir',true);
fprintf('\n=== UR-MPC v2.1D INDEPENDENT HOLDOUT + TUBE FEASIBILITY ===\n');
if ~$fo
  h=airdropx_urmpc_v21d_holdout_validation('ProjectRoot',pwd);
  disp(h.manifest(:,{'speed_mps','cfg_id','eval_samples','trusted_samples','finite_fraction','status'}));
  if ~h.all_pass, error('AirdropX:URMPC:V21DHoldoutFailed','Independent holdout did not pass all 15 vertices.'); end
end
r=airdropx_urmpc_v21d_tube_feasibility('ProjectRoot',pwd);
disp(r.summary);disp(r.ancillary_search);
"@
Write-Host '=== AirdropX UR-MPC v2.1D: independent holdout + empirical tube feasibility ==='
if ($FeasibilityOnly) { Write-Host 'FeasibilityOnly: reusing existing independent holdout data.' }
Write-Host 'NOTE: this stage does NOT modify the flight controller and does NOT enable tube feedback.'
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
