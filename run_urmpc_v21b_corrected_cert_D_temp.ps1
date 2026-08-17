param()
$ErrorActionPreference='Stop'
$ProjectRoot='D:\vscode project\AirdropX'
$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
$TempRoot='D:\MATLAB_TEMP\AirdropX_urmpc_v21b'
$TempDir=Join-Path $TempRoot 'temp';$Cache=Join-Path $TempRoot 'cache';$Codegen=Join-Path $TempRoot 'codegen'
New-Item -ItemType Directory -Force -Path $TempRoot,$TempDir,$Cache,$Codegen | Out-Null
$env:TEMP=$TempDir;$env:TMP=$TempDir
function E([string]$s){$s.Replace("'","''")}
$pr=E $ProjectRoot;$ca=E $Cache;$cg=E $Codegen
$cmd=@"
cd('$pr');addpath('matlab');addpath('matlab/mpc');addpath('matlab/mpc_auto');
Simulink.fileGenControl('set','CacheFolder','$ca','CodeGenFolder','$cg','createDir',true);
fprintf('\n=== UR-MPC v2.1B CORRECTED LPV OFFLINE CERTIFICATION ===\n');
r=airdropx_urmpc_v21_corrected_certify('ProjectRoot',pwd);
disp(r.summary);disp(r.projected_validation(:,{'speed_mps','cfg_id','validation_ratio','status'}));
fprintf('candidate_bank=%s\n',char(r.candidate_bank));
"@
Write-Host '=== AirdropX UR-MPC v2.1B: corrected LPV offline certification ==='
Write-Host 'NOTE: this script does NOT run the V50 nonlinear flight and does NOT overwrite the v2.0 flight bank.'
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
