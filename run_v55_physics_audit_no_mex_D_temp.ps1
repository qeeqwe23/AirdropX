param(
    [string]$ProjectRoot = 'D:\vscode project\AirdropX',
    [string]$MatlabExe = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
if (-not (Test-Path $ProjectRoot)) { throw "Project root not found: $ProjectRoot" }

# NO MEX BUILD HERE. This intentionally uses the currently installed mexw64.
$rootEsc = $ProjectRoot.Replace("'", "''")
$cmd = "cd('$rootEsc'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); addpath('matlab/sfunc_jsbsim'); set(groot,'defaultFigureVisible','off'); r=audit_v55_cfg3_cfg4_no_mex('ProjectRoot','$rootEsc'); disp(r.report_txt);"
& $MatlabExe -batch $cmd
if ($LASTEXITCODE -ne 0) { throw "MATLAB audit failed with exit code $LASTEXITCODE" }
