$ErrorActionPreference = 'Stop'
$root = 'D:\vscode project\AirdropX'
$matlab = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
& $matlab -batch "cd('$root'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); addpath('matlab/sfunc_jsbsim'); r=airdropx_physics_mpc_validation('ProjectRoot','$root'); disp(r.summary);"
exit $LASTEXITCODE
