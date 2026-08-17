$ErrorActionPreference = 'Stop'
$root = 'D:\vscode project\AirdropX'
$matlab = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
& $matlab -batch "cd('$root'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); r=airdropx_physics_mpc_preflight('ProjectRoot','$root'); if ~r.pass, error('Physics MPC preflight failed'); end"
exit $LASTEXITCODE
