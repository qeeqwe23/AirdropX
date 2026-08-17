$ErrorActionPreference = 'Stop'
$root = 'D:\vscode project\AirdropX'
$matlab = 'D:\MATLAB R2026a\matlab\bin\matlab.exe'
& $matlab -batch "cd('$root'); addpath('matlab'); addpath('matlab/mpc'); addpath('matlab/mpc_auto'); addpath('matlab/sfunc_jsbsim'); mx=fullfile('$root','matlab','sfunc_jsbsim',['sfun_airdropx_jsbsim.' mexext]); if ~isfile(mx), error('Existing JSBSim MEX is required: %s',mx); end; fprintf('[PHYS-MPC] NO-MEX mode: reusing existing %s\n',mx); airdropx_physics_mpc_direct_cfg_smoke('ProjectRoot','$root'); r=airdropx_physics_mpc_build('ProjectRoot','$root','ExistingV32Root','matlab/results/mpc_auto_v32_clean','OutputRoot','matlab/results/mpc_physics_v1','SpeedNodesMps',[45;50;55],'ParallelWorkers',3,'ReuseExistingNodeResults',true,'AllowDeterministicRetrim',true); disp(r.bank_mat);"
exit $LASTEXITCODE
