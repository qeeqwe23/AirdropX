AirdropX Physics MPC v1.3.1 NO-MEX
==================================
Purpose
-------
Continue the v1.3 direct-configuration physical trim/model build WITHOUT
recompiling sfun_airdropx_jsbsim.mexw64.

Method
------
For offline trim/linearization only, cfg1..cfg4 use a temporary aircraft
folder copied from aircraft/MQ9_Reaper.  The first cfg cargo point-mass
weights are set to zero in that private XML, so the EXISTING MEX loads the
correct target mass/CG before its normal JSBSim initialization/auto-trim.
No C++ source or build helper is included in this patch.

Older MEX builds keep bookkeeping mass/CG/drop_count variables that do not
know the temporary XML already removed cargo.  Those bookkeeping outputs are
NOT used to certify the physical configuration.  The generated XML itself is
verified before simulation; after the run MATLAB relabels only the OFFLINE
metadata (drop_count/mass/cg) to match that verified physical XML.

Parallel safety
---------------
Every simulation gets both a unique aircraft folder and a unique IC XML.
Three process workers therefore do not share aircraft or reset files.
Temporary aircraft folders are deleted automatically.

Run
---
1) .\run_physics_mpc_preflight_D_temp.ps1
2) .\run_physics_mpc_build_D_temp.ps1

The build script NEVER invokes mex/build_sfun_airdropx_jsbsim.
It only requires the existing matlab/sfunc_jsbsim/sfun_airdropx_jsbsim.mexw64.

Cache version: physics_mpc_v1_3_1_nomex
Old v1.2/v1.3 node caches are not reused.
