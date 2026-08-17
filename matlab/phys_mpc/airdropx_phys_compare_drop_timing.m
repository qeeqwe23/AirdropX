function report=airdropx_phys_compare_drop_timing(varargin) %#ok<INUSD,STOUT>
%AIRDROPX_PHYS_COMPARE_DROP_TIMING Disabled in v0.5.2.
% v0.5.1 ran both JSBSim scenarios sequentially in one MATLAB process.  A
% completed scenario could stall while the persistent Oracle was being torn
% down, preventing the next scenario from starting.  v0.5.2 deliberately
% isolates each scenario in a separate MATLAB child process. Use:
%   .\run_phys_mpc_drop_timing_compare_D.ps1 -Mode Compare
error("AirdropX:PhysMPC:InProcessCompareDisabled", ...
    "v0.5.2 disables in-process sequential scenario execution. Use run_phys_mpc_drop_timing_compare_D.ps1 so each JSBSim scenario has an isolated MATLAB process.");
end
