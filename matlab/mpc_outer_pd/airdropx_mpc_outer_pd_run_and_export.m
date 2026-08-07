function result = airdropx_mpc_outer_pd_run_and_export(varargin)
%AIRDROPX_MPC_OUTER_PD_RUN_AND_EXPORT One-command MPC outer + PD inner run.
%
% Usage:
%   result = airdropx_mpc_outer_pd_run_and_export
%   result = airdropx_mpc_outer_pd_run_and_export("ShowPlots", true)
%   result = airdropx_mpc_outer_pd_run_and_export("DisableVRForBatch", true)
%
% Defaults:
%   - MPC outer loop + PD inner loop
%   - 20 s warm-up, then 30 s evaluation
%   - CARP fixed-target 4-drop mode
%   - dashboard.png, carp_cep.png, carp_cep_points.csv, summary.csv

defaults = { ...
    "StopTimeS", 30.0, ...
    "WarmupTimeS", 20.0, ...
    "DropMode", 2.0, ...
    "ShowPlots", true, ...
    "DisableVRForBatch", false};

result = airdropx_mpc_outer_pd_run_optimized_warmup(defaults{:}, varargin{:});
end
