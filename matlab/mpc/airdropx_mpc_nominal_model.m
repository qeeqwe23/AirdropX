function model = airdropx_mpc_nominal_model(cfg)
%AIRDROPX_MPC_NOMINAL_MODEL Compatibility wrapper for the zero-drop model.

if nargin < 1 || isempty(cfg)
    cfg = airdropx_mpc_config("IncludeModel", false);
end
bank = airdropx_mpc_greybox_model(cfg);
model = bank{1};
end
