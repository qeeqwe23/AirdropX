function cfg = airdropx_mpc_setup_closed_loop_workspace(varargin)
%AIRDROPX_MPC_SETUP_CLOSED_LOOP_WORKSPACE Initialize preserved MPC SLX model.

cfg = airdropx_mpc_setup_id_workspace("UseExcitation", false, varargin{:});
end
