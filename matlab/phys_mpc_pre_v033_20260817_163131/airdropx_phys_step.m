function [xnext,diag] = airdropx_phys_step(x,u,p)
%AIRDROPX_PHYS_STEP Exact JSBSim discrete transition over MPC sample time.
% x=[h_m;Va_mps;gamma_rad;theta_rad;q_radps;N1;N2]
% u=[elevator_abs_norm; throttle_norm]
arguments
    x (7,1) double {mustBeFinite}
    u (2,1) double {mustBeFinite}
    p (1,1) struct
end
if ~isfield(p,"cfgId") || isempty(p.cfgId)
    error("AirdropX:PhysMPC:MissingCfgId","p.cfgId is required.");
end
if ~isfield(p,"fuelScale") || isempty(p.fuelScale), p.fuelScale=1.0; end
if ~isfield(p,"Ts") || isempty(p.Ts), p.Ts=0.1; end
validateattributes(p.cfgId,{'double'},{'scalar','real','finite','integer','>=',0,'<=',4},mfilename,'p.cfgId');
validateattributes(p.fuelScale,{'double'},{'scalar','real','finite','>=',0,'<=',1.2},mfilename,'p.fuelScale');
validateattributes(p.Ts,{'double'},{'scalar','real','finite','positive'},mfilename,'p.Ts');
if u(1)<-1 || u(1)>1, error("AirdropX:PhysMPC:BadElevator","Elevator must be in [-1,1]."); end
if u(2)<0 || u(2)>1, error("AirdropX:PhysMPC:BadThrottle","Throttle must be in [0,1]."); end
[xnext,diag]=airdropx_jsbsim_oracle_mex("eval",x,u,p.cfgId,p.fuelScale,p.Ts);
if any(~isfinite(xnext))
    error("AirdropX:PhysMPC:NonfiniteStep","Physics oracle returned a non-finite next state.");
end
end
