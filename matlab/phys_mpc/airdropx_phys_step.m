function [xnext,diag] = airdropx_phys_step(x,u,p)
%AIRDROPX_PHYS_STEP Exact JSBSim discrete transition over MPC sample time.
% x=[h_m;Va_mps;gamma_rad;theta_rad;q_radps;N1;N2]
% u=[elevator_abs_norm; throttle_norm]
arguments
    x (7,1) double
    u (2,1) double
    p.cfgId (1,1) double
    p.fuelScale (1,1) double = 1.0
    p.Ts (1,1) double = 0.1
end
assert(p.cfgId==round(p.cfgId) && p.cfgId>=0 && p.cfgId<=4,"cfgId must be integer 0..4");
[xnext,diag]=airdropx_jsbsim_oracle_mex("eval",x,u,p.cfgId,p.fuelScale,p.Ts);
end
