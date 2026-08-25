function sol=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,gSequence,warmU)
%AIRDROPX_PHYS_MPC_SOLVE_DISTURBANCE_V130 Solve MPC with a known additive disturbance preview.
%
% gSequence is n-by-N and represents the additive state increment applied on
% each prediction transition.  For the wind-aware controller it is assembled
% from the JSBSim-calibrated wind-increment vector and a short-memory residual
% disturbance observer.  With gSequence==0 this solver is mathematically
% identical to airdropx_phys_mpc_solve.
arguments
    ctrl (1,1) struct
    x (:,1) double {mustBeFinite}
    gSequence double
    warmU double = []
end
if ~isfield(ctrl,"Gdist") || ~isfield(ctrl,"Fdist")
    error("AirdropX:WindMPC:DisturbanceNotEnabled","Call airdropx_phys_mpc_enable_disturbance_v130 first.");
end
if numel(x)~=ctrl.n, error("AirdropX:WindMPC:BadState","x must have %d elements.",ctrl.n); end
if ~isequal(size(gSequence),[ctrl.n ctrl.N]) || any(~isfinite(gSequence),'all')
    error("AirdropX:WindMPC:BadDisturbanceSequence","gSequence must be finite n-by-N.");
end
dx=airdropx_phys_mpc_state_error(double(x(:)),ctrl.xref);
gvec=double(gSequence(:));
f=ctrl.Fx*dx+ctrl.Fdist*gvec;
if isempty(warmU)
    x0=zeros(ctrl.m*ctrl.N,1);
else
    warmU=double(warmU(:));
    if numel(warmU)~=ctrl.m*ctrl.N || any(~isfinite(warmU))
        error("AirdropX:WindMPC:BadWarmStart","Warm start has wrong size or non-finite values.");
    end
    x0=warmU;
end
x0=min(max(x0,ctrl.lb),ctrl.ub);
t0=tic;
[U,fval,exitflag,output,lambda]=quadprog(ctrl.H,f,[],[],[],[],ctrl.lb,ctrl.ub,x0,ctrl.options);
solveTime=toc(t0);
if isempty(U), U=nan(ctrl.m*ctrl.N,1); end
U=double(U(:));
sol=struct();
sol.U=U; sol.du=U(1:ctrl.m); sol.u=ctrl.uref+sol.du;
sol.dx=dx; sol.g_sequence=gSequence; sol.fval=fval; sol.exitflag=exitflag; sol.output=output; sol.lambda=lambda;
sol.solve_time_s=solveTime;
sol.feasible=exitflag>0 && all(isfinite(U)) && all(U>=ctrl.lb-1e-8) && all(U<=ctrl.ub+1e-8);
base=ctrl.Phi*dx+ctrl.Gdist*gvec;
if sol.feasible
    X=base+ctrl.Gamma*U;
    sol.predicted_states=reshape(X,ctrl.n,ctrl.N);
else
    sol.predicted_states=nan(ctrl.n,ctrl.N);
end
end
