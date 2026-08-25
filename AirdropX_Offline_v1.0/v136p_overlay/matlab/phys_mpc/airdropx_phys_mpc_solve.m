function sol=airdropx_phys_mpc_solve(ctrl,x,warmU)
%AIRDROPX_PHYS_MPC_SOLVE Solve one box-constrained receding-horizon QP.
arguments
    ctrl (1,1) struct
    x (7,1) double {mustBeFinite}
    warmU double = []
end
dx=airdropx_phys_mpc_state_error(x,ctrl.xref);
f=ctrl.Fx*dx;
if isempty(warmU)
    x0=zeros(ctrl.m*ctrl.N,1);
else
    warmU=double(warmU(:));
    if numel(warmU)~=ctrl.m*ctrl.N || any(~isfinite(warmU))
        error("AirdropX:PhysMPC:BadWarmStart","Warm start has the wrong size or non-finite values.");
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
sol.dx=dx; sol.fval=fval; sol.exitflag=exitflag; sol.output=output; sol.lambda=lambda;
sol.solve_time_s=solveTime;
sol.feasible=exitflag>0 && all(isfinite(U)) && all(U>=ctrl.lb-1e-8) && all(U<=ctrl.ub+1e-8);
if sol.feasible
    X=ctrl.Phi*dx+ctrl.Gamma*U;
    sol.predicted_states=reshape(X,ctrl.n,ctrl.N);
else
    sol.predicted_states=nan(ctrl.n,ctrl.N);
end
end
