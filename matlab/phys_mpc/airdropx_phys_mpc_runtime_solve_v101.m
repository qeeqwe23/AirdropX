function sol=airdropx_phys_mpc_runtime_solve_v101(ctrl,x,warmAbs)
%AIRDROPX_PHYS_MPC_RUNTIME_SOLVE_V101 Solve one dynamic-feasible scheduled runtime QP.
arguments
    ctrl (1,1) struct
    x (7,1) double {mustBeFinite}
    warmAbs double = []
end
e0=airdropx_phys_mpc_state_error(x,ctrl.Rstate(:,1)); f=ctrl.Fdx*e0+ctrl.fAffine;
if isempty(warmAbs)
    du0=zeros(ctrl.m,ctrl.N);
else
    wa=double(warmAbs); if ~isequal(size(wa),[ctrl.m ctrl.N]) || any(~isfinite(wa),'all'), error("AirdropX:PhysMPC:BadRuntimeWarmStart","warmAbs must be finite m-by-N absolute commands."); end
    du0=wa-ctrl.Rinput;
end
z0=min(max(du0(:),ctrl.lb),ctrl.ub); t0=tic;
[z,fval,exitflag,output,lambda]=quadprog(ctrl.H,f,[],[],[],[],ctrl.lb,ctrl.ub,z0,ctrl.options); solveTime=toc(t0);
if isempty(z), z=nan(ctrl.nu,1); end; z=double(z(:)); feasible=exitflag>0 && all(isfinite(z)) && all(z>=ctrl.lb-1e-8) && all(z<=ctrl.ub+1e-8);
Udelta=nan(ctrl.m,ctrl.N); Uabs=nan(ctrl.m,ctrl.N); Xpred=nan(ctrl.n,ctrl.N); Epred=nan(ctrl.n,ctrl.N);
if feasible
    Udelta=reshape(z,ctrl.m,ctrl.N); Uabs=ctrl.Rinput+Udelta; e=e0;
    for j=1:ctrl.N
        e=ctrl.Astage(:,:,j)*e+ctrl.Bstage(:,:,j)*Udelta(:,j)+ctrl.cstage(:,j); Epred(:,j)=e; Xpred(:,j)=ctrl.Rstate(:,j+1)+e;
    end
end
sol=struct("z",z,"Udelta",Udelta,"U_abs",Uabs,"du",Udelta(:,1),"u",Uabs(:,1),"slack",zeros(ctrl.N,1),"slack_max",0,"e0",e0, ...
    "fval",fval,"exitflag",exitflag,"output",output,"lambda",lambda,"solve_time_s",solveTime,"feasible",feasible,"predicted_errors",Epred,"predicted_states",Xpred);
end
