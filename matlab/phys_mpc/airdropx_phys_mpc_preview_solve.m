function sol=airdropx_phys_mpc_preview_solve(ctrl,x,warmAbs,warmSlack)
%AIRDROPX_PHYS_MPC_PREVIEW_SOLVE Solve one scheduled preview QP.
arguments
    ctrl (1,1) struct
    x (7,1) double {mustBeFinite}
    warmAbs double = []
    warmSlack double = []
end
e0=airdropx_phys_mpc_state_error(x,ctrl.Rstate(:,1));
f=ctrl.Fdx*e0+ctrl.fAffine;
if isempty(warmAbs)
    du0=zeros(ctrl.m,ctrl.N);
else
    wa=double(warmAbs);
    if ~isequal(size(wa),[ctrl.m ctrl.N]) || any(~isfinite(wa),'all')
        error("AirdropX:PhysMPC:BadPreviewWarmStart","warmAbs must be finite m-by-N absolute commands.");
    end
    du0=wa-ctrl.Rinput;
end
z0=zeros(ctrl.nz,1); z0(1:ctrl.nu)=du0(:);
if ctrl.ns>0
    if isempty(warmSlack), ws=zeros(ctrl.ns,1); else, ws=double(warmSlack(:)); end
    if numel(ws)~=ctrl.ns || any(~isfinite(ws)) || any(ws<0)
        error("AirdropX:PhysMPC:BadPreviewWarmStart","warmSlack must contain N finite nonnegative values.");
    end
    z0(ctrl.nu+(1:ctrl.ns))=ws;
end
z0=min(max(z0,ctrl.lb),ctrl.ub);
if isempty(ctrl.Aineq)
    Aineq=[]; bineq=[];
else
    Aineq=ctrl.Aineq; bineq=ctrl.bineq0+ctrl.bineqDx*e0;
end

t0=tic;
[z,fval,exitflag,output,lambda]=quadprog(ctrl.H,f,Aineq,bineq,[],[],ctrl.lb,ctrl.ub,z0,ctrl.options);
solveTime=toc(t0);
if isempty(z), z=nan(ctrl.nz,1); end
z=double(z(:));
feasible=exitflag>0 && all(isfinite(z)) && all(z>=ctrl.lb-1e-8) && all(z<=ctrl.ub+1e-8);
if feasible && ~isempty(Aineq), feasible=all(Aineq*z<=bineq+2e-8); end

Udelta=nan(ctrl.m,ctrl.N); Uabs=nan(ctrl.m,ctrl.N); slack=zeros(ctrl.N,1);
Xpred=nan(ctrl.n,ctrl.N); Epred=nan(ctrl.n,ctrl.N);
if feasible
    Udelta=reshape(z(1:ctrl.nu),ctrl.m,ctrl.N);
    Uabs=ctrl.Rinput+Udelta;
    if ctrl.ns>0, slack=z(ctrl.nu+(1:ctrl.ns)); end
    e=e0;
    for j=1:ctrl.N
        c=ctrl.Rstate(:,j)-ctrl.Rstate(:,j+1);
        e=ctrl.Astage(:,:,j)*e+ctrl.Bstage(:,:,j)*Udelta(:,j)+c;
        Epred(:,j)=e;
        Xpred(:,j)=ctrl.Rstate(:,j+1)+e;
    end
end
sol=struct();
sol.z=z; sol.Udelta=Udelta; sol.U_abs=Uabs; sol.du=Udelta(:,1); sol.u=Uabs(:,1);
sol.slack=slack; sol.slack_max=max(slack,[],'omitnan'); sol.e0=e0;
sol.fval=fval; sol.exitflag=exitflag; sol.output=output; sol.lambda=lambda;
sol.solve_time_s=solveTime; sol.feasible=feasible;
sol.predicted_errors=Epred; sol.predicted_states=Xpred;
end
