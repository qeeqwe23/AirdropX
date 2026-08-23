function out=airdropx_phys_mpc_reweight_v132(ctrl,qMultiplier,rMultiplier)
%AIRDROPX_PHYS_MPC_REWEIGHT_V132 Rebuild only the finite-horizon stage cost.
%
% A/B, trim, terminal P, hard bounds, horizon and disturbance map are retained.
% Positive diagonal congruence scaling preserves positive semidefiniteness of
% Q/R.  The terminal P remains the certified base terminal penalty instead of
% pretending that a transient recovery cost has a newly certified terminal set.
arguments
    ctrl (1,1) struct
    qMultiplier (:,1) double {mustBeFinite,mustBePositive}
    rMultiplier (:,1) double {mustBeFinite,mustBePositive}
end
qMultiplier=double(qMultiplier(:)); rMultiplier=double(rMultiplier(:));
if numel(qMultiplier)~=ctrl.n || numel(rMultiplier)~=ctrl.m
    error("AirdropX:WindMPC:BadRecoveryWeights","Recovery multiplier dimensions are invalid.");
end
Dq=diag(sqrt(qMultiplier)); Dr=diag(sqrt(rMultiplier));
Q=Dq*ctrl.Q*Dq; R=Dr*ctrl.R*Dr;
Qbar=ctrl.Qbar;
for i=1:ctrl.N-1
    rr=(i-1)*ctrl.n+(1:ctrl.n); Qbar(rr,rr)=Q;
end
% Final block intentionally remains the original certified terminal P.
rr=(ctrl.N-1)*ctrl.n+(1:ctrl.n); Qbar(rr,rr)=ctrl.P;
Rbar=kron(eye(ctrl.N),R);
H=2*(ctrl.Gamma'*Qbar*ctrl.Gamma+Rbar); H=(H+H')/2;
[~,flag]=chol(H); if flag~=0, error("AirdropX:WindMPC:RecoveryNonConvex","Recovery QP Hessian is not positive definite."); end
out=ctrl; out.Q=Q; out.R=R; out.Qbar=Qbar; out.Rbar=Rbar; out.H=H;
out.Fx=2*(out.Gamma'*Qbar*out.Phi);
if isfield(out,"Gdist")
    out.Fdist=2*(out.Gamma'*Qbar*out.Gdist);
end
out.recovery_q_multiplier=qMultiplier;
out.recovery_r_multiplier=rMultiplier;
end
