function report=airdropx_phys_mpc_preview_selftest(models,opts)
%AIRDROPX_PHYS_MPC_PREVIEW_SELFTEST Structural tests before nonlinear flight.
arguments
    models (5,1) cell
    opts.QSoftPenaltyMultiplier (1,1) double {mustBePositive} = 1e4
end
fixed=models{1}.ctrl; N=fixed.N;
seq0=zeros(1,N+1);
p0=airdropx_phys_mpc_preview_condense(models,seq0,EnableQSoft=false);
dH=norm(p0.H-fixed.H,inf)/max(1,norm(fixed.H,inf));
dF=norm(p0.Fdx-fixed.Fx,inf)/max(1,norm(fixed.Fx,inf));
df=norm(p0.fAffine,inf);
dlb=norm(p0.lb-fixed.lb,inf); dub=norm(p0.ub-fixed.ub,inf);
pert=[0.10;-0.05;deg2rad(0.05);deg2rad(0.05);deg2rad(0.01);0;0];
x=fixed.xref+pert;
a=airdropx_phys_mpc_solve(fixed,x,[]);
b=airdropx_phys_mpc_preview_solve(p0,x,[],[]);
firstMoveErr=max(abs(a.u-b.u));

% Exercise an actual future 0->1->2->3->4 sequence near the four-drop window.
Ts=models{1}.vertex.p.Ts;
seq=airdropx_phys_mpc_preview_cfg_sequence(9.5,Ts,N,[10 10.2 10.4 10.6]);
pv=airdropx_phys_mpc_preview_condense(models,seq,EnableQSoft=false);
sv=airdropx_phys_mpc_preview_solve(pv,models{1}.ctrl.xref,[],[]);
if ~sv.feasible, error("AirdropX:PhysMPC:PreviewSelftestFailed","Transition preview QP is infeasible in self-test."); end
firstPredictionDirect=models{1}.ctrl.xref + models{1}.ctrl.A*zeros(7,1) + models{1}.ctrl.B*(sv.u-models{1}.ctrl.uref);
firstPredictionErr=max(abs(sv.predicted_states(:,1)-firstPredictionDirect));

% q-soft must remain exactly inactive at a constant certified trim.
pq=airdropx_phys_mpc_preview_condense(models,seq0,EnableQSoft=true,QSoftPenaltyMultiplier=opts.QSoftPenaltyMultiplier);
sq=airdropx_phys_mpc_preview_solve(pq,fixed.xref,[],[]);
qSoftTrimErr=max(abs(sq.u-fixed.uref)); qSoftSlack=max(sq.slack);
% Also exercise the actual close-spacing transition inequalities.
pqt=airdropx_phys_mpc_preview_condense(models,seq,EnableQSoft=true,QSoftPenaltyMultiplier=opts.QSoftPenaltyMultiplier);
sqt=airdropx_phys_mpc_preview_solve(pqt,models{1}.ctrl.xref,[],[]);
if sqt.feasible
    qAbs=abs(sqt.predicted_states(5,:)).';
    qSoftConstraintMargin=max(qAbs-pqt.qSoftLimitRadps-sqt.slack);
else
    qSoftConstraintMargin=Inf;
end
pass=dH<=1e-10 && dF<=1e-10 && df<=1e-12 && dlb<=1e-12 && dub<=1e-12 && ...
    a.feasible && b.feasible && firstMoveErr<=1e-9 && firstPredictionErr<=1e-10 && ...
    sq.feasible && qSoftTrimErr<=1e-9 && qSoftSlack<=1e-12 && sqt.feasible && qSoftConstraintMargin<=2e-8;
report=struct("pass",pass,"constant_H_rel",dH,"constant_F_rel",dF,"constant_affine_inf",df, ...
    "constant_lb_inf",dlb,"constant_ub_inf",dub,"fixed_vs_preview_first_u_inf",firstMoveErr, ...
    "transition_sequence",seq,"transition_first_u",sv.u,"transition_first_prediction_error",firstPredictionErr, ...
    "qsoft_trim_u_error",qSoftTrimErr,"qsoft_trim_slack",qSoftSlack,"qsoft_transition_constraint_margin",qSoftConstraintMargin, ...
    "qsoft_transition_slack_max",max(sqt.slack));
if ~pass
    error("AirdropX:PhysMPC:PreviewSelftestFailed", ...
        "Preview self-test failed: dH=%.3g dF=%.3g df=%.3g du=%.3g pred=%.3g qsoftU=%.3g qsoftS=%.3g qMargin=%.3g.", ...
        dH,dF,df,firstMoveErr,firstPredictionErr,qSoftTrimErr,qSoftSlack,qSoftConstraintMargin);
end
end
