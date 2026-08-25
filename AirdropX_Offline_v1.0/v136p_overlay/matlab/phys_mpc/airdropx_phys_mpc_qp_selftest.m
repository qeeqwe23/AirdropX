function report=airdropx_phys_mpc_qp_selftest(ctrl)
%AIRDROPX_PHYS_MPC_QP_SELFTEST Verify condensed-QP math against terminal LQR.
% With P equal to the DARE solution and no active constraints, the first MPC
% move must equal -K*dx. This catches prediction-matrix/sign/bias mistakes
% before the nonlinear plant is ever advanced.
scale=1e-2*ctrl.stateScale;
scale(6:7)=1e-3*ctrl.stateScale(6:7);
direction=[1;-0.8;0.5;-0.4;0.3;0.2;-0.15];
dx=scale.*direction;
for k=1:12
    duLqr=-ctrl.K*dx;
    uLqr=ctrl.uref+duLqr;
    if all(uLqr>ctrl.umin+1e-5) && all(uLqr<ctrl.umax-1e-5), break; end
    dx=0.5*dx;
end
if ~(all(uLqr>ctrl.umin+1e-5) && all(uLqr<ctrl.umax-1e-5))
    error("AirdropX:PhysMPC:SelftestCouldNotFindInterior","Could not find an interior LQR self-test perturbation.");
end
x=ctrl.xref+dx;
sol=airdropx_phys_mpc_solve(ctrl,x,zeros(ctrl.m*ctrl.N,1));
err=sol.du-duLqr;
report=struct("pass",false,"dx",dx,"du_lqr",duLqr,"du_qp",sol.du, ...
    "max_abs_error",norm(err,inf),"qp_exitflag",sol.exitflag,"qp_time_s",sol.solve_time_s);
report.pass=sol.feasible && report.max_abs_error<=2e-6;
if ~report.pass
    error("AirdropX:PhysMPC:QPSelftestFailed", ...
        "QP/LQR self-test failed: exit=%d max|du_qp-du_lqr|=%.3g.",sol.exitflag,report.max_abs_error);
end
end
