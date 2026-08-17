function test_condensed_qp_math
% Pure-MATLAB seven-state/two-input check of the actual v0.4.0 QP functions.
A=diag([0.995 0.97 0.96 0.95 0.92 0.90 0.88]);
B=[0.01 0.00; 0.02 0.04; 0.03 0.00; 0.04 0.01; 0.08 0.00; 0.00 0.10; 0.00 0.08];
Q=diag([0.06 0.2 10 5 8 1e-8 1e-8]);
R=diag([40 15]);
[K,P]=dlqr(A,B,Q,R);
rho=max(abs(eig(A-B*K)));
v=struct();
v.lin=struct('Ad',A,'Bd',B,'converged',true);
v.Q=Q; v.R=R;
v.terminal=struct('P',P,'K',K,'Np',25,'rho',rho);
v.trim=struct('x',[200;50;0;0.1;0;70;85],'u',[-0.3;0.6],'pass',true);
v.qrMeta=struct('StateScale',[4;2;deg2rad(3);deg2rad(5);deg2rad(2);10;10], ...
                'InputScale',[0.15;0.20]);
ctrl=airdropx_phys_mpc_condense(v,Horizon=25);
r=airdropx_phys_mpc_qp_selftest(ctrl);
assert(r.pass,'QP/LQR self-test did not pass.');
assert(r.max_abs_error<2e-6,'QP first move differs from LQR.');
fprintf('test_condensed_qp_math PASS, error=%.3g rho=%.6f\n',r.max_abs_error,rho);
end
