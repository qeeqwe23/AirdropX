function report=airdropx_phys_runtime_reference_selftest_v101(masterBankPath)
%AIRDROPX_PHYS_RUNTIME_REFERENCE_SELFTEST_V101 Preflight reference/defect contracts on the real master bank.
arguments
    masterBankPath (1,1) string
end
bank=airdropx_phys_runtime_prepare_bank_v101(masterBankPath); N=bank.N; Ts=bank.Ts;
% Constant command: dynamic reference must collapse back to level-flight geometry.
H0=100; V0=55; cfg0=2; Hs=repmat(H0,1,N+1); Vs=repmat(V0,1,N+1); Z=zeros(1,N+1); Cs=repmat(cfg0,1,N+1);
c0=airdropx_phys_mpc_runtime_condense_v101(bank,Hs,Vs,Z,Z,Cs);
staticGamma=max(abs(c0.gammaRef)); staticQ=max(abs(c0.qRef)); staticH=max(abs(c0.Rstate(1,:)-H0)); staticV=max(abs(c0.Rstate(2,:)-V0)); staticDefect=max(abs(c0.cstage),[],'all');
% Moving altitude command: kinematic reference identity must hold over a real horizon.
t0=55; tt=t0+(0:N)*Ts; [H,V,Hdot,Vdot]=airdropx_phys_runtime_command_profile_v101("altitude_down_v45",tt); cfg=sum(tt(:)>=[60 60.2 60.4 60.6],2)'; cfg=min(4,cfg);
c1=airdropx_phys_mpc_runtime_condense_v101(bank,H,V,Hdot,Vdot,cfg); kin=max(abs(V.*sin(c1.gammaRef)-Hdot)); dynamicGamma=max(abs(c1.gammaRef)); finiteOk=all(isfinite(c1.H),'all')&&all(isfinite(c1.Fdx),'all')&&all(isfinite(c1.fAffine));
% Static defect is allowed only at the tiny residual level of stored discrete trims.
passStatic=staticGamma<=1e-12 && staticQ<=1e-10 && staticH<=1e-10 && staticV<=1e-10 && staticDefect<=1e-3;
passDynamic=kin<=1e-10 && dynamicGamma>deg2rad(0.5) && finiteOk;
report=struct("version","Physics-MPC v1.0.1 dynamic reference self-test","pass",passStatic&&passDynamic,"static_gamma_max_rad",staticGamma,"static_q_max_rps",staticQ,"static_defect_max",staticDefect,"dynamic_kinematic_residual_max_mps",kin,"dynamic_gamma_peak_deg",rad2deg(dynamicGamma),"finite",finiteOk);
if ~report.pass, error("AirdropX:PhysMPC:RuntimeReferenceSelfTestFailed","v1.0.1 reference self-test failed: staticDefect=%g kin=%g dynamicGamma=%gdeg.",staticDefect,kin,rad2deg(dynamicGamma)); end
end
