function sched=airdropx_phys_mpc_build_cfg_schedule(bankPath,H,V,fuelScale,opts)
%AIRDROPX_PHYS_MPC_BUILD_CFG_SCHEDULE Build one common MPC kernel over cfg0..4.
%
% This is NOT five independently tuned controllers. Every cfg uses the same
% Q/R, Bryson scales, hard bounds, horizon policy, and QP solver. Only the
% physics-derived trim, A/B and terminal P/K are scheduled from the certified
% JSBSim bank as payload mass/CG/Iyy changes.
arguments
    bankPath (1,1) string
    H (1,1) double {mustBeFinite}
    V (1,1) double {mustBeFinite,mustBePositive}
    fuelScale (1,1) double {mustBeFinite}
    opts.Horizon (1,1) double = NaN
    opts.EqualityTol (1,1) double {mustBePositive} = 1e-12
end
if fuelScale<0 || fuelScale>1.2
    error("AirdropX:PhysMPC:BadFuel","fuelScale must be in [0,1.2].");
end
models=cell(5,1);
rows=zeros(5,1);
Q0=[]; R0=[]; stateScale0=[]; inputScale0=[]; N0=[];
maxQDiff=0; maxRDiff=0; maxStateScaleDiff=0; maxInputScaleDiff=0;
for cfg=0:4
    [vertex,rowIndex,meta]=airdropx_phys_mpc_get_vertex(bankPath,H,V,cfg,fuelScale);
    ctrl=airdropx_phys_mpc_condense(vertex,Horizon=opts.Horizon);
    selftest=airdropx_phys_mpc_qp_selftest(ctrl);
    if cfg==0
        Q0=ctrl.Q; R0=ctrl.R; stateScale0=ctrl.stateScale; inputScale0=ctrl.inputScale; N0=ctrl.N;
    else
        maxQDiff=max(maxQDiff,norm(ctrl.Q-Q0,inf));
        maxRDiff=max(maxRDiff,norm(ctrl.R-R0,inf));
        maxStateScaleDiff=max(maxStateScaleDiff,norm(ctrl.stateScale-stateScale0,inf));
        maxInputScaleDiff=max(maxInputScaleDiff,norm(ctrl.inputScale-inputScale0,inf));
        if ctrl.N~=N0
            error("AirdropX:PhysMPC:MixedHorizon", ...
                "Common MPC kernel requires one horizon; cfg0 N=%d but cfg%d N=%d.",N0,cfg,ctrl.N);
        end
    end
    model=struct();
    model.cfg=cfg;
    model.vertex=vertex;
    model.ctrl=ctrl;
    model.qp_selftest=selftest;
    model.bank_meta=meta;
    models{cfg+1}=model;
    rows(cfg+1)=rowIndex;
end
commonPass=maxQDiff<=opts.EqualityTol && maxRDiff<=opts.EqualityTol && ...
    maxStateScaleDiff<=opts.EqualityTol && maxInputScaleDiff<=opts.EqualityTol;
if ~commonPass
    error("AirdropX:PhysMPC:ControllerNotUnified", ...
        "cfg bank is not one unified Q/R controller: dQ=%.3g dR=%.3g dStateScale=%.3g dInputScale=%.3g.", ...
        maxQDiff,maxRDiff,maxStateScaleDiff,maxInputScaleDiff);
end
sched=struct();
sched.version="Physics-MPC v0.5.2 cfg-scheduled common kernel";
sched.bank_path=bankPath;
sched.H=H; sched.V=V; sched.fuelScale=fuelScale;
sched.models=models; sched.bank_rows=rows;
sched.Q=Q0; sched.R=R0; sched.stateScale=stateScale0; sched.inputScale=inputScale0;
sched.N=N0;
sched.audit=struct("pass",commonPass,"max_Q_diff",maxQDiff,"max_R_diff",maxRDiff, ...
    "max_state_scale_diff",maxStateScaleDiff,"max_input_scale_diff",maxInputScaleDiff, ...
    "same_horizon",true,"horizon",N0,"qp_selftests_pass",all(cellfun(@(m)m.qp_selftest.pass,models)));
if ~sched.audit.qp_selftests_pass
    error("AirdropX:PhysMPC:CfgQPSelftestFailed","At least one cfg QP self-test failed.");
end
end
