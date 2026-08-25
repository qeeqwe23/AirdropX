function W=airdropx_phys_mpc_wind_disturbance_calibrate_v130(projectRoot,bankPath,outPath,opts)
%AIRDROPX_PHYS_MPC_WIND_DISTURBANCE_CALIBRATE_V130 Identify a true JSBSim wind-increment map.
%
% For every cfg, restart the persistent nonlinear Oracle at the exact trim with
% zero wind, then apply +/- longitudinal wind increments for one MPC sample.
% Central differences identify Gw in
%   dx(k+1)=A dx(k)+B du(k)+Gw*DeltaWind(k)
% This is a physical JSBSim experiment, not a guessed E matrix.
arguments
    projectRoot (1,1) string
    bankPath (1,1) string
    outPath (1,1) string
    opts.H (1,1) double = 200
    opts.V (1,1) double = 50
    opts.FuelScale (1,1) double = 1.0
    opts.Probes_mps (1,:) double = [0.5 1.0 2.0]
    opts.Horizon (1,1) double = 100
end
if exist("airdropx_jsbsim_wind_oracle_mex","file")~=3
    error("AirdropX:WindMPC:MissingOracle","airdropx_jsbsim_wind_oracle_mex is required.");
end
sched=airdropx_phys_mpc_build_cfg_schedule(bankPath,opts.H,opts.V,opts.FuelScale,Horizon=opts.Horizon);
airdropx_phys_wind_oracle_init_v121(projectRoot); c=onCleanup(@()localClose()); %#ok<NASGU>
P=double(opts.Probes_mps(:).'); if any(P<=0) || any(~isfinite(P)), error("AirdropX:WindMPC:BadProbe","Probes must be positive finite values."); end
rows=table(); cfgData=cell(5,1);
for cfg=0:4
    ctrl=sched.models{cfg+1}.ctrl; Ts=sched.models{cfg+1}.vertex.p.Ts;
    G=zeros(ctrl.n,numel(P)); zeroResidual=zeros(ctrl.n,numel(P));
    for j=1:numel(P)
        h=P(j);
        airdropx_jsbsim_wind_oracle_mex("start_continuous",ctrl.xref,ctrl.uref,cfg,opts.FuelScale,0);
        [xp,~]=airdropx_jsbsim_wind_oracle_mex("step_continuous",ctrl.uref,cfg,opts.FuelScale,+h,Ts);
        airdropx_jsbsim_wind_oracle_mex("start_continuous",ctrl.xref,ctrl.uref,cfg,opts.FuelScale,0);
        [xm,~]=airdropx_jsbsim_wind_oracle_mex("step_continuous",ctrl.uref,cfg,opts.FuelScale,-h,Ts);
        airdropx_jsbsim_wind_oracle_mex("start_continuous",ctrl.xref,ctrl.uref,cfg,opts.FuelScale,0);
        [xz,~]=airdropx_jsbsim_wind_oracle_mex("step_continuous",ctrl.uref,cfg,opts.FuelScale,0,Ts);
        ep=airdropx_phys_mpc_state_error(xp,ctrl.xref); em=airdropx_phys_mpc_state_error(xm,ctrl.xref); ez=airdropx_phys_mpc_state_error(xz,ctrl.xref);
        G(:,j)=(ep-em)/(2*h); zeroResidual(:,j)=ez;
    end
    [~,j1]=min(abs(P-1.0)); Gw=G(:,j1);
    spread=max(abs(G-Gw),[],2); spreadNorm=max(spread./ctrl.stateScale);
    if any(~isfinite(Gw)) || Gw(2)>=-0.20
        error("AirdropX:WindMPC:BadWindMap","cfg%d wind map invalid; expected positive tailwind step to reduce Va, Gw(Va)=%.6g.",cfg,Gw(2));
    end
    ctrlAug=airdropx_phys_mpc_enable_disturbance_v130(ctrl,Gw);
    self=localSolverSelftest(ctrlAug);
    cfgData{cfg+1}=struct("cfg",cfg,"Gw",Gw,"probe_mps",P,"G_by_probe",G,"normalized_probe_spread",spreadNorm,"solver_selftest",self);
    rows=[rows;table(cfg,Gw(1),Gw(2),Gw(3),Gw(4),Gw(5),Gw(6),Gw(7),spreadNorm,self.zero_disturbance_u_error,self.pass, ...
        'VariableNames',{'cfg','Gw_h','Gw_Va','Gw_gamma','Gw_theta','Gw_q','Gw_N1','Gw_N2','probe_spread_norm','zero_disturbance_u_error','pass'})]; %#ok<AGROW>
end
W=struct(); W.version="Physics-MPC v1.3.0 JSBSim delta-wind disturbance calibration"; W.H=opts.H; W.V=opts.V; W.FuelScale=opts.FuelScale; W.Horizon=opts.Horizon; W.probes_mps=P; W.cfg=cfgData; W.table=rows;
W.pass=height(rows)==5 && all(rows.pass) && all(isfinite(rows.Gw_Va)) && all(rows.Gw_Va<0);
if ~W.pass, error("AirdropX:WindMPC:CalibrationFailed","Wind disturbance calibration failed."); end
[outDir,~,~]=fileparts(outPath); if outDir~="" && ~isfolder(outDir), mkdir(outDir); end
save(outPath,"W","-v7.3"); writetable(rows,fullfile(outDir,"wind_disturbance_calibration.csv"));
fid=fopen(fullfile(outDir,"wind_disturbance_calibration_summary.txt"),"w"); if fid>=0
    cc=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,"%s\npass=%d\nH=%.6g\nV=%.6g\n",W.version,W.pass,W.H,W.V);
    for i=1:height(rows), fprintf(fid,"cfg%d GwVa=%+.9g probeSpreadNorm=%.9g zeroUerr=%.3g pass=%d\n",rows.cfg(i),rows.Gw_Va(i),rows.probe_spread_norm(i),rows.zero_disturbance_u_error(i),rows.pass(i)); end
end
disp(rows);
end
function s=localSolverSelftest(ctrl)
x=ctrl.xref+0.05*ctrl.stateScale; warm=zeros(ctrl.m*ctrl.N,1);
a=airdropx_phys_mpc_solve(ctrl,x,warm); g=zeros(ctrl.n,ctrl.N); b=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,g,warm);
err=max(abs(a.u-b.u)); g(:,1)=ctrl.Gw*0.1; c=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,g,warm);
s=struct("zero_disturbance_u_error",err,"nonzero_feasible",c.feasible,"pass",a.feasible && b.feasible && c.feasible && err<=1e-10);
end
function localClose()
try, airdropx_jsbsim_wind_oracle_mex("close"); catch, end
end
