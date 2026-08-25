function W=airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140(bankPath,logPaths,outPath,opts)
%AIRDROPX_PHYS_MPC_WIND_DISTURBANCE_FIT_FLIGHTLOG_V140 Fit Gw without JSBSim truth.
%
% Fits the v1.3.0-compatible scalar longitudinal wind-increment channel from
% synchronized onboard logs:
%   dx(k+1) = A dx(k) + B du(k) + Gw * (w_hat(k+1)-w_hat(k)) + residual
%
% Required log columns are the v1.4.0 estimated state / command columns. Exact
% plant truth is neither required nor read. For real-aircraft use, supply logs
% synchronized to the controller sample time and a flight-validated model bank.
arguments
    bankPath (1,1) string
    logPaths (1,:) string
    outPath (1,1) string
    opts.H (1,1) double {mustBeFinite,mustBePositive} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite,mustBePositive} = 1.0
    opts.Horizon (1,1) double {mustBeInteger,mustBePositive} = 100
    opts.MinWindConfidence (1,1) double {mustBeNonnegative} = 0.35
    opts.MinAbsDeltaWind_mps (1,1) double {mustBeNonnegative} = 0.05
    opts.MinSamplesPerCfg (1,1) double {mustBeInteger,mustBePositive} = 40
    opts.Ridge (1,1) double {mustBeNonnegative} = 1e-6
end
if opts.MinWindConfidence>1, error("AirdropX:WindMPC:BadConfidence","MinWindConfidence must be <=1."); end
if ~isfile(bankPath), error("AirdropX:WindMPC:MissingBank","Missing model bank: %s",bankPath); end
if isempty(logPaths), error("AirdropX:WindMPC:NoLogs","At least one flight log is required."); end
for i=1:numel(logPaths), if ~isfile(logPaths(i)), error("AirdropX:WindMPC:MissingLog","Missing log: %s",logPaths(i)); end, end
sched=airdropx_phys_mpc_build_cfg_schedule(bankPath,opts.H,opts.V,opts.FuelScale,Horizon=opts.Horizon);
required=["cfg","h_est_m","Va_est_mps","gamma_est_rad","theta_est_rad","q_est_radps","N1_est","N2_est","elevator_cmd","throttle_cmd","wind_est_mps"];
for i=1:numel(logPaths)
    T=readtable(logPaths(i),TextType="string");
    missing=required(~ismember(required,string(T.Properties.VariableNames)));
    if ~isempty(missing), error("AirdropX:WindMPC:BadFlightLog","%s is missing columns: %s",logPaths(i),strjoin(missing,", ")); end
    if ~ismember("wind_confidence",string(T.Properties.VariableNames)), T.wind_confidence=ones(height(T),1); end
end
stateCols=["h_est_m","Va_est_mps","gamma_est_rad","theta_est_rad","q_est_radps","N1_est","N2_est"];
rows=table(); cfgData=cell(5,1);
for cfg=0:4
    ctrl=sched.models{cfg+1}.ctrl;
    Y=[]; DW=[]; WT=[];
    for lp=1:numel(logPaths)
        T=readtable(logPaths(lp),TextType="string");
        if ~ismember("wind_confidence",string(T.Properties.VariableNames)), T.wind_confidence=ones(height(T),1); end
        if height(T)<2, continue; end
        for k=1:height(T)-1
            if double(T.cfg(k))~=cfg || double(T.cfg(k+1))~=cfg, continue; end
            xk=zeros(7,1); x1=zeros(7,1);
            for j=1:7, xk(j)=double(T.(stateCols(j))(k)); x1(j)=double(T.(stateCols(j))(k+1)); end
            uk=[double(T.elevator_cmd(k));double(T.throttle_cmd(k))];
            wk=double(T.wind_est_mps(k)); w1=double(T.wind_est_mps(k+1)); dw=w1-wk;
            conf=min(double(T.wind_confidence(k)),double(T.wind_confidence(k+1)));
            if any(~isfinite([xk;x1;uk;wk;w1;conf])) || conf<opts.MinWindConfidence || abs(dw)<opts.MinAbsDeltaWind_mps, continue; end
            dxk=airdropx_phys_mpc_state_error(xk,ctrl.xref); dx1=airdropx_phys_mpc_state_error(x1,ctrl.xref); du=uk-ctrl.uref;
            y=dx1-ctrl.A*dxk-ctrl.B*du;
            Y=[Y;y.']; DW=[DW;dw]; WT=[WT;max(conf,1e-3)]; %#ok<AGROW>
        end
    end
    n=size(Y,1);
    if n<opts.MinSamplesPerCfg
        error("AirdropX:WindMPC:InsufficientFlightData","cfg%d has only %d qualified samples; need %d.",cfg,n,opts.MinSamplesPerCfg);
    end
    X=[ones(n,1),DW]; sw=sqrt(WT(:)); Xw=X.*sw; Yw=Y.*sw;
    R=diag([1e-12,opts.Ridge]); beta=(Xw.'*Xw+R)\(Xw.'*Yw);
    intercept=beta(1,:).'; Gw=beta(2,:).';
    residual=Y-(X*beta); residualNorm=max(abs(residual)./ctrl.stateScale.',[],2);
    residualP95=localPercentile(residualNorm,95);
    if any(~isfinite(Gw)) || Gw(2)>=0
        error("AirdropX:WindMPC:BadFlightGw","cfg%d fitted Gw is invalid; expected positive tailwind increment to reduce Va, Gw(Va)=%.6g.",cfg,Gw(2));
    end
    ctrlAug=airdropx_phys_mpc_enable_disturbance_v130(ctrl,Gw); self=localSolverSelftest(ctrlAug);
    cfgData{cfg+1}=struct("cfg",cfg,"Gw",Gw,"probe_mps",[],"G_by_probe",[],"normalized_probe_spread",NaN, ...
        "solver_selftest",self,"source","flightlog_sensor_estimates","sample_count",n,"fit_intercept",intercept,"fit_residual_p95_normalized",residualP95);
    rows=[rows;table(cfg,n,Gw(1),Gw(2),Gw(3),Gw(4),Gw(5),Gw(6),Gw(7),residualP95,self.zero_disturbance_u_error,self.pass, ...
        'VariableNames',{'cfg','sample_count','Gw_h','Gw_Va','Gw_gamma','Gw_theta','Gw_q','Gw_N1','Gw_N2','fit_residual_p95_norm','zero_disturbance_u_error','pass'})]; %#ok<AGROW>
end
W=struct();
% Keep the v1.3.0 compatibility token so the existing loader can consume the
% file without changing the certified runtime interface.
W.version="Physics-MPC v1.3.0-compatible v1.4.0 flight-log delta-wind calibration";
W.source="sensor_estimated_flight_logs_not_JSBSim_truth"; W.H=opts.H; W.V=opts.V; W.FuelScale=opts.FuelScale; W.Horizon=opts.Horizon; W.probes_mps=[]; W.cfg=cfgData; W.table=rows; W.log_paths=logPaths;
W.pass=height(rows)==5 && all(rows.pass) && all(rows.sample_count>=opts.MinSamplesPerCfg) && all(rows.Gw_Va<0);
if ~W.pass, error("AirdropX:WindMPC:FlightLogCalibrationFailed","Flight-log wind disturbance calibration failed."); end
[outDir,~,~]=fileparts(outPath); if outDir~="" && ~isfolder(outDir), mkdir(outDir); end
save(outPath,"W","-v7.3"); writetable(rows,fullfile(outDir,"wind_disturbance_flightlog_fit_v140.csv"));
fid=fopen(fullfile(outDir,"wind_disturbance_flightlog_fit_v140_summary.txt"),"w"); if fid>=0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,"%s\nsource=%s\npass=%d\nH=%.6g\nV=%.6g\n",W.version,W.source,W.pass,W.H,W.V);
    for i=1:height(rows), fprintf(fid,"cfg%d samples=%d GwVa=%+.9g residualP95=%.6g zeroUerr=%.3g pass=%d\n",rows.cfg(i),rows.sample_count(i),rows.Gw_Va(i),rows.fit_residual_p95_norm(i),rows.zero_disturbance_u_error(i),rows.pass(i)); end
end
disp(rows);
end

function s=localSolverSelftest(ctrl)
x=ctrl.xref+0.05*ctrl.stateScale; warm=zeros(ctrl.m*ctrl.N,1);
a=airdropx_phys_mpc_solve(ctrl,x,warm); g=zeros(ctrl.n,ctrl.N); b=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,g,warm);
err=max(abs(a.u-b.u)); g(:,1)=ctrl.Gw*0.1; c=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,g,warm);
s=struct("zero_disturbance_u_error",err,"nonzero_feasible",c.feasible,"pass",a.feasible && b.feasible && c.feasible && err<=1e-10);
end

function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
q=1+(numel(x)-1)*p/100; a=floor(q); b=ceil(q); if a==b, y=x(a); else, y=x(a)+(q-a)*(x(b)-x(a)); end
end
