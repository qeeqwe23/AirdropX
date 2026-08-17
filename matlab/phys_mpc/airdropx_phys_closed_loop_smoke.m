function report=airdropx_phys_closed_loop_smoke(projectRoot,opts)
%AIRDROPX_PHYS_CLOSED_LOOP_SMOKE True nonlinear receding-horizon MPC smoke.
%
% The plant is the exact JSBSim v0.3.3 discrete oracle, not the linear model.
% The certified local A/B model is used only inside the QP predictor.
arguments
    projectRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
    opts.H (1,1) double = 200
    opts.V (1,1) double = 50
    opts.CfgId (1,1) double {mustBeInteger} = 0
    opts.FuelScale (1,1) double = 1.0
    opts.Duration_s (1,1) double {mustBePositive} = 30
    opts.InitialError (7,1) double = [2;-1;deg2rad(1);deg2rad(1);0;0;0]
    opts.Horizon (1,1) double = NaN
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxFinalLyapunovRatio (1,1) double {mustBePositive} = 0.05
    opts.MaxQpP95FractionOfTs (1,1) double {mustBePositive} = 1.0
end
if opts.CfgId<0 || opts.CfgId>4, error("AirdropX:PhysMPC:BadCfg","CfgId must be 0..4."); end
if opts.FuelScale<0 || opts.FuelScale>1.2, error("AirdropX:PhysMPC:BadFuel","FuelScale must be in [0,1.2]."); end
if ~(isnan(opts.Horizon) || (isfinite(opts.Horizon) && opts.Horizon==round(opts.Horizon) && opts.Horizon>=1))
    error("AirdropX:PhysMPC:BadHorizon","Horizon must be NaN or a positive integer.");
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v040_closed_loop"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end

report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2
        error("AirdropX:PhysMPC:MissingQuadprog","quadprog is required for the true MPC QP smoke.");
    end
    if exist("airdropx_jsbsim_oracle_mex","file")~=3
        error("AirdropX:PhysMPC:MissingOracleMex","Certified v0.3.3 Oracle MEX is missing.");
    end
    oracleVersion=string(airdropx_jsbsim_oracle_mex("version"));
    if ~contains(oracleVersion,"v0.3.3")
        error("AirdropX:PhysMPC:WrongOracle","Closed-loop v0.4.0 requires the validated v0.3.3 Oracle, got: %s",oracleVersion);
    end
    [vertex,rowIndex,bankMeta]=airdropx_phys_mpc_get_vertex(opts.BankPath,opts.H,opts.V,opts.CfgId,opts.FuelScale);
    ctrl=airdropx_phys_mpc_condense(vertex,Horizon=opts.Horizon);
    qpSelftest=airdropx_phys_mpc_qp_selftest(ctrl);

    info=airdropx_phys_oracle_init(projectRoot);
    cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); %#ok<NASGU>
    if ~contains(string(info.version),"v0.3.3")
        error("AirdropX:PhysMPC:WrongOracle","Initialized Oracle is not v0.3.3.");
    end
    p=vertex.p; Ts=p.Ts;
    Nsim=ceil(opts.Duration_s/Ts);
    x=ctrl.xref+opts.InitialError;
    x(3)=ctrl.xref(3)+atan2(sin(opts.InitialError(3)),cos(opts.InitialError(3)));
    x(4)=ctrl.xref(4)+atan2(sin(opts.InitialError(4)),cos(opts.InitialError(4)));
    trajectoryOracleSelftest=airdropx_phys_oracle_selftest(x,ctrl.uref,p,ThrowOnFail=true);
    warm=zeros(ctrl.m*ctrl.N,1);

    t=(0:Nsim-1)'*Ts;
    X=zeros(Nsim,7); U=zeros(Nsim,2); Exit=zeros(Nsim,1); QPTime=zeros(Nsim,1);
    PredErr=zeros(Nsim,7); PredErrInf=zeros(Nsim,1); Vlyap=zeros(Nsim,1);
    AlgIter=zeros(Nsim,1); AlgErr=zeros(Nsim,1);
    for k=1:Nsim
        dx=airdropx_phys_mpc_state_error(x,ctrl.xref);
        X(k,:)=x.';
        Vlyap(k)=dx.'*ctrl.P*dx;
        sol=airdropx_phys_mpc_solve(ctrl,x,warm);
        Exit(k)=sol.exitflag; QPTime(k)=sol.solve_time_s;
        if ~sol.feasible
            error("AirdropX:PhysMPC:QPInfeasible","QP failed at k=%d t=%.3f s exitflag=%d.",k,t(k),sol.exitflag);
        end
        u=sol.u;
        if any(u<ctrl.umin-1e-9) || any(u>ctrl.umax+1e-9)
            error("AirdropX:PhysMPC:InputViolation","Hard command bound violated at k=%d.",k);
        end
        U(k,:)=u.';
        xPred=ctrl.xref+ctrl.A*dx+ctrl.B*sol.du;
        [xNext,diagNext]=airdropx_phys_step(x,u,p);
        ePred=airdropx_phys_mpc_state_error(xNext,xPred);
        PredErr(k,:)=ePred.';
        PredErrInf(k)=max(abs(ePred)./ctrl.stateScale);
        if isfield(diagNext,"algebraic_settle_iterations"), AlgIter(k)=diagNext.algebraic_settle_iterations; else, AlgIter(k)=NaN; end
        if isfield(diagNext,"algebraic_settle_error"), AlgErr(k)=diagNext.algebraic_settle_error; else, AlgErr(k)=NaN; end
        x=xNext;
        warm=airdropx_phys_mpc_shift_warmstart(sol.U,ctrl.m,ctrl.N);
        if k==1 || mod(k,25)==0 || k==Nsim
            fprintf("[CL] %4d/%4d t=%6.2f hErr=% .4f VaErr=% .4f gamma=% .3fdeg q=% .3fdeg/s u=[% .4f %.4f] qp=%.3fms pred=%.3g\n", ...
                k,Nsim,t(k),dx(1),dx(2),rad2deg(dx(3)),rad2deg(dx(5)),u(1),u(2),1e3*sol.solve_time_s,PredErrInf(k));
        end
    end
    dxFinal=airdropx_phys_mpc_state_error(x,ctrl.xref);
    Vfinal=dxFinal.'*ctrl.P*dxFinal;
    V0=airdropx_phys_mpc_state_error(X(1,:).',ctrl.xref).'*ctrl.P*airdropx_phys_mpc_state_error(X(1,:).',ctrl.xref);
    finalNormInf=max(abs(dxFinal)./ctrl.stateScale);
    lyapRatio=Vfinal/max(V0,eps);

    % Build table explicitly; avoid nested-vector table variables in CSV.
    Err=zeros(Nsim,7);
    for k=1:Nsim, Err(k,:)=airdropx_phys_mpc_state_error(X(k,:).',ctrl.xref).'; end
    T=table(t,X(:,1),X(:,2),X(:,3),X(:,4),X(:,5),X(:,6),X(:,7), ...
        Err(:,1),Err(:,2),Err(:,3),Err(:,4),Err(:,5),Err(:,6),Err(:,7), ...
        U(:,1),U(:,2),Exit,QPTime,Vlyap,PredErrInf,PredErr(:,1),PredErr(:,2),PredErr(:,3),PredErr(:,4),PredErr(:,5),PredErr(:,6),PredErr(:,7),AlgIter,AlgErr, ...
        'VariableNames',{'t_s','h_m','Va_mps','gamma_rad','theta_rad','q_radps','N1','N2', ...
        'h_err_m','Va_err_mps','gamma_err_rad','theta_err_rad','q_err_radps','N1_err','N2_err', ...
        'elevator_cmd','throttle_cmd','qp_exitflag','qp_time_s','lyapunov_value','pred_err_norm_inf', ...
        'pred_h_m','pred_Va_mps','pred_gamma_rad','pred_theta_rad','pred_q_radps','pred_N1','pred_N2', ...
        'algebraic_iter','algebraic_error'});
    writetable(T,fullfile(opts.OutputRoot,"closed_loop_timeseries.csv"));

    metrics=struct();
    metrics.qp_success_fraction=mean(Exit>0);
    metrics.qp_time_mean_ms=1e3*mean(QPTime);
    metrics.qp_time_p95_ms=1e3*localPercentile(QPTime,95);
    metrics.qp_time_max_ms=1e3*max(QPTime);
    metrics.input_min=min(U,[],1); metrics.input_max=max(U,[],1);
    satTol=1e-6;
    metrics.elevator_saturation_fraction=mean(abs(U(:,1)-ctrl.umin(1))<=satTol | abs(U(:,1)-ctrl.umax(1))<=satTol);
    metrics.throttle_saturation_fraction=mean(abs(U(:,2)-ctrl.umin(2))<=satTol | abs(U(:,2)-ctrl.umax(2))<=satTol);
    metrics.pred_error_norm_p95=localPercentile(PredErrInf,95);
    metrics.pred_error_norm_max=max(PredErrInf);
    metrics.final_state_error=dxFinal;
    metrics.final_normalized_inf=finalNormInf;
    metrics.lyapunov_initial=V0; metrics.lyapunov_final=Vfinal; metrics.lyapunov_ratio=lyapRatio;
    metrics.rms_error=sqrt(mean(Err.^2,1)).';
    tailStart=max(1,Nsim-round(5/Ts)+1);
    metrics.tail5s_rms_error=sqrt(mean(Err(tailStart:end,:).^2,1)).';
    metrics.max_abs_error=max(abs(Err),[],1).';
    metrics.final_error=dxFinal;
    metrics.algebraic_iter_max=max(AlgIter,[],'omitnan');
    metrics.algebraic_error_max=max(AlgErr,[],'omitnan');

    gate=struct();
    gate.qp_all_feasible=all(Exit>0);
    gate.hard_input_bounds=all(U(:,1)>=ctrl.umin(1)-1e-9 & U(:,1)<=ctrl.umax(1)+1e-9 & ...
                               U(:,2)>=ctrl.umin(2)-1e-9 & U(:,2)<=ctrl.umax(2)+1e-9);
    gate.finite=all(isfinite(X),'all') && all(isfinite(U),'all') && all(isfinite(PredErrInf));
    gate.final_normalized=finalNormInf<=opts.MaxFinalNormalizedInf;
    gate.lyapunov_recovery=lyapRatio<=opts.MaxFinalLyapunovRatio;
    gate.realtime_p95=metrics.qp_time_p95_ms<=1e3*Ts*opts.MaxQpP95FractionOfTs;
    gate.pass=gate.qp_all_feasible && gate.hard_input_bounds && gate.finite && gate.final_normalized && gate.lyapunov_recovery && gate.realtime_p95;

    plots=strings(0,1); plot_warning="";
    try
        plots=airdropx_phys_mpc_plot_closed_loop(T,opts.OutputRoot);
    catch plotME
        plot_warning=string(plotME.identifier)+": "+string(plotME.message);
        warning("AirdropX:PhysMPC:PlotFailed","Closed-loop control completed, but plotting failed: %s",plot_warning);
    end
    report=struct("pass",gate.pass,"completed_at",datetime("now"),"oracle_version",oracleVersion, ...
        "bank_path",opts.BankPath,"bank_row",rowIndex,"bank_meta",bankMeta,"vertex",vertex, ...
        "controller",rmfield(ctrl,{'Phi','Gamma','Qbar','Rbar','H','Fx','options'}), ...
        "qp_selftest",qpSelftest,"trajectory_oracle_selftest",trajectoryOracleSelftest,"options",opts,"metrics",metrics,"gate",gate,"plots",plots,"plot_warning",plot_warning);
    save(fullfile(opts.OutputRoot,"closed_loop_smoke.mat"),"report","T","PredErr","-v7.3");
    localWriteSummary(report,fullfile(opts.OutputRoot,"closed_loop_summary.txt"));

    fprintf("=== Physics-MPC v0.4.0 TRUE NONLINEAR CLOSED-LOOP SMOKE ===\n");
    fprintf("vertex: H=%.1f V=%.1f cfg=%d fuel=%.2f, N=%d Ts=%.3f s duration=%.1f s\n",opts.H,opts.V,opts.CfgId,opts.FuelScale,ctrl.N,Ts,opts.Duration_s);
    fprintf("QP self-test: max|du_qp-du_lqr|=%.3g PASS\n",qpSelftest.max_abs_error);
    fprintf("Perturbed Oracle: pathErr=%.3g semigroupMaxScaled=%.3g PASS\n",trajectoryOracleSelftest.max_abs_path_error,trajectoryOracleSelftest.semigroup.max_scaled);
    fprintf("QP: success=%.3f mean=%.3f ms p95=%.3f ms max=%.3f ms\n",metrics.qp_success_fraction,metrics.qp_time_mean_ms,metrics.qp_time_p95_ms,metrics.qp_time_max_ms);
    fprintf("Tracking: finalNormInf=%.6g  LyapunovRatio=%.6g\n",metrics.final_normalized_inf,metrics.lyapunov_ratio);
    fprintf("Prediction: p95Norm=%.6g maxNorm=%.6g\n",metrics.pred_error_norm_p95,metrics.pred_error_norm_max);
    fprintf("Inputs: elevator=[%.6g %.6g] throttle=[%.6g %.6g]\n",metrics.input_min(1),metrics.input_max(1),metrics.input_min(2),metrics.input_max(2));
    fprintf("Gates: qp=%d bounds=%d finite=%d final=%d lyap=%d realtime_p95=%d\n",gate.qp_all_feasible,gate.hard_input_bounds,gate.finite,gate.final_normalized,gate.lyapunov_recovery,gate.realtime_p95);
    if report.pass
        fprintf("=== Physics-MPC v0.4.0 CLOSED-LOOP SMOKE PASS ===\n");
    else
        error("AirdropX:PhysMPC:ClosedLoopSmokeFailed", ...
            "Closed-loop smoke failed: finalNormInf=%.6g (limit %.6g), LyapunovRatio=%.6g (limit %.6g), QP p95=%.3f ms (budget %.3f ms).", ...
            finalNormInf,opts.MaxFinalNormalizedInf,lyapRatio,opts.MaxFinalLyapunovRatio,metrics.qp_time_p95_ms,1e3*Ts*opts.MaxQpP95FractionOfTs);
    end
catch ME
    report.pass=false; report.completed_at=datetime("now");
    report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack);
    evidence=struct();
    if exist("X","var"), evidence.X=X; end
    if exist("U","var"), evidence.U=U; end
    if exist("Exit","var"), evidence.qp_exitflag=Exit; end
    if exist("QPTime","var"), evidence.qp_time_s=QPTime; end
    if exist("PredErr","var"), evidence.prediction_error=PredErr; end
    if exist("PredErrInf","var"), evidence.prediction_error_norm_inf=PredErrInf; end
    if exist("t","var"), evidence.t_s=t; end
    if exist("k","var"), evidence.last_k=k; end
    if exist("x","var"), evidence.last_state=x; end
    if exist("ctrl","var"), evidence.reference_state=ctrl.xref; evidence.reference_input=ctrl.uref; end
    save(fullfile(opts.OutputRoot,"closed_loop_smoke_failure.mat"),"report","evidence","-v7.3");
    fprintf(2,"CLOSED-LOOP SMOKE FAIL [%s] %s\n",ME.identifier,ME.message);
    rethrow(ME);
end
end

function y=localPercentile(x,p)
x=sort(double(x(:)));
x=x(isfinite(x));
if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
pos=1+(numel(x)-1)*(double(p)/100);
lo=floor(pos); hi=ceil(pos);
if lo==hi, y=x(lo); else, y=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function localWriteSummary(report,path)
fid=fopen(path,"w");
if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
m=report.metrics; g=report.gate;
fprintf(fid,"Physics-MPC v0.4.0 true nonlinear closed-loop smoke\n");
fprintf(fid,"pass=%d\n",report.pass);
fprintf(fid,"oracle=%s\n",report.oracle_version);
fprintf(fid,"qp_success_fraction=%.9g\n",m.qp_success_fraction);
fprintf(fid,"qp_time_mean_ms=%.9g\n",m.qp_time_mean_ms);
fprintf(fid,"qp_time_p95_ms=%.9g\n",m.qp_time_p95_ms);
fprintf(fid,"qp_time_max_ms=%.9g\n",m.qp_time_max_ms);
fprintf(fid,"final_normalized_inf=%.9g\n",m.final_normalized_inf);
fprintf(fid,"lyapunov_ratio=%.9g\n",m.lyapunov_ratio);
fprintf(fid,"prediction_error_norm_p95=%.9g\n",m.pred_error_norm_p95);
fprintf(fid,"prediction_error_norm_max=%.9g\n",m.pred_error_norm_max);
fprintf(fid,"gate_qp_all_feasible=%d\n",g.qp_all_feasible);
fprintf(fid,"gate_hard_input_bounds=%d\n",g.hard_input_bounds);
fprintf(fid,"gate_finite=%d\n",g.finite);
fprintf(fid,"gate_final_normalized=%d\n",g.final_normalized);
fprintf(fid,"gate_lyapunov_recovery=%d\n",g.lyapunov_recovery);
fprintf(fid,"gate_realtime_p95=%d\n",g.realtime_p95);
end
