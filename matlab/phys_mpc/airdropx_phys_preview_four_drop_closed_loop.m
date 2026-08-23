function report=airdropx_phys_preview_four_drop_closed_loop(projectRoot,opts)
%AIRDROPX_PHYS_PREVIEW_FOUR_DROP_CLOSED_LOOP Known-schedule time-varying MPC mission.
arguments
    projectRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
    opts.H (1,1) double {mustBeFinite} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite} = 1.0
    opts.Duration_s (1,1) double {mustBePositive} = 40
    opts.DropTimes_s (1,4) double = [10 10.2 10.4 10.6]
    opts.EnableQSoft (1,1) logical = false
    opts.QSoftPenaltyMultiplier (1,1) double {mustBePositive} = 1e4
    opts.ScenarioName (1,1) string = "preview_only"
    opts.ThrowOnFail (1,1) logical = true
    opts.CloseOracleOnReturn (1,1) logical = true
    opts.AllowUncertifiedVertices (1,1) logical = false
    opts.Horizon (1,1) double {mustBeInteger,mustBePositive} = 100
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05
    opts.MaxPeakPrimaryNormalized (1,1) double {mustBePositive} = 1.0
    opts.MaxQpP95FractionOfTs (1,1) double {mustBePositive} = 1.0
    opts.MaxMassMatchError_kg (1,1) double {mustBePositive} = 1e-3
    opts.MaxCargoMassError_kg (1,1) double {mustBePositive} = 1e-3
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v060_preview",opts.ScenarioName); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
if any(~isfinite(opts.DropTimes_s)) || any(diff(opts.DropTimes_s)<0) || any(abs(opts.DropTimes_s-[10 10.2 10.4 10.6])>1e-12)
    error("AirdropX:PhysMPC:PreviewMissionSchedule","v0.6.0 certification mission deliberately fixes the original close schedule [10 10.2 10.4 10.6] s.");
end
if opts.Duration_s<=opts.DropTimes_s(end)+5, error("AirdropX:PhysMPC:MissionTooShort","Need >=5 s after the last drop."); end

report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2, error("AirdropX:PhysMPC:MissingQuadprog","quadprog is required."); end
    if exist("airdropx_jsbsim_oracle_mex","file")~=3, error("AirdropX:PhysMPC:MissingOracleMex","Validated Oracle MEX is missing."); end
    oracleVersion=string(airdropx_jsbsim_oracle_mex("version"));
    if ~contains(oracleVersion,"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","v0.6.0 requires validated Oracle v0.3.3."); end

    schedule=airdropx_phys_mpc_build_cfg_schedule(opts.BankPath,opts.H,opts.V,opts.FuelScale,Horizon=opts.Horizon,AllowUncertifiedVertices=opts.AllowUncertifiedVertices);
    models=schedule.models; Ts=models{1}.vertex.p.Ts; N=schedule.N;
    previewSelftest=airdropx_phys_mpc_preview_selftest(models,QSoftPenaltyMultiplier=opts.QSoftPenaltyMultiplier);
    Nsim=ceil(opts.Duration_s/Ts); t=(0:Nsim-1)'*Ts;
    fprintf("[V060] precomputing known-schedule QPs: mode=%s qSoft=%d horizon=%d ...\n",opts.ScenarioName,opts.EnableQSoft,N);
    cache=airdropx_phys_mpc_preview_precompute(models,t,opts.DropTimes_s,EnableQSoft=opts.EnableQSoft,QSoftPenaltyMultiplier=opts.QSoftPenaltyMultiplier);
    fprintf("[V060] preview cache: unique=%d totalBuild=%.3fs maxOne=%.3fs\n",cache.unique_count,cache.build_time_s,cache.build_time_max_s);

    info=airdropx_phys_oracle_init(projectRoot);
    cleanup=[]; %#ok<NASGU>
    if opts.CloseOracleOnReturn, cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); end %#ok<NASGU>
    if ~contains(string(info.version),"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","Initialized Oracle is not v0.3.3."); end
    if numel(info.pointmass_lbs)~=4 || any(double(info.pointmass_lbs(:))<=0), error("AirdropX:PhysMPC:BadCargoBaseline","Expected four cargo point masses."); end

    expectedMass=zeros(5,1); expectedCg=zeros(5,1); expectedIyy=zeros(5,1);
    for c=0:4
        d=models{c+1}.vertex.trim.diag;
        expectedMass(c+1)=double(d.mass_kg); expectedCg(c+1)=double(d.cg_x_m); expectedIyy(c+1)=double(d.Iyy_kgm2);
    end
    expectedCargoKg=double(info.pointmass_lbs(:))*0.45359237;
    x=models{1}.ctrl.xref;
    trajectoryOracleSelftest=airdropx_phys_oracle_selftest(x,models{1}.ctrl.uref,models{1}.vertex.p,ThrowOnFail=true);
    warmAbs=[]; warmSlack=[]; prevCfg=0;

    X=zeros(Nsim,7); RefX=zeros(Nsim,7); Err=zeros(Nsim,7); U=zeros(Nsim,2); RefU=zeros(Nsim,2);
    Cfg=zeros(Nsim,1); DropEvent=false(Nsim,1); DropCount=zeros(Nsim,1); FromCfgEvent=nan(Nsim,1); ToCfgEvent=nan(Nsim,1);
    Exit=zeros(Nsim,1); QPTime=zeros(Nsim,1); PredErr=zeros(Nsim,7); PredErrInf=zeros(Nsim,1);
    Mass=nan(Nsim,1); Cg=nan(Nsim,1); Iyy=nan(Nsim,1); AlgIter=nan(Nsim,1); AlgErr=nan(Nsim,1);
    SlackMax=zeros(Nsim,1); PreviewTransitionCount=zeros(Nsim,1); FirstControlDeltaFromReactive=zeros(Nsim,2);

    for k=1:Nsim
        cfgNow=min(4,sum(t(k)+1e-10>=opts.DropTimes_s));
        if cfgNow~=prevCfg
            DropEvent(k)=true; DropCount(k)=cfgNow-prevCfg; FromCfgEvent(k)=prevCfg; ToCfgEvent(k)=cfgNow;
            fprintf("[DROP] t=%.2f cfg%d->cfg%d releaseCount=%d\n",t(k),prevCfg,cfgNow,cfgNow-prevCfg);
            prevCfg=cfgNow;
        end
        current=models{cfgNow+1}.ctrl; p=models{cfgNow+1}.vertex.p;
        key=char(cache.step_keys(k)); pc=cache.map(key);
        if pc.firstCfg~=cfgNow, error("AirdropX:PhysMPC:PreviewScheduleMismatch","Preview cfg does not match actual cfg at k=%d.",k); end
        dx=airdropx_phys_mpc_state_error(x,current.xref);
        X(k,:)=x.'; RefX(k,:)=current.xref.'; Err(k,:)=dx.'; RefU(k,:)=current.uref.'; Cfg(k)=cfgNow;
        PreviewTransitionCount(k)=sum(diff(pc.cfgSeq)~=0);

        sol=airdropx_phys_mpc_preview_solve(pc,x,warmAbs,warmSlack);
        Exit(k)=sol.exitflag; QPTime(k)=sol.solve_time_s; SlackMax(k)=sol.slack_max;
        if ~sol.feasible, error("AirdropX:PhysMPC:QPInfeasible","Preview QP failed at t=%.3f cfg=%d exit=%d.",t(k),cfgNow,sol.exitflag); end
        u=sol.u; U(k,:)=u.';
        if any(u<current.umin-1e-9) || any(u>current.umax+1e-9), error("AirdropX:PhysMPC:InputViolation","Input bound violated."); end
        % Diagnostic only: quantify preview pre-action during the final 1 s before
        % the first release. Do not solve a second QP during the rest of flight.
        if t(k)>=opts.DropTimes_s(1)-1-1e-10 && t(k)<opts.DropTimes_s(1)-1e-10
            reactive=airdropx_phys_mpc_solve(current,x,[]);
            if reactive.feasible, FirstControlDeltaFromReactive(k,:)=(u-reactive.u).'; end
        end

        xPred=sol.predicted_states(:,1);
        [xNext,diagNext]=airdropx_phys_step(x,u,p);
        ePred=airdropx_phys_mpc_state_error(xNext,xPred); PredErr(k,:)=ePred.'; PredErrInf(k)=max(abs(ePred)./current.stateScale);
        Mass(k)=double(diagNext.mass_kg); Cg(k)=double(diagNext.cg_x_m); Iyy(k)=double(diagNext.Iyy_kgm2);
        if isfield(diagNext,"algebraic_settle_iterations"), AlgIter(k)=double(diagNext.algebraic_settle_iterations); end
        if isfield(diagNext,"algebraic_settle_error"), AlgErr(k)=double(diagNext.algebraic_settle_error); end
        if isfield(diagNext,"algebraic_settle_converged") && ~logical(diagNext.algebraic_settle_converged), error("AirdropX:PhysMPC:AlgebraicClosureLost","Oracle algebraic closure failed."); end
        x=xNext;
        [warmAbs,warmSlack]=airdropx_phys_mpc_preview_shift_warmstart(sol);
        if k==1 || DropEvent(k) || mod(k,25)==0 || k==Nsim
            fprintf("[V060] %4d/%4d t=%6.2f cfg=%d futureDrops=%d hErr=% .4f VaErr=% .4f q=% .3fdeg/s u=[% .4f %.4f] qp=%.3fms pred=%.3g slack=%.3g\n", ...
                k,Nsim,t(k),cfgNow,PreviewTransitionCount(k),dx(1),dx(2),rad2deg(dx(5)),u(1),u(2),1e3*sol.solve_time_s,PredErrInf(k),SlackMax(k));
        end
    end

    finalCtrl=models{5}.ctrl; dxFinal=airdropx_phys_mpc_state_error(x,finalCtrl.xref);
    finalNormInf=max(abs(dxFinal)./finalCtrl.stateScale);
    primaryNorm=max(abs(Err(:,1:5))./schedule.stateScale(1:5).',[],2);
    tailStart=max(1,Nsim-round(5/Ts)+1); tail5sNormalizedRms=max(sqrt(mean((Err(tailStart:end,:)./schedule.stateScale.').^2,1)));
    massRef=expectedMass(Cfg+1); cgRef=expectedCg(Cfg+1); iyyRef=expectedIyy(Cfg+1);
    massMatchErr=Mass-massRef; cgMatchErr=Cg-cgRef; iyyMatchErr=Iyy-iyyRef;
    eventRows=find(DropEvent); observedDropKg=zeros(4,1); dropMassErr=zeros(4,1);
    for g=1:numel(eventRows)
        kk=eventRows(g); toCfg=round(ToCfgEvent(kk));
        if kk==1, preMass=expectedMass(toCfg); else, preMass=Mass(kk-1); end
        observedDropKg(toCfg)=preMass-Mass(kk); dropMassErr(toCfg)=observedDropKg(toCfg)-expectedCargoKg(toCfg);
    end
    peakH=max(abs(Err(:,1))); peakVa=max(abs(Err(:,2))); peakGamma=max(abs(rad2deg(Err(:,3)))); peakTheta=max(abs(rad2deg(Err(:,4)))); peakQ=max(abs(rad2deg(Err(:,5))));
    [dtRecovery,tRecovery]=localRecoveryTime(t,primaryNorm,opts.DropTimes_s(end),0.05);
    preMask=t>=opts.DropTimes_s(1)-1 & t<opts.DropTimes_s(1);
    preActuation=max(abs(FirstControlDeltaFromReactive(preMask,:)),[],1);
    metrics=struct("expected_cargo_kg",expectedCargoKg,"observed_drop_kg",observedDropKg,"drop_mass_error_max_kg",max(abs(dropMassErr)), ...
        "mass_match_error_max_kg",max(abs(massMatchErr)),"cg_match_error_max_m",max(abs(cgMatchErr)),"Iyy_match_error_max_kgm2",max(abs(iyyMatchErr)), ...
        "qp_success_fraction",mean(Exit>0),"qp_time_mean_ms",1e3*mean(QPTime),"qp_time_p95_ms",1e3*localPercentile(QPTime,95),"qp_time_max_ms",1e3*max(QPTime), ...
        "prediction_error_norm_p95",localPercentile(PredErrInf,95),"prediction_error_norm_max",max(PredErrInf), ...
        "peak_h_err_m",peakH,"peak_Va_err_mps",peakVa,"peak_gamma_err_deg",peakGamma,"peak_theta_err_deg",peakTheta,"peak_q_err_dps",peakQ, ...
        "peak_primary_normalized",max(primaryNorm),"final_normalized_inf",finalNormInf,"tail5s_normalized_rms",tail5sNormalizedRms, ...
        "recovery_time_after_last_drop_s",dtRecovery,"recovery_absolute_time_s",tRecovery,"q_soft_slack_max_radps",max(SlackMax), ...
        "pre_drop_preview_minus_reactive_elevator_max",preActuation(1),"pre_drop_preview_minus_reactive_throttle_max",preActuation(2), ...
        "preview_unique_qps",cache.unique_count,"preview_precompute_time_s",cache.build_time_s);
    eventMetrics=localEventMetrics(t,Err,U,PredErrInf,QPTime,SlackMax,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,schedule.stateScale,Ts);
    gate=struct();
    gate.common_controller=schedule.audit.pass; gate.four_drops=sum(DropCount)==4 && Cfg(end)==4;
    gate.mass_configuration=max(abs(massMatchErr))<=opts.MaxMassMatchError_kg && max(abs(cgMatchErr))<=1e-9 && max(abs(iyyMatchErr))<=1e-6;
    gate.cargo_mass=max(abs(dropMassErr))<=opts.MaxCargoMassError_kg; gate.qp_all_feasible=all(Exit>0);
    gate.hard_input_bounds=all(U(:,1)>=models{1}.ctrl.umin(1)-1e-9 & U(:,1)<=models{1}.ctrl.umax(1)+1e-9 & U(:,2)>=models{1}.ctrl.umin(2)-1e-9 & U(:,2)<=models{1}.ctrl.umax(2)+1e-9);
    gate.finite=all(isfinite(X),'all') && all(isfinite(U),'all'); gate.peak_primary=max(primaryNorm)<=opts.MaxPeakPrimaryNormalized;
    gate.final_normalized=finalNormInf<=opts.MaxFinalNormalizedInf; gate.tail5s=tail5sNormalizedRms<=opts.MaxTail5sNormalizedRms;
    gate.realtime_p95=localPercentile(QPTime,95)<=opts.MaxQpP95FractionOfTs*Ts;
    gate.pass=all(structfun(@(z)logical(z),gate));

    T=table(t,Cfg,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,X(:,1),X(:,2),X(:,3),X(:,4),X(:,5),X(:,6),X(:,7), ...
        RefX(:,1),RefX(:,2),RefX(:,3),RefX(:,4),RefX(:,5),RefX(:,6),RefX(:,7),Err(:,1),Err(:,2),Err(:,3),Err(:,4),Err(:,5),Err(:,6),Err(:,7), ...
        U(:,1),U(:,2),RefU(:,1),RefU(:,2),Mass,Cg,Iyy,Exit,QPTime,PredErrInf,AlgIter,AlgErr,SlackMax,PreviewTransitionCount,FirstControlDeltaFromReactive(:,1),FirstControlDeltaFromReactive(:,2), ...
        'VariableNames',{'t_s','cfg','drop_event','drop_count','from_cfg_event','to_cfg_event','h_m','Va_mps','gamma_rad','theta_rad','q_radps','N1','N2', ...
        'href_m','Vref_mps','gamma_ref_rad','theta_ref_rad','q_ref_radps','N1_ref','N2_ref','h_err_m','Va_err_mps','gamma_err_rad','theta_err_rad','q_err_radps','N1_err','N2_err', ...
        'elevator_cmd','throttle_cmd','elevator_trim','throttle_trim','mass_kg','cg_x_m','Iyy_kgm2','qp_exitflag','qp_time_s','pred_err_norm_inf','algebraic_iter','algebraic_error', ...
        'q_soft_slack_radps','preview_transition_count','preview_minus_reactive_elevator','preview_minus_reactive_throttle'});
    writetable(T,fullfile(opts.OutputRoot,"preview_timeseries.csv"));
    writetable(eventMetrics,fullfile(opts.OutputRoot,"preview_event_metrics.csv"));
    plotFiles=strings(0,1); plotWarning="";
    try
        plotFiles=airdropx_phys_mpc_plot_preview(T,opts.OutputRoot,opts.DropTimes_s,opts.ScenarioName);
    catch plotME
        plotWarning=string(plotME.identifier)+": "+string(plotME.message);
        warning("AirdropX:PhysMPC:PlotFailed","Preview mission completed but plotting failed: %s",plotWarning);
    end
    cacheMeta=rmfield(cache,'map');
    report=struct("pass",gate.pass,"completed_at",datetime("now"),"oracle_version",oracleVersion,"scenario_name",opts.ScenarioName,"options",opts, ...
        "schedule_audit",schedule.audit,"preview_selftest",previewSelftest,"trajectory_oracle_selftest",trajectoryOracleSelftest,"cache_meta",cacheMeta,"metrics",metrics,"event_metrics",eventMetrics,"gate",gate, ...
        "plots",plotFiles,"plot_warning",plotWarning);
    save(fullfile(opts.OutputRoot,"preview_mission.mat"),"report","T","PredErr","-v7.3");
    localWriteSummary(report,fullfile(opts.OutputRoot,"preview_summary.txt"));
    fprintf("=== Physics-MPC v0.6.0 PREVIEW MISSION %s: pass=%d peakQ=%.4fdeg/s peakNorm=%.4f final=%.6g QPp95=%.3fms ===\n", ...
        opts.ScenarioName,report.pass,metrics.peak_q_err_dps,metrics.peak_primary_normalized,metrics.final_normalized_inf,metrics.qp_time_p95_ms);
    if ~report.pass && opts.ThrowOnFail
        error("AirdropX:PhysMPC:PreviewMissionFailed","Preview mission failed: peakQ=%.4g deg/s peakNorm=%.4g final=%.4g.",metrics.peak_q_err_dps,metrics.peak_primary_normalized,metrics.final_normalized_inf);
    end
catch ME
    report.pass=false; report.completed_at=datetime("now"); report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack);
    evidence=struct(); vars={'t','X','RefX','Err','U','RefU','Cfg','DropEvent','DropCount','Exit','QPTime','PredErr','PredErrInf','Mass','Cg','Iyy','SlackMax','PreviewTransitionCount','x','warmAbs','warmSlack'};
    for ii=1:numel(vars), vn=vars{ii}; if exist(vn,'var'), evidence.(vn)=eval(vn); end, end %#ok<EVLDIR>
    save(fullfile(opts.OutputRoot,"preview_mission_failure.mat"),"report","evidence","-v7.3");
    fprintf(2,"PREVIEW MISSION FAIL [%s] %s\n",ME.identifier,ME.message); rethrow(ME);
end
end

function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
pos=1+(numel(x)-1)*(double(p)/100); lo=floor(pos); hi=ceil(pos); if lo==hi, y=x(lo); else, y=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function [dtRecovery,tRecovery]=localRecoveryTime(t,primary,lastDrop,thr)
idx=find(t>=lastDrop-1e-10); dtRecovery=NaN; tRecovery=NaN;
for j=1:numel(idx), k=idx(j); if primary(k)<=thr && all(primary(k:end)<=thr), tRecovery=t(k); dtRecovery=tRecovery-lastDrop; return; end, end
end
function E=localEventMetrics(t,Err,U,PredErrInf,QPTime,SlackMax,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,stateScale,Ts)
eventRows=find(DropEvent); n=numel(eventRows);
Event=(1:n).'; Time=t(eventRows); ReleaseCount=DropCount(eventRows); FromCfg=FromCfgEvent(eventRows); ToCfg=ToCfgEvent(eventRows);
WindowEnd=zeros(n,1); PeakH=zeros(n,1); PeakVa=zeros(n,1); PeakGammaDeg=zeros(n,1); PeakThetaDeg=zeros(n,1); PeakQDps=zeros(n,1); PeakNorm=zeros(n,1); PredMax=zeros(n,1); QpMaxMs=zeros(n,1); SlackMaxDegps=zeros(n,1);
for g=1:n
    t0=Time(g); if g<n, t1=min(t0+5,Time(g+1)-0.5*Ts); else, t1=t0+5; end
    WindowEnd(g)=t1; mask=t>=t0-1e-10 & t<=t1+1e-10; ee=Err(mask,:);
    PeakH(g)=max(abs(ee(:,1))); PeakVa(g)=max(abs(ee(:,2))); PeakGammaDeg(g)=max(abs(rad2deg(ee(:,3)))); PeakThetaDeg(g)=max(abs(rad2deg(ee(:,4)))); PeakQDps(g)=max(abs(rad2deg(ee(:,5))));
    PeakNorm(g)=max(max(abs(ee(:,1:5))./stateScale(1:5).',[],2)); PredMax(g)=max(PredErrInf(mask)); QpMaxMs(g)=1e3*max(QPTime(mask)); SlackMaxDegps(g)=rad2deg(max(SlackMax(mask)));
end
E=table(Event,Time,WindowEnd,ReleaseCount,FromCfg,ToCfg,PeakH,PeakVa,PeakGammaDeg,PeakThetaDeg,PeakQDps,PeakNorm,PredMax,QpMaxMs,SlackMaxDegps, ...
    'VariableNames',{'event','t_s','window_end_s','release_count','from_cfg','to_cfg','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','prediction_error_norm_max','qp_time_max_ms','q_soft_slack_max_degps'});
end

function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
m=r.metrics; g=r.gate;
fprintf(fid,"Physics-MPC v0.6.0 known-schedule time-varying preview mission\nscenario=%s\npass=%d\nq_soft=%d\n",r.scenario_name,r.pass,r.options.EnableQSoft);
fprintf(fid,"drop_schedule_s=10 10.2 10.4 10.6\ncommon_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\nhorizon=%d\n",r.schedule_audit.max_Q_diff,r.schedule_audit.max_R_diff,r.schedule_audit.horizon);
fprintf(fid,"preview_selftest_pass=%d\npreview_unique_qps=%d\npreview_precompute_time_s=%.9g\n",r.preview_selftest.pass,m.preview_unique_qps,m.preview_precompute_time_s);
fprintf(fid,"allow_uncertified_vertices=%d\nall_schedule_vertices_certified=%d\ncertified_cfg_count=%d/5\n",r.options.AllowUncertifiedVertices,r.schedule_audit.all_vertices_certified,r.schedule_audit.certified_cfg_count);
fprintf(fid,"peak_h_err_m=%.9g\npeak_Va_err_mps=%.9g\npeak_gamma_err_deg=%.9g\npeak_theta_err_deg=%.9g\npeak_q_err_dps=%.9g\npeak_primary_normalized=%.9g\n",m.peak_h_err_m,m.peak_Va_err_mps,m.peak_gamma_err_deg,m.peak_theta_err_deg,m.peak_q_err_dps,m.peak_primary_normalized);
fprintf(fid,"final_normalized_inf=%.9g\ntail5s_normalized_rms=%.9g\nrecovery_time_after_last_drop_s=%.9g\n",m.final_normalized_inf,m.tail5s_normalized_rms,m.recovery_time_after_last_drop_s);
fprintf(fid,"qp_success_fraction=%.9g\nqp_time_p95_ms=%.9g\nprediction_error_norm_p95=%.9g\nprediction_error_norm_max=%.9g\n",m.qp_success_fraction,m.qp_time_p95_ms,m.prediction_error_norm_p95,m.prediction_error_norm_max);
fprintf(fid,"q_soft_slack_max_radps=%.9g\npre_drop_preview_minus_reactive_elevator_max=%.9g\npre_drop_preview_minus_reactive_throttle_max=%.9g\n",m.q_soft_slack_max_radps,m.pre_drop_preview_minus_reactive_elevator_max,m.pre_drop_preview_minus_reactive_throttle_max);
fn=fieldnames(g); for i=1:numel(fn), fprintf(fid,"gate_%s=%d\n",fn{i},g.(fn{i})); end
end
