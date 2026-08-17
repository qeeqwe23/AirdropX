function report=airdropx_phys_four_drop_closed_loop(projectRoot,opts)
%AIRDROPX_PHYS_FOUR_DROP_CLOSED_LOOP Configurable four-payload release mission in nonlinear JSBSim.
%
% Mission semantics:
%   cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4
% Each transition removes the next real JSBSim cargo point mass, exactly the
% same point-mass order used by the AirdropX S-Function drop mechanism. The
% explicit aircraft state is carried continuously across the release; no
% artificial state reset, recovery controller, TECS, PI layer, BO tuning, or
% fabricated release impulse is added.
%
% The plant is the exact v0.3.3 JSBSim oracle. The MPC is one common QP kernel;
% only the certified physics model/trim/terminal data are scheduled by cfg.
arguments
    projectRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
    opts.H (1,1) double {mustBeFinite} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite} = 1.0
    opts.Duration_s (1,1) double {mustBePositive} = 40
    opts.DropTimes_s (1,4) double = [10 10.2 10.4 10.6]
    opts.ScenarioName (1,1) string = "custom"
    opts.RecoveryNormalizedThreshold (1,1) double {mustBePositive} = 0.05
    opts.ThrowOnFail (1,1) logical = true
    opts.CloseOracleOnReturn (1,1) logical = true
    opts.Horizon (1,1) double = NaN
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05
    opts.MaxPeakPrimaryNormalized (1,1) double {mustBePositive} = 1.0
    opts.MaxQpP95FractionOfTs (1,1) double {mustBePositive} = 1.0
    opts.MaxMassMatchError_kg (1,1) double {mustBePositive} = 1e-3
    opts.MaxCargoMassError_kg (1,1) double {mustBePositive} = 1e-3
end
if opts.FuelScale<0 || opts.FuelScale>1.2, error("AirdropX:PhysMPC:BadFuel","FuelScale must be in [0,1.2]."); end
if any(~isfinite(opts.DropTimes_s)) || any(diff(opts.DropTimes_s)<0) || opts.DropTimes_s(1)<0
    error("AirdropX:PhysMPC:BadDropSchedule","DropTimes_s must contain four finite nondecreasing nonnegative times. Equal times mean simultaneous release in one MPC sample.");
end
if opts.Duration_s<=opts.DropTimes_s(end)+5
    error("AirdropX:PhysMPC:MissionTooShort","Duration must leave at least 5 s after the fourth drop.");
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v052_four_drop"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end

report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2, error("AirdropX:PhysMPC:MissingQuadprog","quadprog is required."); end
    if exist("airdropx_jsbsim_oracle_mex","file")~=3, error("AirdropX:PhysMPC:MissingOracleMex","Validated Oracle MEX is missing."); end
    oracleVersion=string(airdropx_jsbsim_oracle_mex("version"));
    if ~contains(oracleVersion,"v0.3.3")
        error("AirdropX:PhysMPC:WrongOracle","v0.5.2 requires validated Physics Oracle v0.3.3, got %s.",oracleVersion);
    end

    schedule=airdropx_phys_mpc_build_cfg_schedule(opts.BankPath,opts.H,opts.V,opts.FuelScale,Horizon=opts.Horizon);
    info=airdropx_phys_oracle_init(projectRoot);
    cleanup=[]; %#ok<NASGU>
    if opts.CloseOracleOnReturn
        cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); %#ok<NASGU>
    end
    if ~contains(string(info.version),"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","Initialized Oracle is not v0.3.3."); end
    if numel(info.pointmass_lbs)~=4 || any(double(info.pointmass_lbs(:))<=0)
        error("AirdropX:PhysMPC:BadCargoBaseline","Expected exactly four positive JSBSim cargo point masses.");
    end

    models=schedule.models;
    ctrl=models{1}.ctrl;
    Ts=models{1}.vertex.p.Ts;
    if any(abs(opts.DropTimes_s/Ts-round(opts.DropTimes_s/Ts))>1e-9)
        error("AirdropX:PhysMPC:DropNotOnSample","All drop times must lie on the %.6g s MPC sample grid.",Ts);
    end
    Nsim=ceil(opts.Duration_s/Ts);
    x=ctrl.xref; % begin exactly at the certified cfg0 trim
    p=ctrlModelP(models{1});
    trajectoryOracleSelftest=airdropx_phys_oracle_selftest(x,ctrl.uref,p,ThrowOnFail=true);
    warm=zeros(ctrl.m*ctrl.N,1);
    prevCfg=0;

    % Certified expected mass properties for each cfg.
    expectedMass=zeros(5,1); expectedCg=zeros(5,1); expectedIyy=zeros(5,1);
    for c=0:4
        d=models{c+1}.vertex.trim.diag;
        expectedMass(c+1)=double(d.mass_kg);
        expectedCg(c+1)=double(d.cg_x_m);
        expectedIyy(c+1)=double(d.Iyy_kgm2);
    end
    expectedCargoKg=double(info.pointmass_lbs(:))*0.45359237;

    t=(0:Nsim-1)'*Ts;
    X=zeros(Nsim,7); RefX=zeros(Nsim,7); U=zeros(Nsim,2); RefU=zeros(Nsim,2);
    Cfg=zeros(Nsim,1); DropEvent=false(Nsim,1); DropCount=zeros(Nsim,1); FromCfgEvent=nan(Nsim,1); ToCfgEvent=nan(Nsim,1); Exit=zeros(Nsim,1); QPTime=zeros(Nsim,1);
    PredErr=zeros(Nsim,7); PredErrInf=zeros(Nsim,1); Err=zeros(Nsim,7); NormInf=zeros(Nsim,1);
    Mass=nan(Nsim,1); Cg=nan(Nsim,1); Iyy=nan(Nsim,1); AlgIter=nan(Nsim,1); AlgErr=nan(Nsim,1);
    Vlocal=nan(Nsim,1);
    lastSolU=warm;

    for k=1:Nsim
        cfgNow=sum(t(k)+1e-10>=opts.DropTimes_s);
        cfgNow=min(max(cfgNow,0),4);
        if cfgNow~=prevCfg
            fromCfg=prevCfg;
            toCfg=cfgNow;
            releaseCount=toCfg-fromCfg;
            if releaseCount<1
                error("AirdropX:PhysMPC:CfgReversal","Payload cfg cannot decrease during a release mission.");
            end
            oldCtrl=ctrl;
            ctrl=models{toCfg+1}.ctrl;
            warm=airdropx_phys_mpc_rebase_warmstart(warm,oldCtrl,ctrl);
            DropEvent(k)=true;
            DropCount(k)=releaseCount;
            FromCfgEvent(k)=fromCfg;
            ToCfgEvent(k)=toCfg;
            expectedGroupKg=sum(expectedCargoKg(fromCfg+1:toCfg));
            fprintf("[DROP] t=%.2f s cfg%d->cfg%d releaseCount=%d expected group cargo=%.6f kg | trim mass %.6f->%.6f kg cg %.6f->%.6f m\n", ...
                t(k),fromCfg,toCfg,releaseCount,expectedGroupKg,expectedMass(fromCfg+1),expectedMass(toCfg+1),expectedCg(fromCfg+1),expectedCg(toCfg+1));
            prevCfg=toCfg;
        else
            ctrl=models{cfgNow+1}.ctrl;
        end
        p=ctrlModelP(models{cfgNow+1});
        dx=airdropx_phys_mpc_state_error(x,ctrl.xref);
        X(k,:)=x.'; RefX(k,:)=ctrl.xref.'; Err(k,:)=dx.';
        RefU(k,:)=ctrl.uref.'; Cfg(k)=cfgNow; NormInf(k)=max(abs(dx)./ctrl.stateScale);
        Vlocal(k)=dx.'*ctrl.P*dx;

        sol=airdropx_phys_mpc_solve(ctrl,x,warm);
        Exit(k)=sol.exitflag; QPTime(k)=sol.solve_time_s;
        if ~sol.feasible
            error("AirdropX:PhysMPC:QPInfeasible","QP failed at k=%d t=%.3f cfg=%d exitflag=%d.",k,t(k),cfgNow,sol.exitflag);
        end
        u=sol.u;
        if any(u<ctrl.umin-1e-9) || any(u>ctrl.umax+1e-9)
            error("AirdropX:PhysMPC:InputViolation","Hard input bound violated at k=%d t=%.3f cfg=%d.",k,t(k),cfgNow);
        end
        U(k,:)=u.';
        xPred=ctrl.xref+ctrl.A*dx+ctrl.B*sol.du;
        [xNext,diagNext]=airdropx_phys_step(x,u,p);
        ePred=airdropx_phys_mpc_state_error(xNext,xPred);
        PredErr(k,:)=ePred.'; PredErrInf(k)=max(abs(ePred)./ctrl.stateScale);
        Mass(k)=double(diagNext.mass_kg); Cg(k)=double(diagNext.cg_x_m); Iyy(k)=double(diagNext.Iyy_kgm2);
        if isfield(diagNext,"algebraic_settle_iterations"), AlgIter(k)=double(diagNext.algebraic_settle_iterations); end
        if isfield(diagNext,"algebraic_settle_error"), AlgErr(k)=double(diagNext.algebraic_settle_error); end
        if isfield(diagNext,"algebraic_settle_converged") && ~logical(diagNext.algebraic_settle_converged)
            error("AirdropX:PhysMPC:AlgebraicClosureLost","Oracle algebraic closure failed at k=%d t=%.3f cfg=%d.",k,t(k),cfgNow);
        end
        x=xNext;
        lastSolU=sol.U;
        warm=airdropx_phys_mpc_shift_warmstart(sol.U,ctrl.m,ctrl.N);
        if k==1 || DropEvent(k) || mod(k,25)==0 || k==Nsim
            fprintf("[M051] %4d/%4d t=%6.2f cfg=%d hErr=% .4f VaErr=% .4f gamma=% .3fdeg q=% .3fdeg/s mass=%.2f u=[% .4f %.4f] qp=%.3fms pred=%.3g\n", ...
                k,Nsim,t(k),cfgNow,dx(1),dx(2),rad2deg(dx(3)),rad2deg(dx(5)),Mass(k),u(1),u(2),1e3*sol.solve_time_s,PredErrInf(k));
        end
    end

    finalCtrl=models{5}.ctrl;
    dxFinal=airdropx_phys_mpc_state_error(x,finalCtrl.xref);
    finalNormInf=max(abs(dxFinal)./finalCtrl.stateScale);
    primaryNorm=max(abs(Err(:,1:5))./schedule.stateScale(1:5).',[],2);
    tailStart=max(1,Nsim-round(5/Ts)+1);
    tailNormRms=sqrt(mean((Err(tailStart:end,:)./schedule.stateScale.').^2,1));
    tail5sNormalizedRms=max(tailNormRms);

    massRef=expectedMass(Cfg+1);
    cgRef=expectedCg(Cfg+1);
    iyyRef=expectedIyy(Cfg+1);
    massMatchErr=Mass-massRef;
    cgMatchErr=Cg-cgRef;
    iyyMatchErr=Iyy-iyyRef;
    eventRows=find(DropEvent);
    nGroups=numel(eventRows);
    if nGroups<1
        error("AirdropX:PhysMPC:MissingDropEvent","No payload release event was recorded.");
    end
    groupFromCfg=zeros(nGroups,1); groupToCfg=zeros(nGroups,1); groupReleaseCount=zeros(nGroups,1);
    expectedGroupDropKg=zeros(nGroups,1); observedGroupDropKg=zeros(nGroups,1); groupDropMassError=zeros(nGroups,1);
    for g=1:nGroups
        kk=eventRows(g);
        fromCfg=round(FromCfgEvent(kk)); toCfg=round(ToCfgEvent(kk));
        groupFromCfg(g)=fromCfg; groupToCfg(g)=toCfg; groupReleaseCount(g)=toCfg-fromCfg;
        expectedGroupDropKg(g)=sum(expectedCargoKg(fromCfg+1:toCfg));
        if kk==1, preMass=expectedMass(fromCfg+1); else, preMass=Mass(kk-1); end
        observedGroupDropKg(g)=preMass-Mass(kk);
        groupDropMassError(g)=observedGroupDropKg(g)-expectedGroupDropKg(g);
    end
    observedDropKg=nan(4,1);
    for g=1:nGroups
        if groupReleaseCount(g)==1
            observedDropKg(groupToCfg(g))=observedGroupDropKg(g);
        end
    end

    metrics=struct();
    metrics.qp_success_fraction=mean(Exit>0);
    metrics.qp_time_mean_ms=1e3*mean(QPTime);
    metrics.qp_time_p95_ms=1e3*localPercentile(QPTime,95);
    metrics.qp_time_max_ms=1e3*max(QPTime);
    metrics.pred_error_norm_p95=localPercentile(PredErrInf,95);
    metrics.pred_error_norm_max=max(PredErrInf);
    metrics.final_error=dxFinal; metrics.final_normalized_inf=finalNormInf;
    metrics.peak_primary_normalized=max(primaryNorm);
    metrics.tail5s_normalized_rms=tail5sNormalizedRms;
    metrics.rms_error=sqrt(mean(Err.^2,1)).';
    metrics.tail5s_rms_error=sqrt(mean(Err(tailStart:end,:).^2,1)).';
    metrics.max_abs_error=max(abs(Err),[],1).';
    metrics.peak_h_err_m=metrics.max_abs_error(1);
    metrics.peak_Va_err_mps=metrics.max_abs_error(2);
    metrics.peak_gamma_err_deg=rad2deg(metrics.max_abs_error(3));
    metrics.peak_theta_err_deg=rad2deg(metrics.max_abs_error(4));
    metrics.peak_q_err_dps=rad2deg(metrics.max_abs_error(5));
    [metrics.recovery_time_after_last_drop_s,metrics.recovery_absolute_time_s]=localRecoveryTime(t,primaryNorm,max(opts.DropTimes_s),opts.RecoveryNormalizedThreshold,Ts);
    metrics.input_min=min(U,[],1); metrics.input_max=max(U,[],1);
    satTol=1e-6;
    metrics.elevator_saturation_fraction=mean(abs(U(:,1)+1)<=satTol | abs(U(:,1)-1)<=satTol);
    metrics.throttle_saturation_fraction=mean(abs(U(:,2))<=satTol | abs(U(:,2)-1)<=satTol);
    metrics.mass_match_error_max_kg=max(abs(massMatchErr));
    metrics.cg_match_error_max_m=max(abs(cgMatchErr));
    metrics.Iyy_match_error_max_kgm2=max(abs(iyyMatchErr));
    metrics.expected_cargo_kg=expectedCargoKg;
    metrics.observed_drop_kg=observedDropKg;
    metrics.group_from_cfg=groupFromCfg;
    metrics.group_to_cfg=groupToCfg;
    metrics.group_release_count=groupReleaseCount;
    metrics.expected_group_drop_kg=expectedGroupDropKg;
    metrics.observed_group_drop_kg=observedGroupDropKg;
    metrics.group_drop_mass_error_kg=groupDropMassError;
    metrics.drop_mass_error_max_kg=max(abs(groupDropMassError));
    metrics.total_expected_drop_kg=sum(expectedCargoKg);
    metrics.total_observed_drop_kg=sum(observedGroupDropKg);
    metrics.algebraic_iter_max=max(AlgIter,[],'omitnan');
    metrics.algebraic_error_max=max(AlgErr,[],'omitnan');

    eventMetrics=localEventMetrics(t,Err,U,PredErrInf,QPTime,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,schedule.stateScale,Ts);

    gate=struct();
    gate.common_controller=schedule.audit.pass && schedule.audit.qp_selftests_pass;
    gate.four_drops=sum(DropCount)==4 && Cfg(end)==4 && all(ToCfgEvent(DropEvent)>FromCfgEvent(DropEvent));
    gate.mass_configuration=metrics.mass_match_error_max_kg<=opts.MaxMassMatchError_kg;
    gate.cargo_mass=metrics.drop_mass_error_max_kg<=opts.MaxCargoMassError_kg;
    gate.qp_all_feasible=all(Exit>0);
    gate.hard_input_bounds=all(U(:,1)>=-1-1e-9 & U(:,1)<=1+1e-9 & U(:,2)>=-1e-9 & U(:,2)<=1+1e-9);
    gate.finite=all(isfinite(X),'all') && all(isfinite(U),'all') && all(isfinite(Mass)) && all(isfinite(PredErrInf));
    gate.peak_primary=metrics.peak_primary_normalized<=opts.MaxPeakPrimaryNormalized;
    gate.final_normalized=finalNormInf<=opts.MaxFinalNormalizedInf;
    gate.tail5s=tail5sNormalizedRms<=opts.MaxTail5sNormalizedRms;
    gate.realtime_p95=metrics.qp_time_p95_ms<=1e3*Ts*opts.MaxQpP95FractionOfTs;
    gate.pass=gate.common_controller && gate.four_drops && gate.mass_configuration && gate.cargo_mass && ...
        gate.qp_all_feasible && gate.hard_input_bounds && gate.finite && gate.peak_primary && ...
        gate.final_normalized && gate.tail5s && gate.realtime_p95;

    T=table(t,Cfg,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,X(:,1),X(:,2),X(:,3),X(:,4),X(:,5),X(:,6),X(:,7), ...
        RefX(:,1),RefX(:,2),RefX(:,3),RefX(:,4),RefX(:,5),RefX(:,6),RefX(:,7), ...
        Err(:,1),Err(:,2),Err(:,3),Err(:,4),Err(:,5),Err(:,6),Err(:,7), ...
        U(:,1),U(:,2),RefU(:,1),RefU(:,2),Mass,Cg,Iyy,Exit,QPTime,PredErrInf,AlgIter,AlgErr,Vlocal, ...
        'VariableNames',{'t_s','cfg','drop_event','drop_count','from_cfg_event','to_cfg_event','h_m','Va_mps','gamma_rad','theta_rad','q_radps','N1','N2', ...
        'href_m','Vref_mps','gamma_ref_rad','theta_ref_rad','q_ref_radps','N1_ref','N2_ref', ...
        'h_err_m','Va_err_mps','gamma_err_rad','theta_err_rad','q_err_radps','N1_err','N2_err', ...
        'elevator_cmd','throttle_cmd','elevator_trim','throttle_trim','mass_kg','cg_x_m','Iyy_kgm2', ...
        'qp_exitflag','qp_time_s','pred_err_norm_inf','algebraic_iter','algebraic_error','local_P_energy'});
    writetable(T,fullfile(opts.OutputRoot,"four_drop_timeseries.csv"));
    writetable(eventMetrics,fullfile(opts.OutputRoot,"four_drop_event_metrics.csv"));

    plots=strings(0,1); plotWarning="";
    try
        plots=airdropx_phys_mpc_plot_four_drop(T,eventMetrics,opts.OutputRoot,opts.DropTimes_s);
    catch plotME
        plotWarning=string(plotME.identifier)+": "+string(plotME.message);
        warning("AirdropX:PhysMPC:PlotFailed","Mission completed but plotting failed: %s",plotWarning);
    end

    report=struct("pass",gate.pass,"completed_at",datetime("now"),"oracle_version",oracleVersion, ...
        "scenario_name",opts.ScenarioName,"options",opts,"schedule_audit",schedule.audit,"trajectory_oracle_selftest",trajectoryOracleSelftest, ...
        "expected_mass_kg",expectedMass,"expected_cg_x_m",expectedCg,"expected_Iyy_kgm2",expectedIyy, ...
        "metrics",metrics,"event_metrics",eventMetrics,"gate",gate,"plots",plots,"plot_warning",plotWarning);
    save(fullfile(opts.OutputRoot,"four_drop_mission.mat"),"report","T","PredErr","-v7.3");
    localWriteSummary(report,fullfile(opts.OutputRoot,"four_drop_summary.txt"));

    fprintf("=== Physics-MPC v0.5.2 DROP-SCHEDULE NONLINEAR MISSION: %s ===\n",opts.ScenarioName);
    fprintf("common kernel: Q/R unified=%d horizon=%d cfg QP selftests=%d\n",schedule.audit.pass,schedule.N,schedule.audit.qp_selftests_pass);
    fprintf("drops: schedule=[%s] groups=%d expectedTotal=%.6fkg observedTotal=%.6fkg maxGroupErr=%.6gkg\n",localVec(opts.DropTimes_s),nGroups,metrics.total_expected_drop_kg,metrics.total_observed_drop_kg,metrics.drop_mass_error_max_kg);
    fprintf("mass config maxErr=%.6g kg  cg maxErr=%.6g m  Iyy maxErr=%.6g kgm2\n",metrics.mass_match_error_max_kg,metrics.cg_match_error_max_m,metrics.Iyy_match_error_max_kgm2);
    fprintf("tracking: peakPrimaryNorm=%.6g finalNorm=%.6g tail5sNormRMS=%.6g\n",metrics.peak_primary_normalized,metrics.final_normalized_inf,metrics.tail5s_normalized_rms);
    fprintf("QP: success=%.3f mean=%.3fms p95=%.3fms max=%.3fms prediction p95=%.6g max=%.6g\n", ...
        metrics.qp_success_fraction,metrics.qp_time_mean_ms,metrics.qp_time_p95_ms,metrics.qp_time_max_ms,metrics.pred_error_norm_p95,metrics.pred_error_norm_max);
    fprintf("gates: common=%d drops=%d mass=%d cargo=%d qp=%d bounds=%d finite=%d peak=%d final=%d tail=%d realtime=%d\n", ...
        gate.common_controller,gate.four_drops,gate.mass_configuration,gate.cargo_mass,gate.qp_all_feasible,gate.hard_input_bounds,gate.finite,gate.peak_primary,gate.final_normalized,gate.tail5s,gate.realtime_p95);
    if report.pass
        fprintf("=== Physics-MPC v0.5.2 DROP-SCHEDULE MISSION PASS: %s ===\n",opts.ScenarioName);
    elseif opts.ThrowOnFail
        error("AirdropX:PhysMPC:FourDropMissionFailed", ...
            "Four-drop mission failed: peakNorm=%.4g finalNorm=%.4g tailNormRMS=%.4g massErr=%.4gkg cargoErr=%.4gkg QPp95=%.3fms.", ...
            metrics.peak_primary_normalized,metrics.final_normalized_inf,metrics.tail5s_normalized_rms, ...
            metrics.mass_match_error_max_kg,metrics.drop_mass_error_max_kg,metrics.qp_time_p95_ms);
    else
        fprintf("=== Physics-MPC v0.5.2 DROP-SCHEDULE MISSION FAIL (returned without throwing): %s ===\n",opts.ScenarioName);
    end
catch ME
    report.pass=false; report.completed_at=datetime("now");
    report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack);
    evidence=struct();
    vars={'t','X','RefX','Err','U','RefU','Cfg','DropEvent','DropCount','FromCfgEvent','ToCfgEvent','Exit','QPTime','PredErr','PredErrInf','Mass','Cg','Iyy','AlgIter','AlgErr','Vlocal','x','warm','lastSolU'};
    for ii=1:numel(vars)
        vn=vars{ii};
        if exist(vn,'var'), evidence.(vn)=eval(vn); end %#ok<EVLDIR>
    end
    save(fullfile(opts.OutputRoot,"four_drop_mission_failure.mat"),"report","evidence","-v7.3");
    fprintf(2,"FOUR-DROP MISSION FAIL [%s] %s\n",ME.identifier,ME.message);
    rethrow(ME);
end
end

function p=ctrlModelP(model)
p=model.vertex.p;
end

function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x));
if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
pos=1+(numel(x)-1)*(double(p)/100); lo=floor(pos); hi=ceil(pos);
if lo==hi, y=x(lo); else, y=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function E=localEventMetrics(t,Err,U,PredErrInf,QPTime,DropEvent,DropCount,FromCfgEvent,ToCfgEvent,stateScale,Ts)
eventRows=find(DropEvent); n=numel(eventRows);
Group=(1:n).'; Time=t(eventRows); ReleaseCount=DropCount(eventRows); FromCfg=FromCfgEvent(eventRows); ToCfg=ToCfgEvent(eventRows);
WindowEnd=zeros(n,1); PeakH=zeros(n,1); PeakVa=zeros(n,1); PeakGammaDeg=zeros(n,1); PeakThetaDeg=zeros(n,1); PeakQDps=zeros(n,1);
PeakNorm=zeros(n,1); PeakElev=zeros(n,1); PeakThr=zeros(n,1); PredMax=zeros(n,1); QpMaxMs=zeros(n,1);
for g=1:n
    t0=Time(g);
    if g<n
        t1=min(t0+5,Time(g+1)-0.5*Ts);
    else
        t1=t0+5;
    end
    WindowEnd(g)=t1;
    mask=t>=t0-1e-10 & t<=t1+1e-10;
    ee=Err(mask,:); uu=U(mask,:);
    PeakH(g)=max(abs(ee(:,1))); PeakVa(g)=max(abs(ee(:,2)));
    PeakGammaDeg(g)=max(abs(rad2deg(ee(:,3)))); PeakThetaDeg(g)=max(abs(rad2deg(ee(:,4)))); PeakQDps(g)=max(abs(rad2deg(ee(:,5))));
    PeakNorm(g)=max(max(abs(ee(:,1:5))./stateScale(1:5).',[],2));
    PeakElev(g)=max(abs(uu(:,1))); PeakThr(g)=max(uu(:,2));
    PredMax(g)=max(PredErrInf(mask)); QpMaxMs(g)=1e3*max(QPTime(mask));
end
E=table(Group,Time,WindowEnd,ReleaseCount,FromCfg,ToCfg,PeakH,PeakVa,PeakGammaDeg,PeakThetaDeg,PeakQDps,PeakNorm,PeakElev,PeakThr,PredMax,QpMaxMs, ...
    'VariableNames',{'event_group','t_s','window_end_s','release_count','from_cfg','to_cfg','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps', ...
    'peak_primary_normalized','peak_abs_elevator','peak_throttle','prediction_error_norm_max','qp_time_max_ms'});
end

function [dtRecovery,tRecovery]=localRecoveryTime(t,primaryNorm,lastDrop,threshold,Ts)
idx=find(t>=lastDrop-1e-10);
dtRecovery=NaN; tRecovery=NaN;
for j=1:numel(idx)
    kk=idx(j);
    if primaryNorm(kk)<=threshold && all(primaryNorm(kk:end)<=threshold)
        tRecovery=t(kk);
        dtRecovery=max(0,tRecovery-lastDrop);
        return;
    end
end
% If it never stays below threshold, leave NaN rather than inventing recovery.
if ~isempty(idx) && primaryNorm(end)<=threshold && numel(idx)==1
    tRecovery=t(end); dtRecovery=max(0,tRecovery-lastDrop+Ts);
end
end

function s=localVec(v)
s=strjoin(compose("%.6f",double(v(:).'))," ");
end

function localWriteSummary(report,path)
fid=fopen(path,"w"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
m=report.metrics; g=report.gate; a=report.schedule_audit;
fprintf(fid,"Physics-MPC v0.5.2 configurable drop-timing nonlinear JSBSim mission\n");
fprintf(fid,"scenario=%s\n",report.scenario_name);
fprintf(fid,"drop_schedule_s=%s\n",localVec(report.options.DropTimes_s));
fprintf(fid,"pass=%d\n",report.pass);
fprintf(fid,"oracle=%s\n",report.oracle_version);
fprintf(fid,"common_controller_pass=%d\n",g.common_controller);
fprintf(fid,"common_Q_max_diff=%.9g\n",a.max_Q_diff);
fprintf(fid,"common_R_max_diff=%.9g\n",a.max_R_diff);
fprintf(fid,"horizon=%d\n",a.horizon);
fprintf(fid,"four_drops=%d\n",g.four_drops);
fprintf(fid,"expected_cargo_kg=%s\n",localVec(m.expected_cargo_kg));
fprintf(fid,"observed_drop_kg_individual_if_resolved=%s\n",localVec(m.observed_drop_kg));
fprintf(fid,"expected_group_drop_kg=%s\n",localVec(m.expected_group_drop_kg));
fprintf(fid,"observed_group_drop_kg=%s\n",localVec(m.observed_group_drop_kg));
fprintf(fid,"group_release_count=%s\n",localVec(m.group_release_count));
fprintf(fid,"drop_mass_error_max_kg=%.9g\n",m.drop_mass_error_max_kg);
fprintf(fid,"mass_match_error_max_kg=%.9g\n",m.mass_match_error_max_kg);
fprintf(fid,"cg_match_error_max_m=%.9g\n",m.cg_match_error_max_m);
fprintf(fid,"Iyy_match_error_max_kgm2=%.9g\n",m.Iyy_match_error_max_kgm2);
fprintf(fid,"qp_success_fraction=%.9g\n",m.qp_success_fraction);
fprintf(fid,"qp_time_p95_ms=%.9g\n",m.qp_time_p95_ms);
fprintf(fid,"elevator_saturation_fraction=%.9g\n",m.elevator_saturation_fraction);
fprintf(fid,"throttle_saturation_fraction=%.9g\n",m.throttle_saturation_fraction);
fprintf(fid,"prediction_error_norm_p95=%.9g\n",m.pred_error_norm_p95);
fprintf(fid,"prediction_error_norm_max=%.9g\n",m.pred_error_norm_max);
fprintf(fid,"peak_h_err_m=%.9g\n",m.peak_h_err_m);
fprintf(fid,"peak_Va_err_mps=%.9g\n",m.peak_Va_err_mps);
fprintf(fid,"peak_gamma_err_deg=%.9g\n",m.peak_gamma_err_deg);
fprintf(fid,"peak_theta_err_deg=%.9g\n",m.peak_theta_err_deg);
fprintf(fid,"peak_q_err_dps=%.9g\n",m.peak_q_err_dps);
fprintf(fid,"peak_primary_normalized=%.9g\n",m.peak_primary_normalized);
fprintf(fid,"recovery_time_after_last_drop_s=%.9g\n",m.recovery_time_after_last_drop_s);
fprintf(fid,"final_normalized_inf=%.9g\n",m.final_normalized_inf);
fprintf(fid,"tail5s_normalized_rms=%.9g\n",m.tail5s_normalized_rms);
fn=fieldnames(g); for i=1:numel(fn), fprintf(fid,"gate_%s=%d\n",fn{i},g.(fn{i})); end
end
