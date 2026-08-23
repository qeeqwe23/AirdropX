function report=airdropx_phys_runtime_command_closed_loop_v101(projectRoot,opts)
%AIRDROPX_PHYS_RUNTIME_COMMAND_CLOSED_LOOP_V101 Runtime mission with dynamic-feasible H/V reference geometry.
arguments
    projectRoot (1,1) string
    opts.MasterBankPath (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.Duration_s (1,1) double {mustBePositive} = 150
    opts.DropTimes_s (1,4) double = [60 60.2 60.4 60.6]
    opts.CommandPreviewMode (1,1) string = "known_reference"
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05
    opts.MaxPeakPrimaryNormalized (1,1) double {mustBePositive} = 1.0
    opts.MaxComputeP95FractionOfTs (1,1) double {mustBePositive} = 1.0
    opts.MaxMassMatchError_kg (1,1) double {mustBePositive} = 1e-3
    opts.MaxCargoMassError_kg (1,1) double {mustBePositive} = 1e-3
    opts.ThrowOnFail (1,1) logical = false
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
if any(abs(opts.DropTimes_s-[60 60.2 60.4 60.6])>1e-12), error("AirdropX:PhysMPC:RuntimeDropSchedule","v1.0.1 validation fixes releases at [60 60.2 60.4 60.6] s."); end
if opts.Duration_s<140, error("AirdropX:PhysMPC:RuntimeTooShort","v1.0.1 runtime validation needs >=140 s."); end
mode=lower(opts.CommandPreviewMode); if ~ismember(mode,["known_reference","hold_current"]), error("AirdropX:PhysMPC:RuntimePreviewMode","Use known_reference or hold_current."); end
report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2, error("AirdropX:PhysMPC:MissingQuadprog","quadprog is required."); end
    if exist("airdropx_jsbsim_oracle_mex","file")~=3, error("AirdropX:PhysMPC:MissingOracleMex","Validated Oracle MEX is missing."); end
    ov=string(airdropx_jsbsim_oracle_mex("version")); if ~contains(ov,"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","Runtime stage requires Oracle v0.3.3."); end
    bank=airdropx_phys_runtime_prepare_bank_v101(opts.MasterBankPath); Ts=bank.Ts; N=bank.N; Nsim=ceil(opts.Duration_s/Ts); t=(0:Nsim-1)'*Ts;
    [Hcmd,Vcmd,HdotCmd,VdotCmd,profileMeta]=airdropx_phys_runtime_command_profile_v101(opts.ScenarioName,t);
    if numel(Hcmd)~=Nsim || numel(Vcmd)~=Nsim, error("AirdropX:PhysMPC:RuntimeProfileSize","Profile size mismatch."); end
    initStage=airdropx_phys_runtime_interpolate_stage_v101(bank,Hcmd(1),Vcmd(1),0,NeedTerminal=false); x=initStage.xbar;
    info=airdropx_phys_oracle_init(projectRoot); if ~contains(string(info.version),"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","Initialized Oracle is not v0.3.3."); end
    if numel(info.pointmass_lbs)~=4, error("AirdropX:PhysMPC:BadCargoBaseline","Expected four cargo point masses."); end
    expectedCargoKg=double(info.pointmass_lbs(:))*0.45359237; trajectoryOracleSelftest=airdropx_phys_oracle_selftest(x,initStage.ubar,bank.pByCfg{1},ThrowOnFail=true);

    X=zeros(Nsim,7); RefX=zeros(Nsim,7); Err=zeros(Nsim,7); U=zeros(Nsim,2); RefU=zeros(Nsim,2); Cfg=zeros(Nsim,1); DropEvent=false(Nsim,1); Exit=zeros(Nsim,1);
    BuildTime=zeros(Nsim,1); QPTime=zeros(Nsim,1); TotalCompute=zeros(Nsim,1); PredErrInf=zeros(Nsim,1); Mass=nan(Nsim,1); Cg=nan(Nsim,1); Iyy=nan(Nsim,1); SourceCertMin=zeros(Nsim,1); KinResidual=zeros(Nsim,1);
    warmAbs=[]; prevCfg=0;
    for k=1:Nsim
        now=t(k); cfgNow=min(4,sum(now+1e-10>=opts.DropTimes_s)); if cfgNow~=prevCfg, DropEvent(k)=true; fprintf("[DROP] t=%.2f cfg%d->cfg%d\n",now,prevCfg,cfgNow); prevCfg=cfgNow; end
        ft=now+(0:N)*Ts;
        if mode=="known_reference"
            [Hseq,Vseq,HdotSeq,VdotSeq]=airdropx_phys_runtime_command_profile_v101(opts.ScenarioName,ft);
        else
            Hseq=repmat(Hcmd(k),1,N+1); Vseq=repmat(Vcmd(k),1,N+1); HdotSeq=zeros(1,N+1); VdotSeq=zeros(1,N+1);
        end
        cfgSeq=sum(ft(:)>=opts.DropTimes_s,2)'; cfgSeq=min(4,cfgSeq);
        tb=tic; ctrl=airdropx_phys_mpc_runtime_condense_v101(bank,Hseq,Vseq,HdotSeq,VdotSeq,cfgSeq); BuildTime(k)=toc(tb); SourceCertMin(k)=ctrl.source_certified_count_min; KinResidual(k)=ctrl.kinematic_residual_max;
        dx=airdropx_phys_mpc_state_error(x,ctrl.Rstate(:,1)); X(k,:)=x.'; RefX(k,:)=ctrl.Rstate(:,1).'; Err(k,:)=dx.'; RefU(k,:)=ctrl.Rinput(:,1).'; Cfg(k)=cfgNow;
        sol=airdropx_phys_mpc_runtime_solve_v101(ctrl,x,warmAbs); QPTime(k)=sol.solve_time_s; TotalCompute(k)=BuildTime(k)+QPTime(k); Exit(k)=sol.exitflag;
        if ~sol.feasible, error("AirdropX:PhysMPC:RuntimeQPInfeasible","QP failed at t=%.3f H=%.3f V=%.3f cfg=%d exit=%d.",now,Hcmd(k),Vcmd(k),cfgNow,sol.exitflag); end
        u=sol.u; U(k,:)=u.'; if any(u<bank.umin-1e-9)||any(u>bank.umax+1e-9), error("AirdropX:PhysMPC:RuntimeInputViolation","Hard input bound violated."); end
        xPred=sol.predicted_states(:,1); [xNext,dn]=airdropx_phys_step(x,u,bank.pByCfg{cfgNow+1}); pe=airdropx_phys_mpc_state_error(xNext,xPred); PredErrInf(k)=max(abs(pe)./bank.stateScale);
        Mass(k)=double(dn.mass_kg); Cg(k)=double(dn.cg_x_m); Iyy(k)=double(dn.Iyy_kgm2); if isfield(dn,'algebraic_settle_converged') && ~logical(dn.algebraic_settle_converged), error("AirdropX:PhysMPC:RuntimeAlgebraicClosure","Oracle algebraic closure failed."); end
        x=xNext; [warmAbs,~]=airdropx_phys_mpc_preview_shift_warmstart(sol);
        if k==1 || DropEvent(k) || mod(k,100)==0 || k==Nsim
            fprintf("[V101] %4d/%4d t=%6.1f Hcmd=%7.2f Vcmd=%5.2f cfg=%d hErr=% .3f VaErr=% .3f gammaRef=% .3fdeg qErr=% .3fdeg/s total=%.2fms pred=%.3g\n", ...
                k,Nsim,now,Hcmd(k),Vcmd(k),cfgNow,dx(1),dx(2),rad2deg(ctrl.Rstate(3,1)),rad2deg(dx(5)),1e3*TotalCompute(k),PredErrInf(k));
        end
    end
    [Hf,Vf,Hdf,Vdf]=airdropx_phys_runtime_command_profile_v101(opts.ScenarioName,t(end)); %#ok<ASGLU>
    cf=4; ctrlF=airdropx_phys_mpc_runtime_condense_v101(bank,repmat(Hf,1,N+1),repmat(Vf,1,N+1),zeros(1,N+1),zeros(1,N+1),repmat(cf,1,N+1)); dxFinal=airdropx_phys_mpc_state_error(x,ctrlF.Rstate(:,1));
    primaryNorm=max(abs(Err(:,1:5))./bank.stateScale(1:5).',[],2); tailStart=max(1,Nsim-round(5/Ts)+1); finalNorm=max(abs(dxFinal)./bank.stateScale); tailRms=max(sqrt(mean((Err(tailStart:end,:)./bank.stateScale.').^2,1)));
    massRef=bank.expectedMass(Cfg+1); cgRef=bank.expectedCg(Cfg+1); iyyRef=bank.expectedIyy(Cfg+1); massErr=Mass-massRef; cgErr=Cg-cgRef; iyyErr=Iyy-iyyRef;
    ev=find(DropEvent); observed=zeros(4,1); dropErr=zeros(4,1); for j=1:numel(ev), kk=ev(j); to=Cfg(kk); if kk==1, pm=bank.expectedMass(to); else, pm=Mass(kk-1); end; observed(to)=pm-Mass(kk); dropErr(to)=observed(to)-expectedCargoKg(to); end
    gammaRefDeg=rad2deg(RefX(:,3)); thetaRefDeg=rad2deg(RefX(:,4)); qRefDps=rad2deg(RefX(:,5)); kinematicResidual=Vcmd.*sin(RefX(:,3))-HdotCmd;
    metrics=struct("peak_h_err_m",max(abs(Err(:,1))),"peak_Va_err_mps",max(abs(Err(:,2))),"peak_gamma_err_deg",max(abs(rad2deg(Err(:,3)))),"peak_theta_err_deg",max(abs(rad2deg(Err(:,4)))),"peak_q_err_dps",max(abs(rad2deg(Err(:,5)))), ...
        "peak_primary_normalized",max(primaryNorm),"final_normalized_inf",finalNorm,"tail5s_normalized_rms",tailRms,"qp_success_fraction",mean(Exit>0), ...
        "model_build_mean_ms",1e3*mean(BuildTime),"model_build_p95_ms",1e3*localPercentile(BuildTime,95),"qp_time_p95_ms",1e3*localPercentile(QPTime,95),"total_compute_p95_ms",1e3*localPercentile(TotalCompute,95),"total_compute_max_ms",1e3*max(TotalCompute), ...
        "prediction_error_norm_p95",localPercentile(PredErrInf,95),"prediction_error_norm_max",max(PredErrInf),"mass_match_error_max_kg",max(abs(massErr)),"cg_match_error_max_m",max(abs(cgErr)),"Iyy_match_error_max_kgm2",max(abs(iyyErr)), ...
        "expected_cargo_kg",expectedCargoKg,"observed_drop_kg",observed,"drop_mass_error_max_kg",max(abs(dropErr)),"H_cmd_min",min(Hcmd),"H_cmd_max",max(Hcmd),"V_cmd_min",min(Vcmd),"V_cmd_max",max(Vcmd), ...
        "H_cmd_rate_max_mps",max(abs(HdotCmd)),"V_cmd_rate_max_mps2",max(abs(VdotCmd)),"gamma_ref_peak_abs_deg",max(abs(gammaRefDeg)),"theta_ref_peak_abs_deg",max(abs(thetaRefDeg)),"q_ref_peak_abs_dps",max(abs(qRefDps)), ...
        "reference_kinematic_residual_max_mps",max(abs(kinematicResidual)),"horizon_kinematic_residual_max_mps",max(KinResidual),"source_certified_corner_count_min",min(SourceCertMin),"controller_restart_count",0,"retrim_count",0,"reid_count",0);
    gate=struct(); gate.four_drops=numel(ev)==4 && Cfg(end)==4; gate.mass_configuration=max(abs(massErr))<=opts.MaxMassMatchError_kg && max(abs(cgErr))<=1e-9 && max(abs(iyyErr))<=1e-6; gate.cargo_mass=max(abs(dropErr))<=opts.MaxCargoMassError_kg;
    gate.qp_all_feasible=all(Exit>0); gate.hard_input_bounds=all(U(:,1)>=-1-1e-9 & U(:,1)<=1+1e-9 & U(:,2)>=-1e-9 & U(:,2)<=1+1e-9); gate.finite=all(isfinite(X),'all')&&all(isfinite(U),'all');
    gate.reference_kinematics=metrics.reference_kinematic_residual_max_mps<=1e-9 && metrics.horizon_kinematic_residual_max_mps<=1e-9; gate.peak_primary=metrics.peak_primary_normalized<=opts.MaxPeakPrimaryNormalized; gate.final_normalized=finalNorm<=opts.MaxFinalNormalizedInf; gate.tail5s=tailRms<=opts.MaxTail5sNormalizedRms;
    gate.realtime_solver=metrics.qp_time_p95_ms<=1000*Ts*opts.MaxComputeP95FractionOfTs; gate.realtime_total=metrics.total_compute_p95_ms<=1000*Ts*opts.MaxComputeP95FractionOfTs; gate.no_restart=metrics.controller_restart_count==0 && metrics.retrim_count==0 && metrics.reid_count==0; vals=struct2cell(gate); gate.pass=all(cellfun(@(z)logical(z),vals));
    T=table(t,Hcmd,HdotCmd,Vcmd,VdotCmd,Cfg,X(:,1),RefX(:,1),Err(:,1),X(:,2),RefX(:,2),Err(:,2),rad2deg(X(:,3)),gammaRefDeg,rad2deg(Err(:,3)),rad2deg(X(:,4)),thetaRefDeg,rad2deg(Err(:,4)),rad2deg(X(:,5)),qRefDps,rad2deg(Err(:,5)),U(:,1),U(:,2),RefU(:,1),RefU(:,2),Mass,Cg,Iyy,Exit,1e3*BuildTime,1e3*QPTime,1e3*TotalCompute,PredErrInf,SourceCertMin, ...
        'VariableNames',{'t_s','H_cmd_m','H_cmd_rate_mps','Va_cmd_mps','Va_cmd_rate_mps2','cfg','h_m','h_ref_m','h_err_m','Va_mps','Va_ref_mps','Va_err_mps','gamma_deg','gamma_ref_deg','gamma_err_deg','theta_deg','theta_ref_deg','theta_err_deg','q_dps','q_ref_dps','q_err_dps','elevator_cmd','throttle_cmd','elevator_trim','throttle_trim','mass_kg','cg_x_m','Iyy_kgm2','qp_exitflag','model_build_ms','qp_solve_ms','total_compute_ms','prediction_error_norm_inf','source_certified_corner_count_min'});
    writetable(T,fullfile(opts.OutputRoot,'runtime_command_timeseries.csv')); localPlot(T,opts.OutputRoot,opts.ScenarioName,opts.DropTimes_s);
    report=struct("version","Physics-MPC v1.0.1 dynamic-feasible runtime H/V command mission","pass",gate.pass,"scenario",opts.ScenarioName,"preview_mode",mode,"reference_mode","dynamic_feasible","options",opts,"profile_meta",profileMeta,"bank_audit",bank.audit,"oracle_version",ov,"oracle_selftest",trajectoryOracleSelftest,"metrics",metrics,"gate",gate,"completed_at",datetime("now"));
    save(fullfile(opts.OutputRoot,'runtime_command_mission.mat'),'report','T','-v7.3'); localWrite(report,fullfile(opts.OutputRoot,'runtime_command_summary.txt')); if ~report.pass && opts.ThrowOnFail, error("AirdropX:PhysMPC:RuntimeMissionFailed","Runtime command mission failed gates."); end
catch ME
    report.pass=false; report.completed_at=datetime("now"); report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack); save(fullfile(opts.OutputRoot,'runtime_command_mission_failure.mat'),'report','-v7.3'); rethrow(ME);
end
end

function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), y=NaN; return; end; if numel(x)==1, y=x(1); return; end; pos=1+(numel(x)-1)*p/100; lo=floor(pos); hi=ceil(pos); if lo==hi, y=x(lo); else, y=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function localWrite(r,path)
fid=fopen(path,'w'); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
m=r.metrics; fprintf(fid,'Physics-MPC v1.0.1 dynamic-feasible runtime H/V command mission\nscenario=%s\npreview_mode=%s\nreference_mode=dynamic_feasible\npass=%d\n',r.scenario,r.preview_mode,r.pass); fprintf(fid,'drop_schedule_s=60 60.2 60.4 60.6\nNp=100\nNc=100\nq_soft=0\n');
fprintf(fid,'H_cmd_range=[%.9g %.9g]\nV_cmd_range=[%.9g %.9g]\nH_cmd_rate_max_mps=%.9g\nV_cmd_rate_max_mps2=%.9g\n',m.H_cmd_min,m.H_cmd_max,m.V_cmd_min,m.V_cmd_max,m.H_cmd_rate_max_mps,m.V_cmd_rate_max_mps2);
fprintf(fid,'gamma_ref_peak_abs_deg=%.9g\ntheta_ref_peak_abs_deg=%.9g\nq_ref_peak_abs_dps=%.9g\nreference_kinematic_residual_max_mps=%.9g\n',m.gamma_ref_peak_abs_deg,m.theta_ref_peak_abs_deg,m.q_ref_peak_abs_dps,m.reference_kinematic_residual_max_mps);
fprintf(fid,'peak_h_err_m=%.9g\npeak_Va_err_mps=%.9g\npeak_gamma_err_deg=%.9g\npeak_theta_err_deg=%.9g\npeak_q_err_dps=%.9g\npeak_primary_normalized=%.9g\nfinal_normalized_inf=%.9g\ntail5s_normalized_rms=%.9g\n',m.peak_h_err_m,m.peak_Va_err_mps,m.peak_gamma_err_deg,m.peak_theta_err_deg,m.peak_q_err_dps,m.peak_primary_normalized,m.final_normalized_inf,m.tail5s_normalized_rms);
fprintf(fid,'model_build_p95_ms=%.9g\nqp_time_p95_ms=%.9g\ntotal_compute_p95_ms=%.9g\ntotal_compute_max_ms=%.9g\nprediction_error_norm_max=%.9g\nsource_certified_corner_count_min=%d\n',m.model_build_p95_ms,m.qp_time_p95_ms,m.total_compute_p95_ms,m.total_compute_max_ms,m.prediction_error_norm_max,m.source_certified_corner_count_min);
fn=fieldnames(r.gate); for i=1:numel(fn), fprintf(fid,'gate_%s=%d\n',fn{i},r.gate.(fn{i})); end
end

function localPlot(T,out,name,dropTimes)
try
    f=figure('Visible','off','Position',[100 100 1500 1100]); tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact'); title(tl,'Physics-MPC v1.0.1 dynamic feasible reference - '+name);
    nexttile; plot(T.t_s,T.H_cmd_m,'--',T.t_s,T.h_m,'-'); ylabel('H (m)'); xlabel('t (s)'); legend('cmd','actual'); grid on;
    nexttile; plot(T.t_s,T.Va_cmd_mps,'--',T.t_s,T.Va_mps,'-'); ylabel('Va (m/s)'); xlabel('t (s)'); legend('cmd','actual'); grid on;
    nexttile; plot(T.t_s,T.gamma_ref_deg,'--',T.t_s,T.gamma_deg,'-',T.t_s,T.gamma_err_deg,':'); ylabel('gamma (deg)'); xlabel('t (s)'); legend('ref','actual','error'); grid on;
    nexttile; plot(T.t_s,T.q_ref_dps,'--',T.t_s,T.q_dps,'-',T.t_s,T.q_err_dps,':'); ylabel('q (deg/s)'); xlabel('t (s)'); legend('ref','actual','error'); grid on;
    nexttile; plot(T.t_s,T.elevator_cmd,T.t_s,T.throttle_cmd); ylabel('commands'); xlabel('t (s)'); legend('elev','thr'); grid on;
    nexttile; plot(T.t_s,T.total_compute_ms,T.t_s,T.qp_solve_ms); yline(100,'--'); ylabel('compute (ms)'); xlabel('t (s)'); legend('total','QP','Ts'); grid on;
    for ax=findall(f,'Type','axes')', for d=dropTimes, xline(ax,d,':'); end, end; exportgraphics(f,fullfile(out,'runtime_command_tracking.png'),'Resolution',160); close(f);
catch ME, warning("AirdropX:PhysMPC:RuntimePlotFailed","Runtime plot failed: %s",ME.message); end
end
