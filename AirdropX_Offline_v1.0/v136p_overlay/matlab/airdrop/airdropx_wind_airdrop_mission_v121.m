function report=airdropx_wind_airdrop_mission_v121(projectRoot,opts)
%AIRDROPX_WIND_AIRDROP_MISSION_V121 Full longitudinal wind-aware precision airdrop chain.
% Carrier truth: persistent nonlinear JSBSim wind oracle.
% Flight controller: existing unified Physics-MPC, Np=Nc=100.
% Wind input to guidance: estimator only (GPS groundspeed + TAS + Vz).
% Payload truth: AirdropX calibrated longitudinal ballistic model, driven by
% the actual future wind profile after release. The guidance predictor never
% receives the true wind channel.
arguments
    projectRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
    opts.ScenarioName (1,1) string = "calm"
    opts.UseWindCompensation (1,1) logical = true
    opts.H (1,1) double {mustBeFinite,mustBePositive} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite} = 1.0
    opts.Duration_s (1,1) double {mustBePositive} = 55
    opts.TargetStart_m (1,1) double {mustBeFinite,mustBePositive} = 1200
    opts.TargetSpacing_m (1,1) double {mustBeFinite,mustBePositive} = 80
    opts.SensorNoiseSeed (1,1) double {mustBeFinite} = 1
    opts.SigmaGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.MaxLandingError_m (1,1) double {mustBePositive} = 20
    opts.MaxPredictedImpactErrorAtRelease_m (1,1) double {mustBePositive} = 10
    opts.MaxWindErrorP95_mps (1,1) double {mustBePositive} = 0.75
    opts.MaxPeakPrimaryNormalized (1,1) double {mustBePositive} = 1.0
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05
    opts.ThrowOnFail (1,1) logical = false
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
mode="wind_aware"; if ~opts.UseWindCompensation, mode="no_wind_baseline"; end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v121_wind_airdrop",opts.ScenarioName+"_"+mode); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); localStatus(status,"STARTED "+mode);
report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2, error("AirdropX:WindAirdrop:MissingQuadprog","quadprog is required."); end
    if exist("airdropx_jsbsim_wind_oracle_mex","file")~=3, error("AirdropX:WindAirdrop:MissingWindOracle","Build airdropx_jsbsim_wind_oracle_mex first."); end
    vOracle=string(airdropx_jsbsim_wind_oracle_mex("version"));
    if ~contains(vOracle,"v1.2.1"), error("AirdropX:WindAirdrop:WrongWindOracle","Unexpected wind oracle: %s",vOracle); end
    sched=airdropx_phys_mpc_build_cfg_schedule(opts.BankPath,opts.H,opts.V,opts.FuelScale,Horizon=100);
    models=sched.models; Ts=0.1;
    if abs(models{1}.vertex.p.Ts-Ts)>1e-12, error("AirdropX:WindAirdrop:BadTs","v1.2.1 requires Ts=0.1 s."); end
    targets=opts.TargetStart_m+(0:3)'*opts.TargetSpacing_m;
    if opts.Duration_s<30, error("AirdropX:WindAirdrop:MissionTooShort","Use at least 30 s for wind estimation and four precision releases."); end

    localStatus(status,"WIND_ORACLE_INIT");
    info=airdropx_phys_wind_oracle_init_v121(projectRoot); %#ok<NASGU>
    cleanup=onCleanup(@()localCloseWindOracle()); %#ok<NASGU>
    x=models{1}.ctrl.xref; uTrim=models{1}.ctrl.uref;
    w0=double(airdropx_wind_profile_v111(opts.ScenarioName,0));
    D=airdropx_jsbsim_wind_oracle_mex("start_continuous",x,uTrim,0,opts.FuelScale,w0);
    localStatus(status,"CONTINUOUS_JSBSIM_STARTED");

    est=airdropx_longitudinal_wind_estimator_init_v111(Ts, ...
        SigmaGroundspeed_mps=opts.SigmaGroundspeed_mps,SigmaAirspeed_mps=opts.SigmaAirspeed_mps,SigmaVerticalSpeed_mps=opts.SigmaVerticalSpeed_mps);
    rng(round(opts.SensorNoiseSeed),"twister");
    Nsim=ceil(opts.Duration_s/Ts)+1; t=(0:Nsim-1)'*Ts;
    X=nan(Nsim,7); U=nan(Nsim,2); Err=nan(Nsim,7); Cfg=nan(Nsim,1); PosN=nan(Nsim,1); Vg=nan(Nsim,1); Vz=nan(Nsim,1);
    WindTrue=nan(Nsim,1); WindEst=nan(Nsim,1); WindRate=nan(Nsim,1); WindSigma=nan(Nsim,1); WindRaw=nan(Nsim,1);
    PredImpact=nan(Nsim,1); CurrentTarget=nan(Nsim,1); QPTime=nan(Nsim,1); QPExit=nan(Nsim,1); PredErrInf=nan(Nsim,1); Mass=nan(Nsim,1); Cg=nan(Nsim,1); Iyy=nan(Nsim,1);
    DropEvent=false(Nsim,1); ReleaseIndex=zeros(Nsim,1);
    warm=zeros(models{1}.ctrl.m*models{1}.ctrl.N,1); cfg=0;
    release=struct('cargo',(1:4)','target_m',targets,'released',false(4,1),'t_s',nan(4,1),'release_x_m',nan(4,1), ...
        'release_h_m',nan(4,1),'release_vg_mps',nan(4,1),'release_vz_mps',nan(4,1),'wind_est_mps',nan(4,1),'wind_rate_est_mps2',nan(4,1), ...
        'wind_truth_mps',nan(4,1),'predicted_impact_m',nan(4,1),'truth_impact_m',nan(4,1),'predicted_error_m',nan(4,1),'landing_error_m',nan(4,1), ...
        'predicted_fall_time_s',nan(4,1),'truth_fall_time_s',nan(4,1));
    % Convert struct-of-vectors to simple storage; table is built at the end.
    expectedMass=zeros(5,1); expectedCg=zeros(5,1); expectedIyy=zeros(5,1);
    for c=0:4
        dd=models{c+1}.vertex.trim.diag; expectedMass(c+1)=double(dd.mass_kg); expectedCg(c+1)=double(dd.cg_x_m); expectedIyy(c+1)=double(dd.Iyy_kgm2);
    end

    for k=1:Nsim
        ctrl=models{cfg+1}.ctrl;
        X(k,:)=x.'; Cfg(k)=cfg; PosN(k)=double(D.pos_n_m); Vg(k)=double(D.v_north_mps); Vz(k)=double(D.vz_up_mps);
        WindTrue(k)=double(D.wind_long_mps); Mass(k)=double(D.mass_kg); Cg(k)=double(D.cg_x_m); Iyy(k)=double(D.Iyy_kgm2);
        VaMeas=x(2)+opts.SigmaAirspeed_mps*randn; VzMeas=Vz(k)+opts.SigmaVerticalSpeed_mps*randn; VgMeas=Vg(k)+opts.SigmaGroundspeed_mps*randn;
        [est,eo]=airdropx_longitudinal_wind_estimator_step_v111(est,VaMeas,VzMeas,VgMeas);
        WindEst(k)=eo.wind_est_mps; WindRate(k)=eo.wind_rate_est_mps2; WindSigma(k)=eo.wind_sigma_mps; WindRaw(k)=eo.raw_wind_mps;

        % Release guidance. Truth wind is NOT referenced in this block.
        if cfg<4
            CurrentTarget(k)=targets(cfg+1);
            if opts.UseWindCompensation
                wGuide=WindEst(k); rGuide=WindRate(k); vxGuide=VgMeas;
            else
                wGuide=0; rGuide=0; vxGuide=sqrt(max(VaMeas^2-VzMeas^2,0));
            end
            rs=struct("x_m",PosN(k),"h_m",x(1),"vx_ground_mps",vxGuide,"vz_up_mps",VzMeas,"wind_est_mps",wGuide,"wind_rate_est_mps2",rGuide);
            pred=airdropx_airdrop_predict_impact_v121(rs); PredImpact(k)=pred.impact_x_m;
            releaseWindow=max(2,0.5*max(abs(VgMeas),1)*Ts);
            shouldRelease=pred.impact_x_m>=targets(cfg+1)-releaseWindow;
            if shouldRelease
                j=cfg+1; DropEvent(k)=true; ReleaseIndex(k)=j;
                release.released(j)=true; release.t_s(j)=t(k); release.release_x_m(j)=PosN(k); release.release_h_m(j)=x(1); release.release_vg_mps(j)=Vg(k); release.release_vz_mps(j)=Vz(k);
                release.wind_est_mps(j)=WindEst(k); release.wind_rate_est_mps2(j)=WindRate(k); release.wind_truth_mps(j)=WindTrue(k); release.predicted_impact_m(j)=pred.impact_x_m;
                release.predicted_error_m(j)=pred.impact_x_m-targets(j); release.predicted_fall_time_s(j)=pred.fall_time_s;
                truthRelease=struct("x_m",PosN(k),"h_m",x(1),"vx_ground_mps",Vg(k),"vz_up_mps",Vz(k),"t_s",t(k));
                truth=airdropx_airdrop_truth_impact_v121(opts.ScenarioName,truthRelease);
                release.truth_impact_m(j)=truth.impact_x_m; release.landing_error_m(j)=truth.impact_x_m-targets(j); release.truth_fall_time_s(j)=truth.fall_time_s;
                oldCtrl=ctrl; cfg=cfg+1; ctrl=models{cfg+1}.ctrl; warm=airdropx_phys_mpc_rebase_warmstart(warm,oldCtrl,ctrl);
                fprintf("[V121 DROP] %s %s cargo%d t=%.2f target=%.2f release=%.2f pred=%.2f truth=%.2f miss=%+.2f windEst=%+.2f truthWind=%+.2f\n", ...
                    opts.ScenarioName,mode,j,t(k),targets(j),PosN(k),pred.impact_x_m,truth.impact_x_m,release.landing_error_m(j),WindEst(k),WindTrue(k));
            end
        end

        ctrl=models{cfg+1}.ctrl;
        dx=airdropx_phys_mpc_state_error(x,ctrl.xref); Err(k,:)=dx.';
        sol=airdropx_phys_mpc_solve(ctrl,x,warm); QPTime(k)=sol.solve_time_s; QPExit(k)=sol.exitflag;
        if ~sol.feasible, error("AirdropX:WindAirdrop:QPInfeasible","QP failed at t=%.3f cfg=%d.",t(k),cfg); end
        u=sol.u; U(k,:)=u.';
        if any(u<ctrl.umin-1e-9)||any(u>ctrl.umax+1e-9), error("AirdropX:WindAirdrop:InputViolation","Input bounds violated."); end
        if k==Nsim, break; end
        xPred=ctrl.xref+ctrl.A*dx+ctrl.B*sol.du;
        wCmd=double(airdropx_wind_profile_v111(opts.ScenarioName,t(k)));
        [xNext,DNext]=airdropx_jsbsim_wind_oracle_mex("step_continuous",u,cfg,opts.FuelScale,wCmd,Ts);
        ep=airdropx_phys_mpc_state_error(xNext,xPred); PredErrInf(k)=max(abs(ep)./ctrl.stateScale);
        x=xNext; D=DNext; warm=airdropx_phys_mpc_shift_warmstart(sol.U,ctrl.m,ctrl.N);
        if k==1 || DropEvent(k) || mod(k,50)==0
            fprintf("[V121] %s %s t=%5.1f cfg=%d x=%.1f hErr=%+.3f VaErr=%+.3f wind=%+.2f/%+.2f qp=%.2fms\n",opts.ScenarioName,mode,t(k),cfg,PosN(k),dx(1),dx(2),WindEst(k),WindTrue(k),1e3*sol.solve_time_s);
        end
    end

    valid=~isnan(QPExit); finalCtrl=models{cfg+1}.ctrl; dxFinal=airdropx_phys_mpc_state_error(x,finalCtrl.xref);
    primary=max(abs(Err(valid,1:5))./sched.stateScale(1:5).',[],2); tailN=max(1,round(5/Ts)); ev=find(valid); tailStart=max(1,numel(ev)-tailN+1); tailIdx=ev(tailStart:end);
    windMask=valid & t>=2; windErr=WindEst-WindTrue;
    massRef=nan(Nsim,1); cgRef=massRef; iyyRef=massRef; vv=find(valid); massRef(vv)=expectedMass(Cfg(vv)+1); cgRef(vv)=expectedCg(Cfg(vv)+1); iyyRef(vv)=expectedIyy(Cfg(vv)+1);
    metrics=struct();
    metrics.drops_completed=sum(release.released); metrics.max_landing_error_m=max(abs(release.landing_error_m),[],'omitnan'); metrics.rms_landing_error_m=sqrt(mean(release.landing_error_m.^2,'omitnan'));
    metrics.max_predicted_impact_error_at_release_m=max(abs(release.predicted_error_m),[],'omitnan'); metrics.wind_error_p95_mps=localPercentile(abs(windErr(windMask)),95); metrics.wind_error_rmse_mps=sqrt(mean(windErr(windMask).^2));
    metrics.qp_success_fraction=mean(QPExit(valid)>0); metrics.qp_p95_ms=1e3*localPercentile(QPTime(valid),95); metrics.prediction_error_norm_p95=localPercentile(PredErrInf(valid & isfinite(PredErrInf)),95);
    metrics.peak_primary_normalized=max(primary); metrics.final_normalized_inf=max(abs(dxFinal)./finalCtrl.stateScale); metrics.tail5s_normalized_rms=max(sqrt(mean((Err(tailIdx,:)./sched.stateScale.').^2,1)));
    metrics.mass_match_error_max_kg=max(abs(Mass(valid)-massRef(valid))); metrics.cg_match_error_max_m=max(abs(Cg(valid)-cgRef(valid))); metrics.Iyy_match_error_max_kgm2=max(abs(Iyy(valid)-iyyRef(valid)));
    metrics.truth_crosswind_max_mps=0; metrics.sensor_noise_seed=opts.SensorNoiseSeed;

    gate=struct(); gate.truth_not_used_for_release=1; gate.four_drops=metrics.drops_completed==4; gate.wind_estimation=metrics.wind_error_p95_mps<=opts.MaxWindErrorP95_mps;
    gate.predicted_release=metrics.max_predicted_impact_error_at_release_m<=opts.MaxPredictedImpactErrorAtRelease_m; gate.landing_accuracy=metrics.max_landing_error_m<=opts.MaxLandingError_m;
    gate.qp_all_feasible=all(QPExit(valid)>0); gate.input_bounds=all(U(valid,1)>=-1-1e-9 & U(valid,1)<=1+1e-9 & U(valid,2)>=-1e-9 & U(valid,2)<=1+1e-9);
    gate.carrier_peak=metrics.peak_primary_normalized<=opts.MaxPeakPrimaryNormalized; gate.carrier_final=metrics.final_normalized_inf<=opts.MaxFinalNormalizedInf; gate.carrier_tail=metrics.tail5s_normalized_rms<=opts.MaxTail5sNormalizedRms;
    gate.realtime=metrics.qp_p95_ms<=100; gate.mass_configuration=metrics.mass_match_error_max_kg<=1e-2 && metrics.cg_match_error_max_m<=1e-5 && metrics.Iyy_match_error_max_kgm2<=1e-2;
    vals=struct2cell(gate); gate.pass=all(cellfun(@(z)logical(z),vals));

    Cargo=table(release.cargo,release.target_m,release.released,release.t_s,release.release_x_m,release.release_h_m,release.release_vg_mps,release.release_vz_mps, ...
        release.wind_est_mps,release.wind_rate_est_mps2,release.wind_truth_mps,release.predicted_impact_m,release.truth_impact_m,release.predicted_error_m,release.landing_error_m,release.predicted_fall_time_s,release.truth_fall_time_s, ...
        'VariableNames',{'cargo','target_m','released','release_t_s','release_x_m','release_h_m','release_vg_mps','release_vz_mps','wind_est_mps','wind_rate_est_mps2','wind_truth_mps','predicted_impact_m','truth_impact_m','predicted_error_m','landing_error_m','predicted_fall_time_s','truth_fall_time_s'});
    T=table(t,Cfg,PosN,X(:,1),X(:,2),X(:,3),X(:,4),X(:,5),Vg,Vz,WindTrue,WindRaw,WindEst,WindRate,WindSigma,CurrentTarget,PredImpact,DropEvent,ReleaseIndex,U(:,1),U(:,2),QPExit,QPTime,PredErrInf,Mass,Cg,Iyy, ...
        'VariableNames',{'t_s','cfg','pos_n_m','h_m','Va_mps','gamma_rad','theta_rad','q_radps','Vg_long_mps','Vz_up_mps','wind_truth_mps','wind_raw_mps','wind_est_mps','wind_rate_est_mps2','wind_sigma_mps','current_target_m','predicted_impact_m','drop_event','release_index','elevator_cmd','throttle_cmd','qp_exitflag','qp_time_s','prediction_error_norm_inf','mass_kg','cg_x_m','Iyy_kgm2'});
    writetable(T,fullfile(opts.OutputRoot,"wind_airdrop_timeseries.csv")); writetable(Cargo,fullfile(opts.OutputRoot,"cargo_impacts.csv"));
    localPlot(T,Cargo,opts.OutputRoot,opts.ScenarioName,mode);
    report=struct("version","Physics-MPC v1.2.1 wind-aware precision airdrop","pass",gate.pass,"scenario",opts.ScenarioName,"mode",mode,"oracle_version",vOracle,"options",opts,"metrics",metrics,"gate",gate,"cargo",Cargo,"completed_at",datetime("now"));
    save(fullfile(opts.OutputRoot,"wind_airdrop_mission.mat"),"report","T","Cargo","-v7.3"); localWriteSummary(report,fullfile(opts.OutputRoot,"wind_airdrop_summary.txt"));
    fid=fopen(fullfile(opts.OutputRoot,"scenario_complete.ok"),"w"); if fid>=0, fprintf(fid,"complete %s\n",char(datetime("now"))); fclose(fid); end
    localStatus(status,"COMPLETE pass="+string(report.pass));
    if ~report.pass && opts.ThrowOnFail, error("AirdropX:WindAirdrop:MissionFailed","Mission failed: scenario=%s mode=%s landingMax=%.3g m.",opts.ScenarioName,mode,metrics.max_landing_error_m); end
catch ME
    report.pass=false; report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack); report.completed_at=datetime("now");
    save(fullfile(opts.OutputRoot,"wind_airdrop_failure.mat"),"report","-v7.3"); localStatus(status,"FAILED "+string(ME.identifier)); rethrow(ME);
end
end

function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
q=1+(numel(x)-1)*p/100; a=floor(q); b=ceil(q); if a==b, y=x(a); else, y=x(a)+(q-a)*(x(b)-x(a)); end
end
function localCloseWindOracle()
try, airdropx_jsbsim_wind_oracle_mex("close"); catch, end
end
function localStatus(path,line)
fid=fopen(path,"a"); if fid>=0, fprintf(fid,"%s  %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),char(line)); fclose(fid); end
end
function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.2.1 wind-aware precision airdrop\nscenario=%s\nmode=%s\npass=%d\n",r.scenario,r.mode,r.pass);
fprintf(fid,"truth_wind_used_by_release_guidance=0\ncarrier_truth=persistent_nonlinear_JSBSim\npayload_truth=AirdropX_calibrated_longitudinal_ballistic_model_with_true_future_wind\n");
fn=fieldnames(r.metrics); for i=1:numel(fn), v=r.metrics.(fn{i}); if isscalar(v), fprintf(fid,"%s=%.12g\n",fn{i},v); end, end
gn=fieldnames(r.gate); for i=1:numel(gn), fprintf(fid,"gate_%s=%d\n",gn{i},r.gate.(gn{i})); end
end
function localPlot(T,C,out,scenario,mode)
try
    f=figure("Visible","off","Position",[80 80 1500 950]); tl=tiledlayout(f,2,2,"TileSpacing","compact","Padding","compact"); title(tl,"AirdropX v1.2.1 "+scenario+" / "+mode);
    nexttile; plot(T.t_s,T.wind_truth_mps,"k-",T.t_s,T.wind_est_mps,"-"); grid on; ylabel("wind (m/s)"); xlabel("t (s)"); legend("truth","estimate");
    nexttile; plot(T.pos_n_m,T.h_m); grid on; xlabel("along-track x (m)"); ylabel("h (m)"); hold on; for i=1:height(C), if C.released(i), xline(C.release_x_m(i),'--'); end, end
    nexttile; bar(C.cargo,C.landing_error_m); yline(0); grid on; xlabel("cargo #"); ylabel("landing error (m)");
    nexttile; plot(T.t_s,T.current_target_m-T.predicted_impact_m); yline(0); grid on; xlabel("t (s)"); ylabel("target - predicted impact (m)");
    exportgraphics(f,fullfile(out,"wind_airdrop_validation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:WindAirdrop:PlotFailed","Plot failed: %s",ME.message);
end
end
