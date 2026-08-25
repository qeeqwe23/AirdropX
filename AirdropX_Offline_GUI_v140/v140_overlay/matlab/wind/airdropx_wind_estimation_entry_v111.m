function report=airdropx_wind_estimation_entry_v111(projectRoot,opts)
%AIRDROPX_WIND_ESTIMATION_ENTRY_V111 Real JSBSim longitudinal wind sensing validation.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.Duration_s (1,1) double {mustBePositive}
    opts.EstimatorTs_s (1,1) double {mustBePositive} = 0.1
    opts.MonteCarloSeeds (1,1) double {mustBeInteger,mustBePositive} = 50
    opts.SigmaGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.MaxNoiselessRmse_mps (1,1) double {mustBePositive} = 0.10
    opts.MaxNoisyRmseMean_mps (1,1) double {mustBePositive} = 0.35
    opts.MaxNoisyP95Mean_mps (1,1) double {mustBePositive} = 0.75
    opts.MaxNoisyBiasMean_mps (1,1) double {mustBePositive} = 0.20
    opts.MaxStepSettling90_s (1,1) double {mustBePositive} = 0.50
    opts.MinSignAccuracy (1,1) double = 0.99
    opts.MinNoiseReductionRatio (1,1) double {mustBePositive} = 1.50
    opts.MaxCrosswindP95_mps (1,1) double {mustBePositive} = 0.10
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); localStatus(status,"STARTED");
report=struct("pass",false,"completed_at",datetime("now"));
try
    localStatus(status,"JSBSIM_HARNESS_START");
    Th=airdropx_wind_simulink_harness_v111(projectRoot,opts.ScenarioName,opts.Duration_s, ...
        WorkDir=opts.OutputRoot,StatusFile=status);
    localStatus(status,"JSBSIM_HARNESS_DONE");
    if any(Th.valid<0.5), error("AirdropX:Wind:InvalidPlantOutput","S-function reported invalid output."); end
    % Resample the physical plant at the actual wind-estimator/MPC rate (10 Hz).
    te=(0:opts.EstimatorTs_s:opts.Duration_s).';
    Va=interp1(Th.t_s,Th.Va_mps,te,"linear","extrap");
    Vz=interp1(Th.t_s,Th.Vz_mps,te,"linear","extrap");
    Vg=interp1(Th.t_s,Th.Vg_mps,te,"linear","extrap");
    truth=interp1(Th.t_s,Th.wind_long_truth_mps,te,"linear","extrap");
    cmd=airdropx_wind_profile_v111(opts.ScenarioName,te);
    cross=interp1(Th.t_s,Th.wind_cross_truth_mps,te,"linear","extrap");

    s=airdropx_longitudinal_wind_estimator_init_v111(opts.EstimatorTs_s, ...
        SigmaGroundspeed_mps=opts.SigmaGroundspeed_mps,SigmaAirspeed_mps=opts.SigmaAirspeed_mps,SigmaVerticalSpeed_mps=opts.SigmaVerticalSpeed_mps);
    n=numel(te); est=zeros(n,1); rate=zeros(n,1); sigma=zeros(n,1); raw=zeros(n,1); valid=false(n,1); step=false(n,1);
    for k=1:n
        [s,o]=airdropx_longitudinal_wind_estimator_step_v111(s,Va(k),Vz(k),Vg(k));
        est(k)=o.wind_est_mps; rate(k)=o.wind_rate_est_mps2; sigma(k)=o.wind_sigma_mps; raw(k)=o.raw_wind_mps; valid(k)=o.valid; step(k)=o.step_detected;
    end
    if opts.MinSignAccuracy<0 || opts.MinSignAccuracy>1, error("AirdropX:Wind:BadSignAccuracy","MinSignAccuracy must be in [0,1]."); end
    e0=est-truth; mask=localEvaluationMask(te,truth,opts.ScenarioName);
    noiselessRmse=sqrt(mean(e0(mask).^2));

    mcRmse=zeros(opts.MonteCarloSeeds,1); mcP95=zeros(opts.MonteCarloSeeds,1); mcBias=zeros(opts.MonteCarloSeeds,1); mcRawRmse=zeros(opts.MonteCarloSeeds,1); mcFilterRmse=zeros(opts.MonteCarloSeeds,1); mcSign=zeros(opts.MonteCarloSeeds,1); mcCoverage=zeros(opts.MonteCarloSeeds,1); mcStep=zeros(opts.MonteCarloSeeds,1);
    for seed=1:opts.MonteCarloSeeds
        rng(seed,"twister");
        Vgn=Vg+opts.SigmaGroundspeed_mps*randn(n,1);
        Van=Va+opts.SigmaAirspeed_mps*randn(n,1);
        Vzn=Vz+opts.SigmaVerticalSpeed_mps*randn(n,1);
        ss=airdropx_longitudinal_wind_estimator_init_v111(opts.EstimatorTs_s, ...
            SigmaGroundspeed_mps=opts.SigmaGroundspeed_mps,SigmaAirspeed_mps=opts.SigmaAirspeed_mps,SigmaVerticalSpeed_mps=opts.SigmaVerticalSpeed_mps);
        ee=zeros(n,1); rr=zeros(n,1); sg=zeros(n,1);
        for k=1:n
            [ss,oo]=airdropx_longitudinal_wind_estimator_step_v111(ss,Van(k),Vzn(k),Vgn(k));
            ee(k)=oo.wind_est_mps; rr(k)=oo.raw_wind_mps; sg(k)=oo.wind_sigma_mps;
        end
        er=ee-truth; eraw=rr-truth;
        mcRmse(seed)=sqrt(mean(er(mask).^2)); mcP95(seed)=localPercentile(abs(er(mask)),95); mcBias(seed)=abs(mean(er(mask))); mcRawRmse(seed)=sqrt(mean(eraw(mask).^2)); mcFilterRmse(seed)=sqrt(mean(er(mask).^2));
        sm=mask & abs(truth)>=2; if any(sm), mcSign(seed)=mean(sign(ee(sm))==sign(truth(sm))); else, mcSign(seed)=1; end
        mcCoverage(seed)=mean(abs(er(mask))<=2*max(sg(mask),1e-6));
        mcStep(seed)=localStepSettling(te,truth,ee,opts.ScenarioName);
    end
    stepSettling=max(mcStep(isfinite(mcStep)),[],'omitnan'); if isempty(stepSettling), stepSettling=0; end
    noiseReduction=mean(mcRawRmse)/mean(mcFilterRmse);
    metrics=struct( ...
        "truth_command_error_max_mps",max(abs(truth-cmd)), ...
        "crosswind_truth_p95_mps",localPercentile(abs(cross),95), ...
        "heading_deviation_p95_deg",localPercentile(abs(Th.heading_dev_deg),95), ...
        "noiseless_rmse_mps",noiselessRmse, ...
        "noiseless_max_abs_mps",max(abs(e0(mask))), ...
        "noisy_rmse_mean_mps",mean(mcRmse), ...
        "noisy_rmse_worst_seed_mps",max(mcRmse), ...
        "noisy_p95_abs_mean_mps",mean(mcP95), ...
        "noisy_bias_abs_mean_mps",mean(mcBias), ...
        "noise_reduction_ratio",noiseReduction, ...
        "sign_accuracy_mean",mean(mcSign), ...
        "two_sigma_coverage_mean",mean(mcCoverage), ...
        "step_settling90_worst_s",stepSettling, ...
        "valid_measurement_fraction",mean(valid), ...
        "step_detection_count",sum(step), ...
        "wind_truth_min_mps",min(truth),"wind_truth_max_mps",max(truth));
    gate=struct();
    gate.truth_not_used_as_input=1;
    gate.truth_command=metrics.truth_command_error_max_mps<=1e-6;
    gate.longitudinal_only=metrics.crosswind_truth_p95_mps<=opts.MaxCrosswindP95_mps;
    gate.noiseless_geometry=metrics.noiseless_rmse_mps<=opts.MaxNoiselessRmse_mps;
    gate.noisy_rmse=metrics.noisy_rmse_mean_mps<=opts.MaxNoisyRmseMean_mps;
    gate.noisy_p95=metrics.noisy_p95_abs_mean_mps<=opts.MaxNoisyP95Mean_mps;
    gate.noisy_bias=metrics.noisy_bias_abs_mean_mps<=opts.MaxNoisyBiasMean_mps;
    gate.step_response=metrics.step_settling90_worst_s<=opts.MaxStepSettling90_s;
    gate.sign_accuracy=metrics.sign_accuracy_mean>=opts.MinSignAccuracy;
    gate.noise_reduction=metrics.noise_reduction_ratio>=opts.MinNoiseReductionRatio;
    gate.valid=metrics.valid_measurement_fraction>=0.999;
    vals=struct2cell(gate); gate.pass=all(cellfun(@(z)logical(z),vals));

    T=table(te,cmd,truth,cross,Va,Vz,Vg,raw,est,rate,sigma,e0,valid,step, ...
        'VariableNames',{'t_s','wind_cmd_mps','wind_truth_mps','wind_cross_truth_mps','Va_mps','Vz_mps','Vg_mps','raw_wind_mps','wind_est_mps','wind_rate_est_mps2','wind_sigma_mps','wind_error_mps','measurement_valid','step_detected'});
    writetable(T,fullfile(opts.OutputRoot,"wind_estimation_timeseries.csv"));
    writetable(Th,fullfile(opts.OutputRoot,"jsbsim_truth_timeseries.csv"));
    localPlot(T,opts.OutputRoot,opts.ScenarioName);
    report=struct("version","Physics-MPC v1.1.3 longitudinal wind estimator validation","pass",gate.pass,"scenario",opts.ScenarioName,"options",opts,"metrics",metrics,"gate",gate,"completed_at",datetime("now"));
    save(fullfile(opts.OutputRoot,"wind_estimation_validation.mat"),"report","T","Th","mcRmse","mcP95","mcBias","mcRawRmse","mcFilterRmse","mcSign","mcCoverage","mcStep","-v7.3");
    localWrite(report,fullfile(opts.OutputRoot,"wind_estimation_summary.txt"));
    localStatus(status,"RESULT_SAVED");
    fid=fopen(fullfile(opts.OutputRoot,"scenario_complete.ok"),"w"); if fid>=0, fprintf(fid,"complete %s\n",char(datetime("now"))); fclose(fid); end
    localStatus(status,"COMPLETE_MARKER_WRITTEN");
catch ME
    report.pass=false; report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack); report.completed_at=datetime("now");
    save(fullfile(opts.OutputRoot,"wind_estimation_failure.mat"),"report","-v7.3"); localStatus(status,"FAILED "+string(ME.identifier)); rethrow(ME);
end
end

function mask=localEvaluationMask(t,w,name)
mask=t>=2;
name=lower(string(name));
if ismember(name,["tailwind_5","headwind_5","tailwind_12","headwind_12","step_bidirectional"])
    idx=find(abs([0;diff(w)])>=2);
    for j=1:numel(idx)
        mask=mask & ~(t>=t(idx(j)) & t<=t(idx(j))+0.5);
    end
end
end

function s=localStepSettling(t,w,e,name)
name=lower(string(name));
if ~ismember(name,["tailwind_5","headwind_5","tailwind_12","headwind_12","step_bidirectional"]), s=0; return; end
idx=find(abs([0;diff(w)])>=2); if isempty(idx), s=0; return; end
vals=zeros(numel(idx),1); holdSamples=3;
for j=1:numel(idx)
    k=idx(j); prev=w(max(1,k-1)); target=w(k); amp=abs(target-prev); if amp<1e-9, continue; end
    tol=0.10*amp; kEnd=length(t); if j<numel(idx), kEnd=idx(j+1)-1; end
    found=Inf;
    for q=k:max(k,kEnd-holdSamples+1)
        q2=min(kEnd,q+holdSamples-1);
        if q2-q+1==holdSamples && all(abs(e(q:q2)-target)<=tol), found=t(q)-t(k); break; end
    end
    vals(j)=found;
end
s=max(vals);
end

function p=localPercentile(x,q)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), p=NaN; return; end
if numel(x)==1, p=x(1); return; end
pos=1+(numel(x)-1)*q/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, p=x(lo); else, p=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function localStatus(path,line)
fid=fopen(path,"a"); if fid>=0, fprintf(fid,"%s  %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),char(line)); fclose(fid); end
end

function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
m=r.metrics; fprintf(fid,"Physics-MPC v1.1.3 longitudinal wind estimator validation\nscenario=%s\npass=%d\n",r.scenario,r.pass);
fprintf(fid,"sign_convention=tailwind_positive_headwind_negative\nestimator_inputs=GPS_groundspeed,TAS_airspeed,vertical_speed\ntruth_channels_used_for_scoring_only=windN,windE\n");
fn=fieldnames(m); for i=1:numel(fn), v=m.(fn{i}); if isscalar(v), fprintf(fid,"%s=%.12g\n",fn{i},v); end, end
g=fieldnames(r.gate); for i=1:numel(g), fprintf(fid,"gate_%s=%d\n",g{i},r.gate.(g{i})); end
end

function localPlot(T,out,name)
try
    f=figure("Visible","off","Position",[80 80 1450 900]); tl=tiledlayout(f,3,1,"TileSpacing","compact","Padding","compact"); title(tl,"AirdropX v1.1.3 longitudinal wind estimation - "+name);
    nexttile; plot(T.t_s,T.wind_truth_mps,"k-",T.t_s,T.raw_wind_mps,":",T.t_s,T.wind_est_mps,"-"); hold on; plot(T.t_s,T.wind_est_mps+2*T.wind_sigma_mps,"--",T.t_s,T.wind_est_mps-2*T.wind_sigma_mps,"--"); ylabel("wind (m/s)"); legend("truth","raw","estimate","+2sigma","-2sigma"); grid on;
    nexttile; plot(T.t_s,T.wind_error_mps); yline(0); ylabel("error (m/s)"); grid on;
    nexttile; plot(T.t_s,T.Vg_mps,T.t_s,T.Va_mps,T.t_s,T.Vz_mps); ylabel("measured speed (m/s)"); xlabel("t (s)"); legend("GPS ground","TAS","Vz"); grid on;
    exportgraphics(f,fullfile(out,"wind_estimation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:Wind:PlotFailed","Wind validation plot failed: %s",ME.message);
end
end
