function R=airdropx_phys_mpc_base_equivalence_audit_v136p(projectRoot,opts)
%AIRDROPX_PHYS_MPC_BASE_EQUIVALENCE_AUDIT_V136P Same-seed fair-sensor calm A/B.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string = ""
    opts.WindCalibrationPath (1,1) string = ""
    opts.SensorNoiseSeed (1,1) double {mustBeFinite} = 101
    opts.Duration_s (1,1) double {mustBePositive} = 55
    opts.ControlTolerance (1,1) double {mustBePositive} = 1e-10
    opts.StateTolerance (1,1) double {mustBePositive} = 1e-9
    opts.ReleaseTolerance_s (1,1) double {mustBePositive} = 1e-12
end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v136p_paper_base_equivalence_audit"); end
if opts.WindCalibrationPath=="", opts.WindCalibrationPath=fullfile(projectRoot,"matlab","results","physics_mpc_v130_wind_disturbance_calibration","wind_disturbance_model_v130.mat"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
wmDir=fullfile(opts.OutputRoot,"calm_wind_mpc_aware"); legacyDir=fullfile(opts.OutputRoot,"calm_legacy_mpc_aware");
rw=airdropx_wind_airdrop_mission_v136p(projectRoot,OutputRoot=wmDir,ScenarioName="calm",UseWindCompensation=true,UseWindDisturbanceMPC=true,UseFractionalRelease=true,UseWindConfidenceGate=true,UseUnifiedGustRecovery=true,UsePaperSensorModel=true,UseIndependentCargoTruth=false,WindCalibrationPath=opts.WindCalibrationPath,Duration_s=opts.Duration_s,SensorNoiseSeed=opts.SensorNoiseSeed,ThrowOnFail=false);
rl=airdropx_wind_airdrop_mission_v136p(projectRoot,OutputRoot=legacyDir,ScenarioName="calm",UseWindCompensation=true,UseWindDisturbanceMPC=false,UseFractionalRelease=true,UseWindConfidenceGate=true,UseUnifiedGustRecovery=false,UsePaperSensorModel=true,UseIndependentCargoTruth=false,WindCalibrationPath=opts.WindCalibrationPath,Duration_s=opts.Duration_s,SensorNoiseSeed=opts.SensorNoiseSeed,ThrowOnFail=false);
W=load(fullfile(wmDir,"wind_airdrop_mission.mat"),"T","Cargo"); L=load(fullfile(legacyDir,"wind_airdrop_mission.mat"),"T","Cargo"); TW=W.T; TL=L.T;
if height(TW)~=height(TL) || any(abs(TW.t_s-TL.t_s)>1e-14), error("AirdropX:WindMPC:EquivalenceTimebase","A/B time bases differ."); end
stateNames=["h_truth_m","Va_truth_mps","gamma_truth_rad","theta_truth_rad","q_truth_radps","N1_truth","N2_truth","pos_n_truth_m","Vg_truth_mps","Vz_truth_mps", ...
    "h_est_m","Va_est_mps","gamma_est_rad","theta_est_rad","q_est_radps","N1_est","N2_est","pos_n_est_m","Vg_est_mps","Vz_est_mps"];
controlNames=["elevator_cmd","throttle_cmd"];
Dstate=zeros(height(TW),numel(stateNames)); Dcontrol=zeros(height(TW),numel(controlNames));
for j=1:numel(stateNames), Dstate(:,j)=abs(TW.(stateNames(j))-TL.(stateNames(j))); end
for j=1:numel(controlNames), Dcontrol(:,j)=abs(TW.(controlNames(j))-TL.(controlNames(j))); end
maxStatePerRow=max(Dstate,[],2); maxControlPerRow=max(Dcontrol,[],2);
cfgMismatch=TW.cfg~=TL.cfg; dropMismatch=(TW.drop_event~=TL.drop_event) | (TW.release_index~=TL.release_index);
phaseDiff=zeros(height(TW),1); q=TW.drop_event | TL.drop_event; phaseDiff(q)=abs(TW.release_phase_s(q)-TL.release_phase_s(q)); phaseDiff(~isfinite(phaseDiff))=Inf;
windPathUnexpected=logical(TW.wind_mpc_active) | logical(TW.disturbance_evidence_active);
bad=maxControlPerRow>opts.ControlTolerance | maxStatePerRow>opts.StateTolerance | cfgMismatch | dropMismatch | phaseDiff>opts.ReleaseTolerance_s | windPathUnexpected;
first=find(bad,1,"first"); firstTime=NaN; firstReason="none";
if ~isempty(first)
    firstTime=TW.t_s(first);
    if windPathUnexpected(first), firstReason="wind_disturbance_path_active";
    elseif maxControlPerRow(first)>opts.ControlTolerance, firstReason="control";
    elseif maxStatePerRow(first)>opts.StateTolerance, firstReason="state_or_sensor_estimate";
    elseif cfgMismatch(first), firstReason="cfg";
    elseif dropMismatch(first), firstReason="release_event";
    else, firstReason="release_phase"; end
end
Delta=table(TW.t_s,maxControlPerRow,maxStatePerRow,cfgMismatch,dropMismatch,phaseDiff,windPathUnexpected, ...
    'VariableNames',{'t_s','max_abs_control_delta','max_abs_state_or_estimate_delta','cfg_mismatch','release_event_mismatch','release_phase_delta_s','wind_path_unexpected'});
writetable(Delta,fullfile(opts.OutputRoot,"base_equivalence_delta.csv"));
R=struct(); R.version="Physics-MPC v1.3.6-Paper fair-sensor base-equivalence audit"; R.sensor_noise_seed=opts.SensorNoiseSeed; R.wind_report_pass=rw.pass; R.legacy_report_pass=rl.pass;
R.max_abs_control_delta=max(maxControlPerRow,[],'omitnan'); R.max_abs_state_delta=max(maxStatePerRow,[],'omitnan'); R.cfg_mismatch_count=sum(cfgMismatch); R.release_event_mismatch_count=sum(dropMismatch); R.max_release_phase_delta_s=max(phaseDiff(isfinite(phaseDiff)),[],'omitnan');
R.wind_mpc_active_fraction=mean(TW.wind_mpc_active); R.disturbance_evidence_active_fraction=mean(TW.disturbance_evidence_active); R.first_divergence_time_s=firstTime; R.first_divergence_reason=firstReason;
R.pass=R.max_abs_control_delta<=opts.ControlTolerance && R.max_abs_state_delta<=opts.StateTolerance && R.cfg_mismatch_count==0 && R.release_event_mismatch_count==0 && R.max_release_phase_delta_s<=opts.ReleaseTolerance_s && R.wind_mpc_active_fraction==0 && R.disturbance_evidence_active_fraction==0;
save(fullfile(opts.OutputRoot,"base_equivalence_audit.mat"),"R","Delta","-v7.3"); localWrite(R,fullfile(opts.OutputRoot,"base_equivalence_summary.txt"));
end
function localWrite(R,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.3.6-Paper fair-sensor base-equivalence audit\n");
fprintf(fid,"pass=%d\nseed=%g\nmax_abs_control_delta=%.17g\nmax_abs_state_delta=%.17g\ncfg_mismatch_count=%d\nrelease_event_mismatch_count=%d\nmax_release_phase_delta_s=%.17g\nwind_mpc_active_fraction=%.17g\ndisturbance_evidence_active_fraction=%.17g\nfirst_divergence_time_s=%.17g\nfirst_divergence_reason=%s\n",R.pass,R.sensor_noise_seed,R.max_abs_control_delta,R.max_abs_state_delta,R.cfg_mismatch_count,R.release_event_mismatch_count,R.max_release_phase_delta_s,R.wind_mpc_active_fraction,R.disturbance_evidence_active_fraction,R.first_divergence_time_s,char(R.first_divergence_reason));
end
