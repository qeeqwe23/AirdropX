function result = airdropx_auto_final_mission_validation(varargin)
%AIRDROPX_AUTO_FINAL_MISSION_VALIDATION Final cfg0->cfg4 full-mission acceptance test.
%
% This is a pure validation run. It does NOT run bayesopt, retrain a Plant,
% update the LearningBank, or change any controller parameters.
%
% It starts from cfg0 with all four payloads on board, performs all four
% physical drops, switches through cfg0->cfg1->cfg2->cfg3->cfg4, then keeps
% flying without artificial test pulses so the final recovery can be judged.
%
% Typical use after v29 has already made cfg0..cfg4 VERIFIED:
%   r = airdropx_auto_final_mission_validation( ...
%       "ProjectRoot", pwd, ...
%       "OutputRoot", "matlab/results/mpc_auto_200m_all_cfg_v16");
%
% Key outputs under <OutputRoot>/final_mission_validation:
%   final_mission_summary.csv
%   final_mission_gate_report.csv
%   drop_transition_summary.csv
%   final_mission_curves.png
%   FINAL_MISSION_PASS.txt / FINAL_MISSION_FAIL.txt
%   final_mission_result.mat
%   simulation/closed_loop_timeseries.csv

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

outRoot = local_resolve_path(paths.projectRoot, opts.OutputRoot);
checkpointFile = fullfile(outRoot, "airdropx_200m_cfg_checkpoint.mat");
masterFile = fullfile(outRoot, "identified_plants_200m_master.mat");
if ~isfile(checkpointFile)
    error("AirdropX:FinalMission:MissingCheckpoint", "Checkpoint not found: %s", checkpointFile);
end
if ~isfile(masterFile)
    error("AirdropX:FinalMission:MissingMaster", "Master Plant file not found: %s", masterFile);
end

S = load(checkpointFile, "checkpoint");
if ~isfield(S, "checkpoint")
    error("AirdropX:FinalMission:BadCheckpoint", "checkpoint variable missing in %s", checkpointFile);
end
checkpoint = S.checkpoint;
if ~isfield(checkpoint, "status") || numel(checkpoint.status) < 5 || ...
        ~all(string(checkpoint.status(1:5)) == "verified")
    error("AirdropX:FinalMission:NotReady", ...
        "Final mission requires cfg0..cfg4 all VERIFIED. Current status: %s", ...
        strjoin(string(checkpoint.status(:)).', ", "));
end

M = load(masterFile, "result");
if ~isfield(M, "result") || ~isfield(M.result, "trim_bank")
    error("AirdropX:FinalMission:BadMaster", "result.trim_bank missing in %s", masterFile);
end
master = M.result;
if numel(master.trim_bank) < 5
    error("AirdropX:FinalMission:BadMaster", "Master trim bank has fewer than five configurations.");
end

missionRoot = fullfile(outRoot, char(string(opts.ValidationSubdir)));
simRoot = fullfile(missionRoot, "simulation");
if ~isfolder(missionRoot), mkdir(missionRoot); end
if ~isfolder(simRoot), mkdir(simRoot); end

bankMat = local_resolve_cfg4_bank(paths.projectRoot, outRoot, checkpoint);
B = load(bankMat, "controllers", "trim_bank", "mpc_meta");
if ~isfield(B, "controllers") || numel(B.controllers) < 5 || any(cellfun(@isempty, B.controllers(1:5)))
    error("AirdropX:FinalMission:IncompleteBank", ...
        "The cfg4 best bank does not contain all five cfg0..cfg4 controller objects: %s", bankMat);
end

hiddenTrim = NaN;
if isfield(checkpoint, "hidden_elevator_trim")
    hiddenTrim = double(checkpoint.hidden_elevator_trim);
end
if ~isfinite(hiddenTrim), hiddenTrim = 0.0; end
physicalNominals = local_physical_nominals(B, master, hiddenTrim);
throttleNominals = NaN(5,1);
for k = 1:5
    throttleNominals(k) = local_field(master.trim_bank(k), "throttle_cmd", NaN);
end

[authorityByCfg, gainByCfg, integralByCfg, vzLimitByCfg] = local_controller_vectors(checkpoint);
if any(~isfinite(authorityByCfg)) || any(~isfinite(gainByCfg)) || ...
        any(~isfinite(integralByCfg)) || any(~isfinite(vzLimitByCfg))
    error("AirdropX:FinalMission:MissingControllerParameters", ...
        "One or more VERIFIED cfg controller parameter records are incomplete.");
end

lastDropScheduledS = double(opts.DropStartS) + double(opts.DropIntervalS) * (double(opts.TotalDropCount) - 1.0);
stopS = lastDropScheduledS + double(opts.PostFinalDropS);
initialElev = physicalNominals(1) - hiddenTrim;
initialThrottle = throttleNominals(1);

fprintf("\n============================================================\n");
fprintf("[FINAL-MISSION] cfg0 -> cfg4 complete four-drop validation\n");
fprintf("[FINAL-MISSION] H=%.1f m, Va=%.1f m/s, drops=%d, drop schedule %.2f/%.2f s\n", ...
    opts.TargetAltitudeM, opts.TargetAirspeedMps, opts.TotalDropCount, opts.DropStartS, opts.DropIntervalS);
fprintf("[FINAL-MISSION] No artificial test pulses. No optimization. Bank: %s\n", bankMat);
fprintf("============================================================\n");

simResult = airdropx_auto_run_closed_loop( ...
    "ProjectRoot", paths.projectRoot, ...
    "MpcBankMat", bankMat, ...
    "OutputRoot", simRoot, ...
    "CaseId", "final_full_four_drop_mission", ...
    "StopTimeS", stopS, ...
    "FixedConfigId", NaN, ...
    "FixedDropTotal", double(opts.TotalDropCount), ...
    "FixedDropStartS", double(opts.DropStartS), ...
    "FixedDropIntervalS", double(opts.DropIntervalS), ...
    "InitialAltitudeM", double(opts.TargetAltitudeM), ...
    "InitialAirspeedMps", double(opts.TargetAirspeedMps), ...
    "InitialPitchDeg", local_field(master.trim_bank(1), "pitch_deg", 0.0), ...
    "InitialFlightPathDeg", 0.0, ...
    "InitialElevatorDelta", initialElev, ...
    "InitialThrottleCmd", initialThrottle, ...
    "ReferenceMassKg", double(opts.ReferenceMassKg), ...
    "CargoMassKg", double(opts.CargoMassKg), ...
    "HiddenElevatorTrim", hiddenTrim, ...
    "MpcEnableTimeS", double(opts.MpcEnableTimeS), ...
    "MpcAuthorityScale", authorityByCfg(1), ...
    "MpcAuthorityByConfig", authorityByCfg, ...
    "HeightToVzGain", gainByCfg(1), ...
    "HeightToVzGainByConfig", gainByCfg, ...
    "HeightIntegralGain", integralByCfg(1), ...
    "HeightIntegralGainByConfig", integralByCfg, ...
    "HeightVzRefLimitMps", vzLimitByCfg(1), ...
    "HeightVzRefLimitByConfig", vzLimitByCfg, ...
    "BumplessTransitionEnabled", logical(opts.BumplessTransitionEnabled), ...
    "TransitionMoveTransferScale", double(opts.TransitionMoveTransferScale), ...
    "TransitionIntegralTransferScale", double(opts.TransitionIntegralTransferScale), ...
    "V31ContinuousControllerStateEnabled", logical(opts.V31ContinuousControllerStateEnabled), ...
    "V31HeightGovernorEnabled", logical(opts.V31HeightGovernorEnabled), ...
    "V31HeightVzSlewRateMps2", double(opts.V31HeightVzSlewRateMps2), ...
    "V31HeightBiasFraction", double(opts.V31HeightBiasFraction), ...
    "V31HeightBiasLeak", double(opts.V31HeightBiasLeak), ...
    "TestPulse1StartS", Inf, ...
    "TestPulse1DurationS", 0.0, ...
    "TestPulse1Elevator", 0.0, ...
    "TestPulse1Throttle", 0.0, ...
    "TestPulse2StartS", Inf, ...
    "TestPulse2DurationS", 0.0, ...
    "TestPulse2Elevator", 0.0, ...
    "TestPulse2Throttle", 0.0, ...
    "ElevatorDevStepLimit", double(opts.ElevatorDeviationRateLimit), ...
    "ThrottleDevStepLimit", double(opts.ThrottleDeviationRateLimit), ...
    "TrustAltitudeM", 1.0e6, ...
    "TrustAirspeedMps", double(opts.TrustAirspeedMps), ...
    "TrustPitchDeg", double(opts.TrustPitchDeg), ...
    "TrustVzMps", double(opts.TrustVzMps), ...
    "TrustQDps", double(opts.TrustQDps), ...
    "TargetAltitudeM", double(opts.TargetAltitudeM), ...
    "TargetAirspeedMps", double(opts.TargetAirspeedMps), ...
    "TargetPitchDeg", local_field(master.trim_bank(5), "pitch_deg", 0.0), ...
    "UseTrimPitchReference", 1);

T = simResult.timeseries;
[summary, gateReport, dropSummary] = local_score_mission(T, physicalNominals, throttleNominals, opts);
missionGateRatio = local_mission_gate_ratio(gateReport, summary);
summary.mission_gate_ratio = repmat(missionGateRatio,height(summary),1);
summary.bumpless_transition_enabled = repmat(logical(opts.BumplessTransitionEnabled),height(summary),1);
summary.transition_move_transfer_scale = repmat(double(opts.TransitionMoveTransferScale),height(summary),1);
summary.transition_integral_transfer_scale = repmat(double(opts.TransitionIntegralTransferScale),height(summary),1);
summary.v31_continuous_controller_state_enabled = repmat(logical(opts.V31ContinuousControllerStateEnabled),height(summary),1);
summary.v31_height_governor_enabled = repmat(logical(opts.V31HeightGovernorEnabled),height(summary),1);
summary.v31_height_vz_slew_rate_mps2 = repmat(double(opts.V31HeightVzSlewRateMps2),height(summary),1);
writetable(summary, fullfile(missionRoot, "final_mission_summary.csv"));
writetable(gateReport, fullfile(missionRoot, "final_mission_gate_report.csv"));
writetable(dropSummary, fullfile(missionRoot, "drop_transition_summary.csv"));
local_plot_mission(T, dropSummary, physicalNominals, throttleNominals, ...
    fullfile(missionRoot, "final_mission_curves.png"), opts);

missionPass = logical(summary.mission_pass(1));
if missionPass
    flagFile = fullfile(missionRoot, "FINAL_MISSION_PASS.txt");
    otherFlag = fullfile(missionRoot, "FINAL_MISSION_FAIL.txt");
else
    flagFile = fullfile(missionRoot, "FINAL_MISSION_FAIL.txt");
    otherFlag = fullfile(missionRoot, "FINAL_MISSION_PASS.txt");
end
if isfile(otherFlag), delete(otherFlag); end
fid = fopen(flagFile, "w");
if fid >= 0
    fprintf(fid, "MISSION_PASS=%d\n", missionPass);
    fprintf(fid, "timestamp=%s\n", char(datetime("now")));
    fprintf(fid, "target_altitude_m=%.9g\n", opts.TargetAltitudeM);
    fprintf(fid, "target_airspeed_mps=%.9g\n", opts.TargetAirspeedMps);
    fprintf(fid, "final_drop_count=%.9g\n", summary.final_drop_count(1));
    fprintf(fid, "h_rms_m=%.9g\n", summary.mission_h_rms_m(1));
    fprintf(fid, "h_max_abs_m=%.9g\n", summary.mission_h_max_abs_m(1));
    fprintf(fid, "tail_h_error_m=%.9g\n", summary.tail_h_error_m(1));
    fprintf(fid, "tail_vz_mps=%.9g\n", summary.tail_vz_mps(1));
    fprintf(fid, "mission_gate_ratio=%.9g\n", missionGateRatio);
    fprintf(fid, "bumpless_transition_enabled=%d\n", logical(opts.BumplessTransitionEnabled));
    fprintf(fid, "transition_move_transfer_scale=%.9g\n", double(opts.TransitionMoveTransferScale));
    fprintf(fid, "transition_integral_transfer_scale=%.9g\n", double(opts.TransitionIntegralTransferScale));
    fprintf(fid, "v31_continuous_controller_state_enabled=%d\n", logical(opts.V31ContinuousControllerStateEnabled));
    fprintf(fid, "v31_height_governor_enabled=%d\n", logical(opts.V31HeightGovernorEnabled));
    fprintf(fid, "v31_height_vz_slew_rate_mps2=%.8g\n", double(opts.V31HeightVzSlewRateMps2));
    fclose(fid);
end

result = struct();
result.mission_pass = missionPass;
result.mission_gate_ratio = missionGateRatio;
result.bumpless_transition_enabled = logical(opts.BumplessTransitionEnabled);
result.transition_move_transfer_scale = double(opts.TransitionMoveTransferScale);
result.transition_integral_transfer_scale = double(opts.TransitionIntegralTransferScale);
result.v31_continuous_controller_state_enabled = logical(opts.V31ContinuousControllerStateEnabled);
result.v31_height_governor_enabled = logical(opts.V31HeightGovernorEnabled);
result.v31_height_vz_slew_rate_mps2 = double(opts.V31HeightVzSlewRateMps2);
result.summary = summary;
result.gate_report = gateReport;
result.drop_summary = dropSummary;
result.output_root = string(missionRoot);
result.bank_mat = string(bankMat);
result.timeseries_csv = string(simResult.timeseries_csv);
result.summary_csv = string(fullfile(missionRoot, "final_mission_summary.csv"));
result.gate_report_csv = string(fullfile(missionRoot, "final_mission_gate_report.csv"));
result.drop_summary_csv = string(fullfile(missionRoot, "drop_transition_summary.csv"));
result.curves_png = string(fullfile(missionRoot, "final_mission_curves.png"));
save(fullfile(missionRoot, "final_mission_result.mat"), "result", "opts", "-v7.3");

fprintf("[FINAL-MISSION] drops=%d/%d hRMS=%.4f m hMax=%.4f m VaRMS=%.4f m/s vzRMS=%.4f m/s qRMS=%.4f deg/s\n", ...
    round(summary.final_drop_count(1)), opts.TotalDropCount, summary.mission_h_rms_m(1), ...
    summary.mission_h_max_abs_m(1), summary.mission_Va_rms_mps(1), ...
    summary.mission_vz_rms_mps(1), summary.mission_q_rms_dps(1));
fprintf("[FINAL-MISSION] tail h=%.4f m tail vz=%.4f m/s tail q=%.4f deg/s gate=%.4f -> MISSION_PASS=%d\n", ...
    summary.tail_h_error_m(1), summary.tail_vz_mps(1), summary.tail_q_dps(1), missionGateRatio, missionPass);
end

function ratio = local_mission_gate_ratio(G,S)
ratio = Inf;
try
    if isempty(S) || height(S)<1 || logical(S.hard_fail(1)), return; end
    if isempty(G) || height(G)<3, return; end
    a=double(G.actual(3:end)); lim=double(G.limit(3:end));
    r=a./max(lim,eps); r=r(isfinite(r));
    if isempty(r), return; end
    ratio=max(r);
catch
    ratio=Inf;
end
end

function [S, G, D] = local_score_mission(T, eNomByCfg, tNomByCfg, opts)
t = local_col(T, "time_s");
h = local_col(T, "altitude_m");
V = local_col(T, "airspeed_mps");
vz = local_col(T, "vz_up_mps");
q = local_col(T, "q_dps");
pitch = local_col(T, "pitch_deg");
e = local_col(T, "elevator_cmd_norm");
th = local_col(T, "throttle_norm");
be = local_col(T, "bridge_elevator_error");
bt = local_col(T, "bridge_throttle_error");
dropCount = local_col(T, "drop_count");
mass = local_col(T, "mass_kg");
cg = local_col(T, "cg_x_m");

valid = isfinite(t) & isfinite(h) & isfinite(V) & isfinite(vz) & isfinite(q);
missionStart = max(double(opts.MpcEnableTimeS) + 1.0, double(opts.DropStartS) - double(opts.PreDropScoreLeadS));
mission = valid & t >= missionStart;
if nnz(mission) < 20, mission = valid; end
lastT = max(t(valid));
tail = valid & t >= max(lastT - double(opts.FinalTailWindowS), missionStart);
if nnz(tail) < 10, tail = mission; end
head = valid & t >= missionStart & t <= missionStart + double(opts.DriftHeadWindowS);
if nnz(head) < 5
    idx = find(mission);
    head = false(size(t));
    head(idx(1:min(10,numel(idx)))) = true;
end

hErr = h - double(opts.TargetAltitudeM);
vErr = V - double(opts.TargetAirspeedMps);
hRms = local_rms(hErr(mission));
hMax = max(abs(hErr(mission)), [], "omitnan");
hDrift = median(h(tail), "omitnan") - median(h(head), "omitnan");
vaRms = local_rms(vErr(mission));
vaMax = max(abs(vErr(mission)), [], "omitnan");
vzRms = local_rms(vz(mission));
vzMax = max(abs(vz(mission)), [], "omitnan");
qRms = local_rms(q(mission));
qMax = max(abs(q(mission)), [], "omitnan");
pitchMax = max(abs(pitch(mission)), [], "omitnan");
pitchStd = std(pitch(mission), 0, "omitnan");
minH = min(h(mission), [], "omitnan");
maxH = max(h(mission), [], "omitnan");
tailHErr = median(hErr(tail), "omitnan");
tailVz = median(vz(tail), "omitnan");
tailQ = median(q(tail), "omitnan");
finalDropCount = max(dropCount(isfinite(dropCount)), [], "omitnan");
if isempty(finalDropCount) || ~isfinite(finalDropCount), finalDropCount = NaN; end

cfgIdx = round(dropCount) + 1;
cfgIdx(~isfinite(cfgIdx)) = 1;
cfgIdx = min(max(cfgIdx,1),5);
eNom = eNomByCfg(cfgIdx);
tNom = tNomByCfg(cfgIdx);
eDev = abs(e - eNom);
tDev = abs(th - tNom);
maxEDev = max(eDev(mission), [], "omitnan");
maxTDev = max(tDev(mission), [], "omitnan");
maxBE = max(abs(be(mission)), [], "omitnan");
maxBT = max(abs(bt(mission)), [], "omitnan");

D = local_drop_summary(t,h,V,vz,q,pitch,mass,cg,dropCount,hErr,opts);
completeDrops = height(D) >= double(opts.TotalDropCount) && ...
    all(isfinite(D.drop_time_s(1:double(opts.TotalDropCount)))) && ...
    isfinite(finalDropCount) && finalDropCount >= double(opts.TotalDropCount) - 1e-9;

hardFail = ~completeDrops || ~isfinite(minH) || ~isfinite(maxH) || ...
    max(abs([minH maxH] - double(opts.TargetAltitudeM))) > double(opts.HardMaxAltitudeErrorM) || ...
    qRms > double(opts.HardMaxQRmsDps) || pitchMax > double(opts.HardMaxAbsPitchDeg) || ...
    maxBE > double(opts.MaxBridgeError) || maxBT > double(opts.MaxBridgeError);

missionPass = ~hardFail && ...
    hRms <= double(opts.PassMissionAltitudeRmsM) && ...
    hMax <= double(opts.PassMissionAltitudeMaxM) && ...
    abs(hDrift) <= double(opts.PassMissionAltitudeDriftM) && ...
    vaRms <= double(opts.PassMissionAirspeedRmsMps) && ...
    vzRms <= double(opts.PassMissionVzRmsMps) && ...
    qRms <= double(opts.PassMissionQRmsDps) && ...
    abs(tailHErr) <= double(opts.PassTailAltitudeErrorM) && ...
    abs(tailVz) <= double(opts.PassTailVzMps) && ...
    abs(tailQ) <= double(opts.PassTailQDps);

S = table(logical(missionPass),logical(hardFail),logical(completeDrops),finalDropCount, ...
    missionStart,lastT,minH,maxH,hRms,hMax,hDrift,vaRms,vaMax,vzRms,vzMax,qRms,qMax, ...
    pitchMax,pitchStd,tailHErr,tailVz,tailQ,maxEDev,maxTDev,maxBE,maxBT, ...
    median(mass(mission),"omitnan"),median(cg(mission),"omitnan"), ...
    'VariableNames',{'mission_pass','hard_fail','completed_all_drops','final_drop_count', ...
    'score_start_s','stop_time_s','min_altitude_m','max_altitude_m','mission_h_rms_m', ...
    'mission_h_max_abs_m','mission_h_drift_m','mission_Va_rms_mps','mission_Va_max_abs_mps', ...
    'mission_vz_rms_mps','mission_vz_max_abs_mps','mission_q_rms_dps','mission_q_max_abs_dps', ...
    'mission_pitch_max_abs_deg','mission_pitch_std_deg','tail_h_error_m','tail_vz_mps', ...
    'tail_q_dps','max_physical_elevator_deviation','max_throttle_deviation', ...
    'max_bridge_elevator_error','max_bridge_throttle_error','median_mass_kg','median_cg_x_m'});

names = [ ...
    "all_4_drops_completed"; "no_hard_failure"; "mission_h_rms"; "mission_h_max_abs"; ...
    "mission_h_drift"; "mission_Va_rms"; "mission_vz_rms"; "mission_q_rms"; ...
    "tail_h_error"; "tail_vz"; "tail_q" ...
    ];
actual = [double(completeDrops); double(~hardFail); hRms; hMax; abs(hDrift); vaRms; vzRms; qRms; abs(tailHErr); abs(tailVz); abs(tailQ)];
limit = [1;1;double(opts.PassMissionAltitudeRmsM);double(opts.PassMissionAltitudeMaxM); ...
    double(opts.PassMissionAltitudeDriftM);double(opts.PassMissionAirspeedRmsMps); ...
    double(opts.PassMissionVzRmsMps);double(opts.PassMissionQRmsDps); ...
    double(opts.PassTailAltitudeErrorM);double(opts.PassTailVzMps);double(opts.PassTailQDps)];
pass = false(size(actual));
pass(1) = logical(completeDrops);
pass(2) = logical(~hardFail);
pass(3:end) = actual(3:end) <= limit(3:end) + 1e-12;
G = table(names,actual,limit,pass,'VariableNames',{'gate','actual','limit','pass'});
end

function D = local_drop_summary(t,h,V,vz,q,pitch,mass,cg,dropCount,hErr,opts)
n = round(double(opts.TotalDropCount));
D = table();
for k = 1:n
    idx = find(isfinite(dropCount) & dropCount >= k - 1e-9, 1, "first");
    if isempty(idx)
        row = table(k,k,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN, ...
            'VariableNames',{'drop_number','config_after','drop_time_s','altitude_at_drop_m', ...
            'airspeed_at_drop_mps','pitch_at_drop_deg','vz_at_drop_mps','q_at_drop_dps', ...
            'mass_after_drop_kg','cg_after_drop_m','segment_min_altitude_m','segment_max_h_error_m', ...
            'segment_h_rms_m','segment_vz_rms_mps','segment_q_rms_dps','recovery_to_band_s'});
    else
        t0 = t(idx);
        if k < n
            idx2 = find(isfinite(dropCount) & dropCount >= k+1 - 1e-9, 1, "first");
            if isempty(idx2), t1 = max(t(isfinite(t))); else, t1 = t(idx2); end
        else
            t1 = max(t(isfinite(t)));
        end
        seg = isfinite(t) & t >= t0 & t <= t1;
        rec = local_recovery_time(t,hErr,vz,t0,opts);
        row = table(k,k,t0,h(idx),V(idx),pitch(idx),vz(idx),q(idx),mass(idx),cg(idx), ...
            min(h(seg),[],"omitnan"),max(abs(hErr(seg)),[],"omitnan"),local_rms(hErr(seg)), ...
            local_rms(vz(seg)),local_rms(q(seg)),rec, ...
            'VariableNames',{'drop_number','config_after','drop_time_s','altitude_at_drop_m', ...
            'airspeed_at_drop_mps','pitch_at_drop_deg','vz_at_drop_mps','q_at_drop_dps', ...
            'mass_after_drop_kg','cg_after_drop_m','segment_min_altitude_m','segment_max_h_error_m', ...
            'segment_h_rms_m','segment_vz_rms_mps','segment_q_rms_dps','recovery_to_band_s'});
    end
    if isempty(D), D = row; else, D = [D;row]; end %#ok<AGROW>
end
end

function rec = local_recovery_time(t,hErr,vz,eventTime,opts)
rec = NaN;
for i = find(isfinite(t) & t >= eventTime).'
    t0 = t(i);
    mask = isfinite(t) & t >= t0 & t <= t0 + double(opts.RecoveryHoldS);
    if nnz(mask) < 5, continue; end
    if all(abs(hErr(mask)) <= double(opts.RecoveryAltitudeBandM)) && ...
            all(abs(vz(mask)) <= double(opts.RecoveryVzBandMps))
        rec = t0 - eventTime;
        return;
    end
end
end

function local_plot_mission(T,D,eNomByCfg,tNomByCfg,outFile,opts)
t = local_col(T,"time_s");
dropCount = local_col(T,"drop_count");
cfgIdx = round(dropCount)+1; cfgIdx(~isfinite(cfgIdx))=1; cfgIdx=min(max(cfgIdx,1),5);
eNom = eNomByCfg(cfgIdx); tNom=tNomByCfg(cfgIdx);
fig = figure('Visible','off','Color','w','Position',[100 50 1500 1250]);
tl = tiledlayout(9,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t,local_col(T,"altitude_m")); hold on; yline(opts.TargetAltitudeM,'--'); grid on; ylabel('h m'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"airspeed_mps")); hold on; yline(opts.TargetAirspeedMps,'--'); grid on; ylabel('Va'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"vz_up_mps")); hold on; yline(0,'--'); grid on; ylabel('vz'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"pitch_deg")); grid on; ylabel('pitch'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"q_dps")); hold on; yline(0,'--'); grid on; ylabel('q'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"elevator_cmd_norm")); hold on; plot(t,eNom,'--'); grid on; ylabel('elev'); local_drop_lines(D);
nexttile; plot(t,local_col(T,"throttle_norm")); hold on; plot(t,tNom,'--'); grid on; ylabel('thr'); local_drop_lines(D);
nexttile; yyaxis left; plot(t,local_col(T,"mass_kg")); ylabel('mass kg'); yyaxis right; plot(t,local_col(T,"cg_x_m")); ylabel('CG m'); grid on; local_drop_lines(D);
nexttile; stairs(t,dropCount); grid on; ylabel('cfg/drop'); xlabel('time s'); ylim([-0.2 4.4]); local_drop_lines(D);
title(tl,sprintf('FINAL MISSION: %.0f m / %.1f m/s / four consecutive drops / cfg0 -> cfg4',opts.TargetAltitudeM,opts.TargetAirspeedMps),'Interpreter','none');
exportgraphics(fig,outFile,'Resolution',170); close(fig);
end

function local_drop_lines(D)
for k=1:height(D)
    if isfinite(D.drop_time_s(k))
        xline(D.drop_time_s(k),'--',sprintf('Drop %d',D.drop_number(k)), ...
            'LabelOrientation','horizontal','HandleVisibility','off');
    end
end
end

function bankMat = local_resolve_cfg4_bank(projectRoot,outRoot,checkpoint)
candidates = strings(0,1);
if isfield(checkpoint,"best_bank_path") && numel(checkpoint.best_bank_path) >= 5
    candidates(end+1,1) = string(checkpoint.best_bank_path(5)); %#ok<AGROW>
end
candidates(end+1,1) = string(fullfile(outRoot,"cfg4","best_mpc_bank_200m.mat"));
for i=1:numel(candidates)
    p = char(candidates(i));
    if isempty(p), continue; end
    if isfile(p), bankMat = p; return; end
    p2 = fullfile(projectRoot,p);
    if isfile(p2), bankMat = p2; return; end
end
error("AirdropX:FinalMission:MissingCfg4Bank", ...
    "Could not find the VERIFIED cfg4 combined MPC bank. Expected checkpoint.best_bank_path{5} or cfg4/best_mpc_bank_200m.mat.");
end

function p = local_physical_nominals(B,master,hiddenTrim)
p = NaN(5,1);
if isfield(B,"mpc_meta") && isfield(B.mpc_meta,"physical_elevator_nominals")
    v = double(B.mpc_meta.physical_elevator_nominals(:));
    p(1:min(5,numel(v))) = v(1:min(5,numel(v)));
end
for k=1:5
    if isfinite(p(k)), continue; end
    explicit = local_field(master.trim_bank(k),"physical_elevator_cmd",NaN);
    if isfinite(explicit), p(k)=explicit; continue; end
    ext = local_field(master.trim_bank(k),"elevator_cmd",NaN);
    if isfinite(ext), p(k)=hiddenTrim+ext; end
end
if any(~isfinite(p))
    error("AirdropX:FinalMission:MissingNominals","Could not resolve physical elevator nominals for all five configs.");
end
end

function [authority,gain,integral,vzLimit] = local_controller_vectors(checkpoint)
authority=NaN(5,1); gain=NaN(5,1); integral=NaN(5,1); vzLimit=NaN(5,1);
if ~isfield(checkpoint,"best_candidate") || numel(checkpoint.best_candidate)<5, return; end
for k=1:5
    c=checkpoint.best_candidate{k};
    if isempty(c), continue; end
    authority(k)=local_field(c,"Authority",NaN);
    gain(k)=local_field(c,"HeightToVzGain",NaN);
    integral(k)=local_field(c,"HeightIntegralGain",0.0);
    vzLimit(k)=local_field(c,"HeightVzLimit",NaN);
end
end

function v = local_field(s,name,fallback)
try
    v=double(s.(name));
    if isempty(v) || ~isscalar(v) || ~isfinite(v), v=fallback; end
catch
    v=fallback;
end
end

function x = local_col(T,name)
if ismember(string(name),string(T.Properties.VariableNames))
    x=double(T.(char(name))(:));
else
    x=NaN(height(T),1);
end
end

function v=local_rms(x)
x=double(x(:)); x=x(isfinite(x));
if isempty(x), v=NaN; else, v=sqrt(mean(x.^2)); end
end

function p = local_resolve_path(projectRoot,p)
p=char(string(p));
if isempty(p), p=fullfile(projectRoot,"matlab","results","mpc_auto_200m_all_cfg_v16"); return; end
if isfolder(p) || isfile(p), return; end
p2=fullfile(projectRoot,p);
if isfolder(p2) || isfile(p2), p=p2; end
end

function paths=local_paths(projectRoot)
projectRoot=char(string(projectRoot));
if isempty(projectRoot)
    thisDir=fileparts(mfilename("fullpath"));
    matlabDir=fileparts(thisDir);
    projectRoot=fileparts(matlabDir);
else
    matlabDir=fullfile(projectRoot,"matlab");
end
paths=struct("projectRoot",projectRoot,"matlabDir",matlabDir, ...
    "mpcDir",fullfile(matlabDir,"mpc"),"autoDir",fullfile(matlabDir,"mpc_auto"), ...
    "sfuncDir",fullfile(matlabDir,"sfunc_jsbsim"));
end

function opts=local_options(varargin)
opts.ProjectRoot="";
opts.OutputRoot="matlab/results/mpc_auto_200m_all_cfg_v16";
opts.ValidationSubdir="final_mission_validation";
opts.TargetAltitudeM=200.0;
opts.TargetAirspeedMps=50.0;
opts.ReferenceMassKg=3423.0;
opts.CargoMassKg=300.0;
opts.TotalDropCount=4;
opts.MpcEnableTimeS=2.0;
opts.DropStartS=5.0;
opts.DropIntervalS=2.0;
opts.PostFinalDropS=50.0;
opts.PreDropScoreLeadS=1.0;
opts.FinalTailWindowS=10.0;
opts.DriftHeadWindowS=2.0;

% Same local controller envelope used by v29 certification.
opts.ElevatorDeviationRateLimit=0.006;
opts.ThrottleDeviationRateLimit=0.010;
opts.TrustAirspeedMps=4.0;
opts.TrustPitchDeg=4.5;
opts.TrustVzMps=2.5;
opts.TrustQDps=4.0;
opts.MaxBridgeError=0.01;

% Hard safety sanity checks.
opts.HardMaxAltitudeErrorM=50.0;
opts.HardMaxQRmsDps=10.0;
opts.HardMaxAbsPitchDeg=30.0;

% Final-mission transition gates are intentionally wider than the stationary
% per-cfg certification gates because they include all four real mass/CG
% discontinuities. The final tail gates stay as strict as v29 graduation.
opts.PassMissionAltitudeRmsM=2.0;
opts.PassMissionAltitudeMaxM=4.0;
opts.PassMissionAltitudeDriftM=2.5;
opts.PassMissionAirspeedRmsMps=1.5;
opts.PassMissionVzRmsMps=1.0;
opts.PassMissionQRmsDps=1.0;
opts.PassTailAltitudeErrorM=1.0;
opts.PassTailVzMps=0.35;
opts.PassTailQDps=1.0;
opts.RecoveryAltitudeBandM=1.0;
opts.RecoveryVzBandMps=0.35;
opts.RecoveryHoldS=2.0;
% v30.6.2 universal transition policy. Defaults preserve the legacy reset baseline; bumpless transfer is learned only when measured better.
opts.BumplessTransitionEnabled=true;
opts.TransitionMoveTransferScale=0.0;
opts.TransitionIntegralTransferScale=0.0;
opts.V31ContinuousControllerStateEnabled=false;
opts.V31HeightGovernorEnabled=false;
opts.V31HeightVzSlewRateMps2=0.30;
opts.V31HeightBiasFraction=0.70;
opts.V31HeightBiasLeak=1.0;

if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    name=string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name)=varargin{i+1};
end
end
