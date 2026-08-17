function report = audit_v55_cfg3_cfg4_no_mex(varargin)
%AUDIT_V55_CFG3_CFG4_NO_MEX V55 cfg3/cfg4 longitudinal audit without rebuilding MEX.
%
% Uses the existing compiled sfun_airdropx_jsbsim MEX and the existing
% 20-output interface only.  It performs zero-excitation direct-ID runs,
% estimates qdot from the logged pitch rate, estimates alpha from pitch and
% flight-path angle, and computes mass/CG/Iyy offline from MQ9_Reaper.xml.
%
% IMPORTANT: this is a read-only diagnostic for v32 learning memory.
% It does not edit trim checkpoints, ID data, controller memory, or
% certificates.

opts = local_options(varargin{:});
projectRoot = string(opts.ProjectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath"));
    matlabDir = fileparts(thisDir);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end
addpath(matlabDir);
addpath(fullfile(matlabDir, "mpc"));
addpath(fullfile(matlabDir, "mpc_auto"));
addpath(fullfile(matlabDir, "sfunc_jsbsim"));

oldFig = get(groot, "defaultFigureVisible");
figCleanup = onCleanup(@() set(groot, "defaultFigureVisible", oldFig)); %#ok<NASGU>
set(groot, "defaultFigureVisible", "off");

memoryRoot = fullfile(matlabDir, "results", "mpc_auto_v32_clean");
nodeRoot = fullfile(memoryRoot, "knowledge_bank", "physics", sprintf("V%.3f", opts.SpeedMps));
[bank, bankSource] = local_load_preparation_bank(nodeRoot);
if isempty(bank) || numel(bank) < 4
    error("AirdropX:V55NoMexAudit:BadBank", ...
        "Could not recover V55 cfg0..cfg3 trim bank from %s", bankSource);
end

stamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
if strlength(string(opts.OutputRoot)) == 0
    outputRoot = fullfile(memoryRoot, "audits", "v55_cfg3_cfg4_no_mex_" + string(stamp));
else
    outputRoot = string(opts.OutputRoot);
end
if ~isfolder(outputRoot), mkdir(outputRoot); end

cfg2 = bank(3);
cfg3 = bank(4);
e2 = local_field(cfg2, "elevator_cmd", NaN);
e3 = local_field(cfg3, "elevator_cmd", NaN);
t2 = local_field(cfg2, "throttle_cmd", NaN);
t3 = local_field(cfg3, "throttle_cmd", NaN);
if ~all(isfinite([e2 e3 t2 t3]))
    error("AirdropX:V55NoMexAudit:MissingTrim", ...
        "cfg2/cfg3 trim controls are missing or non-finite in %s", bankSource);
end

ePred = e3 + (e3 - e2);
tPred = min(max(t3 + (t3 - t2), opts.ThrottleLimits(1)), opts.ThrottleLimits(2));
scanLo = max(opts.ElevatorLimits(1), min(e3, ePred) - opts.ElevatorBelowSpan);
scanHi = min(opts.ElevatorLimits(2), max(e3, ePred) + opts.ElevatorAboveSpan);
eCandidates = unique(round(linspace(scanLo, scanHi, opts.ElevatorSamples), 5), "stable");
if all(abs(eCandidates-e3) > 1e-8), eCandidates(end+1) = e3; end %#ok<AGROW>
if all(abs(eCandidates-ePred) > 1e-8)
    eCandidates(end+1) = min(max(ePred, opts.ElevatorLimits(1)), opts.ElevatorLimits(2)); %#ok<AGROW>
end
eCandidates = unique(sort(eCandidates));

fprintf("\n=== V55 cfg3/cfg4 NO-MEX longitudinal audit ===\n");
fprintf("Existing MEX only; no build/mex command will be called.\n");
fprintf("V32 memory is read-only: %s\n", memoryRoot);
fprintf("Trim bank: %s\n", bankSource);
fprintf("Output: %s\n", outputRoot);
fprintf("cfg3 external trim: elevator=%+.6f throttle=%.6f\n", e3, t3);
fprintf("cfg4 continuation center (diagnostic only): elevator=%+.6f throttle=%.6f\n", ePred, tPred);
fprintf("External elevator candidates: %s\n\n", mat2str(eCandidates,5));

% Static XML physics estimate. This includes fuel and the actual remaining
% cargo point masses. It is independent of the 20-output metadata mass.
xmlPhysics = local_xml_physics(projectRoot);
staticTable = local_static_cfg_table(xmlPhysics, [3 4]);
writetable(staticTable, fullfile(outputRoot, "xml_estimated_mass_cg_inertia.csv"));

% Known-good cfg3 reference.
refRoot = fullfile(outputRoot, "cfg3_reference");
refResult = local_run_one(projectRoot, refRoot, 3, cfg3, bank, opts, "cfg3_reference");
refSummary = local_summarize_timeseries(refResult.timeseries, 3, opts);
refSummary.run_ok = true;
refSummary.error_message = "";
refSummary.requested_external_elevator = e3;
refSummary.requested_throttle = t3;
refSummary.run_label = "cfg3_reference";
refTable = struct2table(refSummary);
writetable(refTable, fullfile(outputRoot, "cfg3_reference_summary.csv"));

% cfg4 pitch-control scan. No excitation and no MPC learning are involved.
rows = repmat(local_empty_summary(), 0, 1);
for i = 1:numel(eCandidates)
    target = cfg3;
    target.elevator_cmd = eCandidates(i);
    target.throttle_cmd = tPred;
    if isfield(target, "score"), target.score = Inf; end
    if isfield(target, "resume_seed_valid"), target.resume_seed_valid = false; end
    label = sprintf("cfg4_e_%+0.5f", eCandidates(i));
    label = strrep(label, "+", "p");
    label = strrep(label, "-", "m");
    label = strrep(label, ".", "d");
    fprintf("[NO-MEX-AUDIT] cfg4 %d/%d external elevator=%+.5f throttle=%.5f\n", ...
        i, numel(eCandidates), target.elevator_cmd, target.throttle_cmd);
    runRoot = fullfile(outputRoot, label);
    try
        rr = local_run_one(projectRoot, runRoot, 4, target, bank, opts, label);
        sm = local_summarize_timeseries(rr.timeseries, 4, opts);
        sm.run_ok = true;
        sm.error_message = "";
    catch ME
        sm = local_empty_summary();
        sm.run_ok = false;
        sm.error_message = string(ME.identifier) + ": " + string(ME.message);
        fprintf("[NO-MEX-AUDIT]   run failed but sweep continues: %s\n", sm.error_message);
    end
    sm.requested_external_elevator = target.elevator_cmd;
    sm.requested_throttle = target.throttle_cmd;
    sm.run_label = string(label);
    rows(end+1,1) = sm; %#ok<AGROW>
end
scanTable = struct2table(rows);
writetable(scanTable, fullfile(outputRoot, "cfg4_elevator_qdot_scan_no_mex.csv"));

cross = local_find_zero_crossing(scanTable);
comparison = local_mass_comparison(refTable, scanTable, staticTable);
writetable(comparison, fullfile(outputRoot, "metadata_vs_xml_physics.csv"));

reportPath = fullfile(outputRoot, "longitudinal_audit_no_mex_report.txt");
local_write_report(reportPath, projectRoot, bankSource, refTable, scanTable, ...
    staticTable, comparison, cross, ePred, tPred, opts);

report = struct();
report.output_root = string(outputRoot);
report.report_txt = string(reportPath);
report.cfg3_summary_csv = string(fullfile(outputRoot, "cfg3_reference_summary.csv"));
report.cfg4_scan_csv = string(fullfile(outputRoot, "cfg4_elevator_qdot_scan_no_mex.csv"));
report.xml_physics_csv = string(fullfile(outputRoot, "xml_estimated_mass_cg_inertia.csv"));
report.metadata_vs_xml_csv = string(fullfile(outputRoot, "metadata_vs_xml_physics.csv"));
report.qdot_zero_crossing_found = cross.found;
report.qdot_zero_physical_elevator = cross.physical_elevator;
report.qdot_zero_external_elevator = cross.external_elevator;

fprintf("\n[NO-MEX-AUDIT] Complete.\n  %s\n", reportPath);
fprintf("[NO-MEX-AUDIT] No MEX rebuild and no v32 learning-memory modification occurred.\n");
end

function [bank, source] = local_load_preparation_bank(nodeRoot)
bankPath = fullfile(nodeRoot, "v32_trim_bank.mat");
source = string(bankPath);
bank = [];
if isfile(bankPath)
    S = load(bankPath);
    if isfield(S, "bank"), bank = S.bank; end
end
if ~isempty(bank) && numel(bank) >= 4, return; end
cfg3Result = fullfile(nodeRoot, "trim", "cfg3", "trim_result.mat");
source = string(cfg3Result);
if ~isfile(cfg3Result)
    error("AirdropX:V55NoMexAudit:MissingBank", ...
        "Neither node trim bank nor cfg3 trim_result exists. Checked %s and %s", bankPath, cfg3Result);
end
S = load(cfg3Result);
if isfield(S, "trimBank")
    bank = S.trimBank;
elseif isfield(S, "result") && isstruct(S.result) && isfield(S.result, "trim_bank")
    bank = S.result.trim_bank;
end
end

function rr = local_run_one(projectRoot, runRoot, cfgId, targetTrim, bank, opts, label)
if ~isfolder(runRoot), mkdir(runRoot); end
pitch0 = local_field(bank(1), "pitch_deg", 4.0);
gamma0 = local_field(bank(1), "initial_flight_path_deg", 0.0);
rr = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", projectRoot, "Model", opts.Model, ...
    "OutputRoot", runRoot, "RunId", label, "ConfigId", cfgId, ...
    "Trim", targetTrim, "PreparationTrimBank", bank, "UsePreparationTrimSchedule", true, ...
    "StopTimeS", opts.StopTimeS, "RecordStartS", 0.0, "ExportStartS", 0.0, ...
    "PrepDropStartS", opts.PrepDropStartS, "PrepDropIntervalS", opts.PrepDropIntervalS, ...
    "KeepFixedConfigurationOnly", true, "DirectIdMode", true, ...
    "InitialAirspeedMps", opts.SpeedMps, "TargetAirspeedMps", opts.SpeedMps, ...
    "InitialAltitudeM", opts.AltitudeM, "TargetAltitudeM", opts.AltitudeM, ...
    "InitialPitchDeg", pitch0, "InitialFlightPathDeg", gamma0, ...
    "ReferenceMassKg", opts.ReferenceMassKg, "CargoMassKg", opts.CargoMassKg, ...
    "ElevatorAmplitude", 0.0, "ThrottleAmplitude", 0.0, ...
    "ExcitationStartS", opts.StopTimeS + 1.0, ...
    "OperatingPointWindowS", opts.TailWindowS);
end

function sm = local_summarize_timeseries(T, cfgId, opts)
if isempty(T), error("AirdropX:V55NoMexAudit:EmptyRun", "Empty run for cfg%d", cfgId); end
maskCfg = round(double(T.drop_count)) == cfgId;
T = T(maskCfg,:);
if isempty(T), error("AirdropX:V55NoMexAudit:MissingCfgRows", "No cfg%d rows in exported run.", cfgId); end

t = double(T.time_s(:));
tRel = t - t(1);
q = double(T.q_dps(:));
va = double(T.airspeed_mps(:));
vz = double(T.vz_up_mps(:));
pitch = double(T.pitch_deg(:));
% The existing MEX exposes its post-internal-trim physical elevator as
% elevator_cmd_norm.  The Direct-ID logged mpc_elevator_to_plant signal is
% the external delta sent INTO the S-function, so prefer elevator_cmd_norm
% for the actual aerodynamic control position.
if ismember("elevator_cmd_norm", string(T.Properties.VariableNames)) && any(isfinite(double(T.elevator_cmd_norm)))
    physicalElev = double(T.elevator_cmd_norm(:));
else
    physicalElev = double(T.elevator_cmd_actual(:));
end
if ismember("throttle_norm", string(T.Properties.VariableNames)) && any(isfinite(double(T.throttle_norm)))
    throttle = double(T.throttle_norm(:));
else
    throttle = double(T.throttle_cmd_actual(:));
end
metadataMass = double(T.mass_kg(:));
metadataCg = double(T.cg_x_m(:));

% qdot from the existing q output.  Smooth only enough to reduce numerical
% differentiation noise; do not erase the initial pitch acceleration.
dt = median(diff(t), "omitnan");
if ~isfinite(dt) || dt <= 0, dt = 1/120; end
win = max(3, 2*floor((opts.QSmoothWindowS/dt)/2)+1);
if numel(q) >= win
    qSmooth = movmean(q, win, "omitnan");
else
    qSmooth = q;
end
qdot = gradient(qSmooth, t);

ratio = vz ./ max(abs(va), 0.1);
ratio = min(max(ratio, -0.999), 0.999);
gammaDeg = asind(ratio);
alphaEstDeg = pitch - gammaDeg;

early = tRel >= opts.EarlyWindowS(1) & tRel <= opts.EarlyWindowS(2);
mid = tRel >= opts.MidWindowS(1) & tRel <= opts.MidWindowS(2);
tail = tRel >= max(0, max(tRel)-opts.TailWindowS);
if nnz(early) < 3, early = tRel <= min(max(tRel),0.35); end
if nnz(mid) < 3, mid = tRel <= min(max(tRel),0.75); end
if nnz(tail) < 3, tail = true(size(tRel)); end

sm = local_empty_summary();
sm.config_id = cfgId;
sm.cfg_entry_time_s = t(1);
sm.duration_in_cfg_s = max(tRel);
sm.physical_elevator_early = median(physicalElev(early), "omitnan");
sm.physical_elevator_tail = median(physicalElev(tail), "omitnan");
sm.throttle_early = median(throttle(early), "omitnan");
sm.metadata_mass_kg = median(metadataMass(tail), "omitnan");
sm.metadata_cg_x_m = median(metadataCg(tail), "omitnan");
sm.alpha_est_early_deg = median(alphaEstDeg(early), "omitnan");
sm.alpha_est_mid_deg = median(alphaEstDeg(mid), "omitnan");
sm.gamma_early_deg = median(gammaDeg(early), "omitnan");
sm.q_early_dps = median(q(early), "omitnan");
sm.qdot_early_deg_s2 = median(qdot(early), "omitnan");
sm.qdot_early_mean_deg_s2 = mean(qdot(early), "omitnan");
sm.qdot_mid_deg_s2 = median(qdot(mid), "omitnan");
sm.q_peak_abs_first1s_dps = max(abs(q(tRel <= min(max(tRel),1.0))), [], "omitnan");
sm.pitch_change_first1s_deg = local_change_at(tRel, pitch, 1.0);
sm.airspeed_change_first1s_mps = local_change_at(tRel, va, 1.0);
sm.vz_change_first1s_mps = local_change_at(tRel, vz, 1.0);
sm.tail_va_mean_mps = mean(va(tail), "omitnan");
sm.tail_va_rms_error_mps = sqrt(mean((va(tail)-opts.SpeedMps).^2, "omitnan"));
sm.tail_vz_mean_mps = mean(vz(tail), "omitnan");
sm.tail_q_mean_dps = mean(q(tail), "omitnan");
sm.tail_q_rms_dps = sqrt(mean(q(tail).^2, "omitnan"));
sm.tail_pitch_drift_deg_s = local_slope(tRel(tail), pitch(tail));

% Save augmented per-run data beside auto_id_timeseries.csv when the caller
% can infer its folder from the run label.  The main script also retains the
% raw exported CSV from airdropx_auto_run_id_experiment.
end

function v = local_change_at(t, y, horizon)
if isempty(t) || isempty(y), v = NaN; return; end
[~,k] = min(abs(t-horizon));
v = double(y(k)-y(1));
end

function s = local_empty_summary()
s = struct( ...
    "run_label", "", "run_ok", false, "error_message", "", "config_id", NaN, ...
    "requested_external_elevator", NaN, "requested_throttle", NaN, ...
    "cfg_entry_time_s", NaN, "duration_in_cfg_s", NaN, ...
    "physical_elevator_early", NaN, "physical_elevator_tail", NaN, "throttle_early", NaN, ...
    "metadata_mass_kg", NaN, "metadata_cg_x_m", NaN, ...
    "alpha_est_early_deg", NaN, "alpha_est_mid_deg", NaN, "gamma_early_deg", NaN, ...
    "q_early_dps", NaN, "qdot_early_deg_s2", NaN, "qdot_early_mean_deg_s2", NaN, "qdot_mid_deg_s2", NaN, ...
    "q_peak_abs_first1s_dps", NaN, "pitch_change_first1s_deg", NaN, ...
    "airspeed_change_first1s_mps", NaN, "vz_change_first1s_mps", NaN, ...
    "tail_va_mean_mps", NaN, "tail_va_rms_error_mps", NaN, "tail_vz_mean_mps", NaN, ...
    "tail_q_mean_dps", NaN, "tail_q_rms_dps", NaN, "tail_pitch_drift_deg_s", NaN);
end

function cross = local_find_zero_crossing(T)
cross = struct("found", false, "physical_elevator", NaN, "external_elevator", NaN, ...
    "bracket_low", NaN, "bracket_high", NaN, "nearest_abs_qdot", NaN, "nearest_physical_elevator", NaN);
if isempty(T), return; end
[e, idx] = sort(double(T.physical_elevator_early));
qdot = double(T.qdot_early_deg_s2(idx));
ext = double(T.requested_external_elevator(idx));
valid = isfinite(e) & isfinite(qdot) & isfinite(ext);
e=e(valid); qdot=qdot(valid); ext=ext(valid);
if isempty(e), return; end
[~,kn] = min(abs(qdot));
cross.nearest_abs_qdot = qdot(kn);
cross.nearest_physical_elevator = e(kn);
for i=1:numel(e)-1
    if qdot(i)==0
        cross.found = true;
        cross.physical_elevator = e(i);
        cross.external_elevator = ext(i);
        cross.bracket_low = e(i);
        cross.bracket_high = e(i);
        return;
    end
    if qdot(i)*qdot(i+1) < 0
        f = -qdot(i)/(qdot(i+1)-qdot(i));
        cross.found=true;
        cross.physical_elevator = e(i) + f*(e(i+1)-e(i));
        cross.external_elevator = ext(i) + f*(ext(i+1)-ext(i));
        cross.bracket_low=e(i); cross.bracket_high=e(i+1);
        return;
    end
end
end

function P = local_xml_physics(projectRoot)
xmlPath = fullfile(projectRoot, "aircraft", "MQ9_Reaper", "MQ9_Reaper.xml");
if ~isfile(xmlPath)
    error("AirdropX:V55NoMexAudit:MissingXML", "Missing aircraft XML: %s", xmlPath);
end
txt = fileread(xmlPath);
P = struct();
P.xml_path = string(xmlPath);
P.empty_weight_lb = local_rx(txt, '<emptywt[^>]*>\s*([-+0-9.eE]+)\s*</emptywt>');
P.empty_iyy_slugft2 = local_rx(txt, '<iyy[^>]*>\s*([-+0-9.eE]+)\s*</iyy>');
P.empty_cg_x_in = local_rx(txt, '(?s)<location\s+name="CG"[^>]*>.*?<x>\s*([-+0-9.eE]+)\s*</x>');
P.empty_cg_z_in = local_rx(txt, '(?s)<location\s+name="CG"[^>]*>.*?<z>\s*([-+0-9.eE]+)\s*</z>');
P.aerorp_x_in = local_rx(txt, '(?s)<location\s+name="AERORP"[^>]*>.*?<x>\s*([-+0-9.eE]+)\s*</x>');

pmBlocks = regexp(txt, '(?s)<pointmass\s+name="([^"]+)"[^>]*>(.*?)</pointmass>', 'tokens');
P.pointmasses = repmat(struct("name","", "weight_lb",NaN,"x_in",NaN,"z_in",NaN),0,1);
for i=1:numel(pmBlocks)
    b = pmBlocks{i}; body=b{2};
    one.name=string(b{1});
    one.weight_lb=local_rx(body,'<weight[^>]*>\s*([-+0-9.eE]+)\s*</weight>');
    one.x_in=local_rx(body,'<x>\s*([-+0-9.eE]+)\s*</x>');
    one.z_in=local_rx(body,'<z>\s*([-+0-9.eE]+)\s*</z>');
    P.pointmasses(end+1,1)=one; %#ok<AGROW>
end

tankBlocks = regexp(txt, '(?s)<tank\s+type="FUEL"[^>]*>(.*?)</tank>', 'tokens');
P.tanks = repmat(struct("contents_lb",NaN,"x_in",NaN,"z_in",NaN),0,1);
for i=1:numel(tankBlocks)
    body=tankBlocks{i}{1};
    one = struct();
    one.contents_lb=local_rx(body,'<contents[^>]*>\s*([-+0-9.eE]+)\s*</contents>');
    one.x_in=local_rx(body,'<x>\s*([-+0-9.eE]+)\s*</x>');
    one.z_in=local_rx(body,'<z>\s*([-+0-9.eE]+)\s*</z>');
    P.tanks(end+1,1)=one; %#ok<AGROW>
end
end

function T = local_static_cfg_table(P, cfgIds)
rows = repmat(struct("config_id",NaN,"estimated_jsbsim_mass_kg",NaN,"estimated_cg_x_m",NaN, ...
    "estimated_cg_z_m",NaN,"estimated_iyy_slugft2",NaN,"fuel_mass_kg",NaN,"remaining_cargo_count",NaN, ...
    "cg_minus_aerorp_m",NaN),0,1);
for cfg=cfgIds
    compsW = P.empty_weight_lb;
    compsX = P.empty_cg_x_in;
    compsZ = P.empty_cg_z_in;
    % triggerDrop removes pointmass indices 0,1,2,3 in order, so cfgN keeps
    % pointmasses N+1..end.
    for i=(cfg+1):numel(P.pointmasses)
        compsW(end+1,1)=P.pointmasses(i).weight_lb; %#ok<AGROW>
        compsX(end+1,1)=P.pointmasses(i).x_in; %#ok<AGROW>
        compsZ(end+1,1)=P.pointmasses(i).z_in; %#ok<AGROW>
    end
    fuelMassKg=0;
    for i=1:numel(P.tanks)
        compsW(end+1,1)=P.tanks(i).contents_lb; %#ok<AGROW>
        compsX(end+1,1)=P.tanks(i).x_in; %#ok<AGROW>
        compsZ(end+1,1)=P.tanks(i).z_in; %#ok<AGROW>
        fuelMassKg=fuelMassKg+P.tanks(i).contents_lb*0.45359237;
    end
    W=sum(compsW);
    cgX=sum(compsW.*compsX)/W;
    cgZ=sum(compsW.*compsZ)/W;
    g0=32.174049;
    % Empty-airframe inertia is specified about empty CG. Shift it to the
    % combined CG, then add point-mass and tank parallel-axis terms.
    mEmptySlug=P.empty_weight_lb/g0;
    iyy=P.empty_iyy_slugft2 + mEmptySlug*(((P.empty_cg_x_in-cgX)/12)^2 + ((P.empty_cg_z_in-cgZ)/12)^2);
    for i=(cfg+1):numel(P.pointmasses)
        m=P.pointmasses(i).weight_lb/g0;
        dx=(P.pointmasses(i).x_in-cgX)/12; dz=(P.pointmasses(i).z_in-cgZ)/12;
        iyy=iyy+m*(dx^2+dz^2);
    end
    for i=1:numel(P.tanks)
        m=P.tanks(i).contents_lb/g0;
        dx=(P.tanks(i).x_in-cgX)/12; dz=(P.tanks(i).z_in-cgZ)/12;
        iyy=iyy+m*(dx^2+dz^2);
    end
    one=struct();
    one.config_id=cfg;
    one.estimated_jsbsim_mass_kg=W*0.45359237;
    one.estimated_cg_x_m=cgX*0.0254;
    one.estimated_cg_z_m=cgZ*0.0254;
    one.estimated_iyy_slugft2=iyy;
    one.fuel_mass_kg=fuelMassKg;
    one.remaining_cargo_count=max(0,numel(P.pointmasses)-cfg);
    one.cg_minus_aerorp_m=(cgX-P.aerorp_x_in)*0.0254;
    rows(end+1,1)=one; %#ok<AGROW>
end
T=struct2table(rows);
end

function T = local_mass_comparison(ref, scan, staticT)
rows=repmat(struct("config_id",NaN,"metadata_mass_kg",NaN,"xml_estimated_jsbsim_mass_kg",NaN, ...
    "mass_gap_est_kg",NaN,"metadata_cg_x_m",NaN,"xml_estimated_cg_x_m",NaN,"cg_gap_est_m",NaN, ...
    "xml_estimated_iyy_slugft2",NaN),0,1);
for cfg=[3 4]
    if cfg==3
        metaMass=ref.metadata_mass_kg(1); metaCg=ref.metadata_cg_x_m(1);
    else
        metaMass=median(scan.metadata_mass_kg,"omitnan"); metaCg=median(scan.metadata_cg_x_m,"omitnan");
    end
    st=staticT(staticT.config_id==cfg,:);
    one.config_id=cfg;
    one.metadata_mass_kg=metaMass;
    one.xml_estimated_jsbsim_mass_kg=st.estimated_jsbsim_mass_kg(1);
    one.mass_gap_est_kg=one.xml_estimated_jsbsim_mass_kg-metaMass;
    one.metadata_cg_x_m=metaCg;
    one.xml_estimated_cg_x_m=st.estimated_cg_x_m(1);
    one.cg_gap_est_m=one.xml_estimated_cg_x_m-metaCg;
    one.xml_estimated_iyy_slugft2=st.estimated_iyy_slugft2(1);
    rows(end+1,1)=one; %#ok<AGROW>
end
T=struct2table(rows);
end

function local_write_report(path, projectRoot, bankSource, ref, scan, staticT, cmp, cross, ePred, tPred, opts)
fid=fopen(path,"w"); if fid<0, error("Could not write %s",path); end
c=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,"AirdropX V55 cfg3/cfg4 longitudinal audit -- NO MEX REBUILD\n");
fprintf(fid,"Generated: %s\n",char(datetime("now")));
fprintf(fid,"Project: %s\n",projectRoot);
fprintf(fid,"Trim source (read-only): %s\n\n",bankSource);
fprintf(fid,"METHOD\n");
fprintf(fid,"  Uses the currently installed compiled sfun_airdropx_jsbsim MEX.\n");
fprintf(fid,"  No mex/build command and no S-function source change are required.\n");
fprintf(fid,"  qdot is numerically estimated from the existing logged q_dps signal.\n");
fprintf(fid,"  alpha_est = pitch - asin(vz/Va); it is a diagnostic approximation.\n");
fprintf(fid,"  JSBSim mass/CG/Iyy are estimated offline from MQ9_Reaper.xml, including initial fuel.\n");
fprintf(fid,"  V32 learning memory is not modified.\n\n");

fprintf(fid,"STATIC XML PHYSICS ESTIMATE\n");
for i=1:height(staticT)
    fprintf(fid,"  cfg%d: mass %.3f kg, CGx %.6f m, Iyy %.3f slug*ft^2, fuel %.3f kg, CG-AERORP %+0.6f m\n", ...
        staticT.config_id(i),staticT.estimated_jsbsim_mass_kg(i),staticT.estimated_cg_x_m(i), ...
        staticT.estimated_iyy_slugft2(i),staticT.fuel_mass_kg(i),staticT.cg_minus_aerorp_m(i));
end
if height(staticT)>=2
    fprintf(fid,"  cfg3->cfg4 estimated Iyy change: %+0.3f %%\n", ...
        100*(staticT.estimated_iyy_slugft2(2)/staticT.estimated_iyy_slugft2(1)-1));
end
fprintf(fid,"\nMETADATA VS XML ESTIMATE\n");
for i=1:height(cmp)
    fprintf(fid,"  cfg%d: output mass %.3f kg vs XML-est %.3f kg (gap %+0.3f kg)\n", ...
        cmp.config_id(i),cmp.metadata_mass_kg(i),cmp.xml_estimated_jsbsim_mass_kg(i),cmp.mass_gap_est_kg(i));
    fprintf(fid,"        output CGx %.6f m vs XML-est %.6f m (gap %+0.6f m)\n", ...
        cmp.metadata_cg_x_m(i),cmp.xml_estimated_cg_x_m(i),cmp.cg_gap_est_m(i));
end

fprintf(fid,"\nCFG3 REFERENCE\n");
fprintf(fid,"  requested external elevator %+.6f, measured physical elevator %+.6f, throttle %.6f\n", ...
    ref.requested_external_elevator(1),ref.physical_elevator_early(1),ref.throttle_early(1));
fprintf(fid,"  early alpha_est %+.3f deg, qdot median %+.4f deg/s^2, q peak first1s %.3f deg/s\n", ...
    ref.alpha_est_early_deg(1),ref.qdot_early_deg_s2(1),ref.q_peak_abs_first1s_dps(1));
fprintf(fid,"  tail Va %.3f, vz %+.3f, q %+.3f\n",ref.tail_va_mean_mps(1),ref.tail_vz_mean_mps(1),ref.tail_q_mean_dps(1));

fprintf(fid,"\nCFG4 ELEVATOR -> QDOT SCAN\n");
fprintf(fid,"  continuation center only: external elevator %+.6f, throttle %.6f\n",ePred,tPred);
fprintf(fid,"  extElev  physicalElev  alphaEstEarly  qdotEarly  qdotMid  qPeak1s  dPitch1s  tailVa  tailVz  tailQ\n");
for i=1:height(scan)
    fprintf(fid,"  %+0.6f  %+0.6f  %+0.3f  %+0.5f  %+0.5f  %0.3f  %+0.3f  %0.3f  %+0.3f  %+0.3f\n", ...
        scan.requested_external_elevator(i),scan.physical_elevator_early(i),scan.alpha_est_early_deg(i), ...
        scan.qdot_early_deg_s2(i),scan.qdot_mid_deg_s2(i),scan.q_peak_abs_first1s_dps(i), ...
        scan.pitch_change_first1s_deg(i),scan.tail_va_mean_mps(i),scan.tail_vz_mean_mps(i),scan.tail_q_mean_dps(i));
end

fprintf(fid,"\nQDOT ZERO-CROSSING TEST\n");
if cross.found
    fprintf(fid,"  FOUND: qdot changes sign. Estimated zero near physical elevator %+.6f (external %+.6f).\n", ...
        cross.physical_elevator,cross.external_elevator);
    fprintf(fid,"  Bracket physical elevator [%+.6f, %+.6f].\n",cross.bracket_low,cross.bracket_high);
    fprintf(fid,"  Interpretation: a pitch-angular-acceleration balance exists inside the tested control range; 55 m/s is NOT yet proven physically untrimmable.\n");
else
    fprintf(fid,"  NOT FOUND: early qdot did not change sign over the tested physical-elevator range.\n");
    fprintf(fid,"  Nearest measured qdot %+.6f deg/s^2 at physical elevator %+.6f.\n", ...
        cross.nearest_abs_qdot,cross.nearest_physical_elevator);
    fprintf(fid,"  Interpretation: this supports a pitch-moment/control-authority boundary hypothesis, but is still a dynamic audit rather than a formal static Cm sweep.\n");
end

fprintf(fid,"\nDECISION GUIDE\n");
fprintf(fid,"  A) qdot sign crossing + modest Iyy change => do not lower Vmax yet; fix/rework cfg4 trim initialization or joint elevator/throttle search.\n");
fprintf(fid,"  B) no qdot sign crossing across a wide physical-elevator range => stronger evidence cfg4@55 lacks pitch-moment balance/control authority.\n");
fprintf(fid,"  C) output mass differs from XML-estimated mass by roughly fuel mass => metadata/controller mass context is inconsistent with JSBSim dynamics and should be unified before final certification.\n");
fprintf(fid,"  D) alpha_est is approximate; use it for trend comparison, not as a formal JSBSim aero/alpha property replacement.\n");
fprintf(fid,"\nSettings: V %.2f m/s, H %.1f m, prep drop start %.2f s, interval %.2f s, stop %.2f s, q smoothing %.3f s.\n", ...
    opts.SpeedMps,opts.AltitudeM,opts.PrepDropStartS,opts.PrepDropIntervalS,opts.StopTimeS,opts.QSmoothWindowS);
end

function v=local_rx(txt,expr)
t=regexp(txt,expr,'tokens','once'); if isempty(t), v=NaN; else, v=str2double(t{1}); end
end
function slope=local_slope(t,y)
t=double(t(:)); y=double(y(:)); m=isfinite(t)&isfinite(y); t=t(m); y=y(m);
if numel(t)<2 || max(t)-min(t)<=eps, slope=NaN; else, p=polyfit(t-t(1),y,1); slope=p(1); end
end
function v=local_field(s,name,fallback)
v=double(fallback); try, if isstruct(s)&&isfield(s,name)&&~isempty(s.(name))&&isfinite(double(s.(name))), v=double(s.(name)); end, catch, end
end
function opts=local_options(varargin)
opts.ProjectRoot=""; opts.OutputRoot=""; opts.Model="airdropx_mpc_id";
opts.SpeedMps=55.0; opts.AltitudeM=200.0; opts.ReferenceMassKg=3423.0; opts.CargoMassKg=300.0;
opts.PrepDropStartS=1.0; opts.PrepDropIntervalS=2.0; opts.StopTimeS=14.0;
opts.EarlyWindowS=[0.08 0.30]; opts.MidWindowS=[0.30 0.70]; opts.TailWindowS=2.0; opts.QSmoothWindowS=0.05;
opts.ElevatorLimits=[-0.75 0.45]; opts.ThrottleLimits=[0.35 0.88]; opts.ElevatorBelowSpan=0.35; opts.ElevatorAboveSpan=0.25; opts.ElevatorSamples=9;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
