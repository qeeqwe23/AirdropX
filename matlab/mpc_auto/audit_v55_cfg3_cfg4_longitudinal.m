function report = audit_v55_cfg3_cfg4_longitudinal(varargin)
%AUDIT_V55_CFG3_CFG4_LONGITUDINAL Read-only V55 cfg3/cfg4 JSBSim physics audit.
%
% This diagnostic does NOT modify v32 checkpoints, trim banks, MPC memory, or
% certificates. It runs short, zero-excitation direct-ID simulations and uses
% the opt-in AIRDROPX_JSBSIM_AUDIT_CSV hook in sfun_airdropx_jsbsim.cpp to
% record the actual JSBSim mass, CG, inertia and aerodynamic pitch moments.
%
% Typical use:
%   audit_v55_cfg3_cfg4_longitudinal
%
% The companion PowerShell runner rebuilds the instrumented MEX first.

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
bankPath = fullfile(nodeRoot, "v32_trim_bank.mat");
bankSource = string(bankPath);
if isfile(bankPath)
    S = load(bankPath, "bank");
    if isfield(S, "bank")
        bank = S.bank;
    else
        bank = [];
    end
else
    % V55 has not reached the node-level save because cfg4 failed. Recover the
    % preparation bank from the latest successful cfg3 trim_result instead.
    cfg3Result = fullfile(nodeRoot, "trim", "cfg3", "trim_result.mat");
    if ~isfile(cfg3Result)
        error("AirdropX:V55Audit:MissingBank", ...
            "Neither V55 node trim bank nor cfg3 trim_result exists. Checked: %s ; %s", ...
            bankPath, cfg3Result);
    end
    S = load(cfg3Result, "trimBank", "result");
    if isfield(S, "trimBank")
        bank = S.trimBank;
    elseif isfield(S, "result") && isstruct(S.result) && isfield(S.result, "trim_bank")
        bank = S.result.trim_bank;
    else
        bank = [];
    end
    bankSource = string(cfg3Result);
end
if isempty(bank) || numel(bank) < 4
    error("AirdropX:V55Audit:BadBank", ...
        "Could not recover a V55 preparation bank containing cfg0..cfg3 from %s", bankSource);
end

stamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
if strlength(string(opts.OutputRoot)) == 0
    outputRoot = fullfile(memoryRoot, "audits", "v55_cfg3_cfg4_" + string(stamp));
else
    outputRoot = string(opts.OutputRoot);
end
if ~isfolder(outputRoot), mkdir(outputRoot); end

cfg2 = bank(3);
cfg3 = bank(4);
e3 = local_field(cfg3, "elevator_cmd", NaN);
t3 = local_field(cfg3, "throttle_cmd", NaN);
e2 = local_field(cfg2, "elevator_cmd", e3);
t2 = local_field(cfg2, "throttle_cmd", t3);
if ~all(isfinite([e2 e3 t2 t3]))
    error("AirdropX:V55Audit:MissingVerifiedTrim", ...
        "cfg2/cfg3 verified trim controls are missing/non-finite in %s", bankSource);
end

% Continuation is only a diagnostic center. The scan deliberately spans both
% sides of it, so the audit does not assume cfg4 must lie on a linear branch.
ePred = e3 + (e3 - e2);
tPred = t3 + (t3 - t2);
tPred = min(max(tPred, opts.ThrottleLimits(1)), opts.ThrottleLimits(2));
scanLo = max(opts.ElevatorLimits(1), min(e3, ePred) - opts.ElevatorBelowSpan);
scanHi = min(opts.ElevatorLimits(2), max(e3, ePred) + opts.ElevatorAboveSpan);
eCandidates = unique(round(linspace(scanLo, scanHi, opts.ElevatorSamples), 5), "stable");
if all(abs(eCandidates - e3) > 1e-6), eCandidates(end+1) = e3; end %#ok<AGROW>
if all(abs(eCandidates - ePred) > 1e-6), eCandidates(end+1) = min(max(ePred, opts.ElevatorLimits(1)), opts.ElevatorLimits(2)); end %#ok<AGROW>
eCandidates = unique(sort(eCandidates));

fprintf("\n=== AirdropX V55 cfg3/cfg4 longitudinal physics audit ===\n");
fprintf("Read-only v32 memory: %s\n", memoryRoot);
fprintf("Preparation bank source: %s\n", bankSource);
fprintf("Output: %s\n", outputRoot);
fprintf("cfg3 external trim: elevator=%+.6f throttle=%.6f\n", e3, t3);
fprintf("cfg2->cfg3 continuation center for cfg4: elevator=%+.6f throttle=%.6f\n", ePred, tPred);
fprintf("cfg4 elevator scan: %s\n\n", mat2str(eCandidates, 5));

% Run cfg3 once as the known-good reference.
cfg3Root = fullfile(outputRoot, "cfg3_reference");
ref = local_run_one(projectRoot, cfg3Root, 3, cfg3, bank, opts, "cfg3_reference");
refSummary = local_summarize_run(ref.audit, 3, opts);
refSummary.requested_elevator = e3;
refSummary.requested_throttle = t3;
refSummary.run_label = "cfg3_reference";

% Sweep cfg4 elevator while keeping throttle at the cfg2->cfg3 continuation.
scanRows = repmat(local_empty_summary(), 0, 1);
for i = 1:numel(eCandidates)
    target = cfg3;
    target.elevator_cmd = eCandidates(i);
    target.throttle_cmd = tPred;
    if isfield(target, "score"), target.score = Inf; end
    if isfield(target, "resume_seed_valid"), target.resume_seed_valid = false; end
    runLabel = sprintf("cfg4_e_%+0.5f", eCandidates(i));
    runLabel = strrep(runLabel, "+", "p");
    runLabel = strrep(runLabel, "-", "m");
    runLabel = strrep(runLabel, ".", "d");
    oneRoot = fullfile(outputRoot, runLabel);
    fprintf("[V55-AUDIT] cfg4 %d/%d: external elevator=%+.5f throttle=%.5f\n", ...
        i, numel(eCandidates), target.elevator_cmd, target.throttle_cmd);
    one = local_run_one(projectRoot, oneRoot, 4, target, bank, opts, runLabel);
    sm = local_summarize_run(one.audit, 4, opts);
    sm.requested_elevator = target.elevator_cmd;
    sm.requested_throttle = target.throttle_cmd;
    sm.run_label = string(runLabel);
    scanRows(end+1,1) = sm; %#ok<AGROW>
end

refTable = struct2table(refSummary);
scanTable = struct2table(scanRows);
writetable(refTable, fullfile(outputRoot, "cfg3_reference_summary.csv"));
writetable(scanTable, fullfile(outputRoot, "cfg4_elevator_qdot_scan.csv"));

massTable = local_mass_inertia_table(refTable, scanTable);
writetable(massTable, fullfile(outputRoot, "cfg3_cfg4_mass_inertia.csv"));

zeroCross = local_qdot_zero_crossing(scanTable);
xmlAudit = local_xml_snapshot(projectRoot);
reportPath = fullfile(outputRoot, "longitudinal_audit_report.txt");
local_write_report(reportPath, projectRoot, bankSource, refTable, scanTable, massTable, ...
    zeroCross, xmlAudit, ePred, tPred, opts);

report = struct();
report.output_root = string(outputRoot);
report.report_txt = string(reportPath);
report.cfg3_summary_csv = string(fullfile(outputRoot, "cfg3_reference_summary.csv"));
report.cfg4_scan_csv = string(fullfile(outputRoot, "cfg4_elevator_qdot_scan.csv"));
report.mass_inertia_csv = string(fullfile(outputRoot, "cfg3_cfg4_mass_inertia.csv"));
report.estimated_qdot_zero_external_elevator = zeroCross;
report.cfg3 = refTable;
report.cfg4_scan = scanTable;

fprintf("\n[V55-AUDIT] Complete.\n  %s\n", reportPath);
fprintf("[V55-AUDIT] v32 training memory was read only; no checkpoints/certificates were changed.\n");
end

function one = local_run_one(projectRoot, runRoot, cfgId, targetTrim, bank, opts, runLabel)
if ~isfolder(runRoot), mkdir(runRoot); end
auditCsv = fullfile(runRoot, "jsbsim_runtime_audit.csv");
if isfile(auditCsv), delete(auditCsv); end
oldEnv = getenv("AIRDROPX_JSBSIM_AUDIT_CSV");
envCleanup = onCleanup(@() setenv("AIRDROPX_JSBSIM_AUDIT_CSV", oldEnv)); %#ok<NASGU>
setenv("AIRDROPX_JSBSIM_AUDIT_CSV", char(auditCsv));

pitch0 = local_field(bank(1), "pitch_deg", 4.0);
gamma0 = local_field(bank(1), "initial_flight_path_deg", 0.0);
result = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", projectRoot, "Model", opts.Model, ...
    "OutputRoot", runRoot, "RunId", runLabel, "ConfigId", cfgId, ...
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

% Ensure mdlTerminate has closed the file before reading it.
drawnow;
if ~isfile(auditCsv)
    error("AirdropX:V55Audit:NoRuntimeAudit", ...
        "JSBSim runtime audit CSV was not created. Rebuild the instrumented sfun_airdropx_jsbsim MEX using run_v55_physics_audit_D_temp.ps1. Expected: %s", ...
        auditCsv);
end
A = readtable(auditCsv, "VariableNamingRule", "preserve");
if isempty(A)
    error("AirdropX:V55Audit:EmptyRuntimeAudit", "Runtime audit CSV is empty: %s", auditCsv);
end
one = struct("result", result, "audit", A, "audit_csv", string(auditCsv));
end

function sm = local_summarize_run(A, cfgId, opts)
A = A(round(A.drop_count) == cfgId, :);
if isempty(A)
    error("AirdropX:V55Audit:MissingConfigRows", "Runtime audit contains no cfg%d rows.", cfgId);
end
t0 = A.time_s(1);
tRel = A.time_s - t0;
early = tRel >= opts.EarlyWindowS(1) & tRel <= opts.EarlyWindowS(2);
if nnz(early) < 3
    early = tRel <= min(max(tRel), 0.5);
end
tailStart = max(tRel(1), max(tRel) - opts.TailWindowS);
tail = tRel >= tailStart;
if nnz(tail) < 3, tail = true(height(A),1); end

sm = local_empty_summary();
sm.config_id = cfgId;
sm.cfg_entry_time_s = t0;
sm.duration_in_cfg_s = max(tRel);
sm.requested_elevator = median(A.requested_elevator_delta(tail), "omitnan");
sm.requested_throttle = median(A.throttle_cmd_norm(tail), "omitnan");
sm.physical_elevator_norm = median(A.elevator_pos_norm(tail), "omitnan");
sm.physical_elevator_rad = median(A.elevator_pos_rad(tail), "omitnan");
sm.metadata_mass_kg = median(A.metadata_mass_kg(tail), "omitnan");
sm.jsbsim_mass_kg = median(A.jsbsim_mass_kg(tail), "omitnan");
sm.mass_gap_kg = sm.jsbsim_mass_kg - sm.metadata_mass_kg;
sm.metadata_cg_x_m = median(A.metadata_cg_x_m(tail), "omitnan");
sm.jsbsim_cg_x_m = median(A.jsbsim_cg_x_m(tail), "omitnan");
sm.cg_gap_m = sm.jsbsim_cg_x_m - sm.metadata_cg_x_m;
sm.iyy_slugs_ft2 = median(A.iyy_slugs_ft2(tail), "omitnan");
sm.ixx_slugs_ft2 = median(A.ixx_slugs_ft2(tail), "omitnan");
sm.izz_slugs_ft2 = median(A.izz_slugs_ft2(tail), "omitnan");
sm.ixz_slugs_ft2 = median(A.ixz_slugs_ft2(tail), "omitnan");
sm.fuel_mass_kg = median((A.tank0_lbs(tail) + A.tank1_lbs(tail)) * 0.45359237, "omitnan");
sm.alpha_early_deg = mean(A.alpha_deg(early), "omitnan");
sm.qdot_early_deg_s2 = mean(A.qdot_deg_s2(early), "omitnan");
sm.aero_pitch_moment_early_lbsft = mean(A.aero_pitch_moment_lbsft(early), "omitnan");
sm.total_pitch_moment_early_lbsft = mean(A.total_pitch_moment_lbsft(early), "omitnan");
sm.pitch_alpha_moment_early_lbsft = mean(A.pitch_alpha_lbsft(early), "omitnan");
sm.pitch_elevator_moment_early_lbsft = mean(A.pitch_elevator_lbsft(early), "omitnan");
sm.q_tail_mean_dps = mean(A.q_dps(tail), "omitnan");
sm.q_tail_rms_dps = sqrt(mean(A.q_dps(tail).^2, "omitnan"));
sm.qdot_tail_mean_deg_s2 = mean(A.qdot_deg_s2(tail), "omitnan");
sm.alpha_tail_mean_deg = mean(A.alpha_deg(tail), "omitnan");
sm.airspeed_tail_mean_mps = mean(A.airspeed_mps(tail), "omitnan");
sm.airspeed_tail_rms_error_mps = sqrt(mean((A.airspeed_mps(tail)-opts.SpeedMps).^2, "omitnan"));
sm.vz_tail_mean_mps = mean(A.vz_up_mps(tail), "omitnan");
sm.vz_tail_rms_mps = sqrt(mean(A.vz_up_mps(tail).^2, "omitnan"));
sm.pitch_tail_mean_deg = mean(A.pitch_deg(tail), "omitnan");
sm.pitch_tail_drift_deg_s = local_slope(tRel(tail), A.pitch_deg(tail));
sm.aero_pitch_moment_tail_lbsft = mean(A.aero_pitch_moment_lbsft(tail), "omitnan");
sm.total_pitch_moment_tail_lbsft = mean(A.total_pitch_moment_lbsft(tail), "omitnan");
end

function s = local_empty_summary()
s = struct( ...
    "run_label", "", "config_id", NaN, "cfg_entry_time_s", NaN, "duration_in_cfg_s", NaN, ...
    "requested_elevator", NaN, "requested_throttle", NaN, ...
    "physical_elevator_norm", NaN, "physical_elevator_rad", NaN, ...
    "metadata_mass_kg", NaN, "jsbsim_mass_kg", NaN, "mass_gap_kg", NaN, ...
    "metadata_cg_x_m", NaN, "jsbsim_cg_x_m", NaN, "cg_gap_m", NaN, ...
    "iyy_slugs_ft2", NaN, "ixx_slugs_ft2", NaN, "izz_slugs_ft2", NaN, "ixz_slugs_ft2", NaN, ...
    "fuel_mass_kg", NaN, ...
    "alpha_early_deg", NaN, "qdot_early_deg_s2", NaN, ...
    "aero_pitch_moment_early_lbsft", NaN, "total_pitch_moment_early_lbsft", NaN, ...
    "pitch_alpha_moment_early_lbsft", NaN, "pitch_elevator_moment_early_lbsft", NaN, ...
    "q_tail_mean_dps", NaN, "q_tail_rms_dps", NaN, "qdot_tail_mean_deg_s2", NaN, ...
    "alpha_tail_mean_deg", NaN, "airspeed_tail_mean_mps", NaN, "airspeed_tail_rms_error_mps", NaN, ...
    "vz_tail_mean_mps", NaN, "vz_tail_rms_mps", NaN, ...
    "pitch_tail_mean_deg", NaN, "pitch_tail_drift_deg_s", NaN, ...
    "aero_pitch_moment_tail_lbsft", NaN, "total_pitch_moment_tail_lbsft", NaN);
end

function T = local_mass_inertia_table(ref, scan)
% cfg4 mass/inertia do not depend on elevator; use medians across scan runs.
vars = ["config_id","metadata_mass_kg","jsbsim_mass_kg","mass_gap_kg", ...
    "metadata_cg_x_m","jsbsim_cg_x_m","cg_gap_m", ...
    "ixx_slugs_ft2","iyy_slugs_ft2","izz_slugs_ft2","ixz_slugs_ft2","fuel_mass_kg"];
r3 = ref(1, vars);
r4 = table();
r4.config_id = 4;
for v = vars(2:end)
    r4.(v) = median(scan.(v), "omitnan");
end
T = [r3; r4];
end

function e0 = local_qdot_zero_crossing(T)
e0 = NaN;
if isempty(T), return; end
[e, idx] = sort(double(T.requested_elevator));
q = double(T.qdot_early_deg_s2(idx));
valid = isfinite(e) & isfinite(q);
e = e(valid); q = q(valid);
for i = 1:numel(e)-1
    if q(i) == 0
        e0 = e(i); return;
    end
    if q(i) * q(i+1) < 0
        e0 = e(i) + (0-q(i))*(e(i+1)-e(i))/(q(i+1)-q(i));
        return;
    end
end
if ~isempty(q)
    [~, k] = min(abs(q));
    e0 = e(k);
end
end

function x = local_xml_snapshot(projectRoot)
x = struct("aerorp_x_m", NaN, "empty_cg_x_m", NaN, ...
    "empty_iyy_slugs_ft2", NaN, "fuel_initial_kg", NaN);
xmlPath = fullfile(projectRoot, "aircraft", "MQ9_Reaper", "MQ9_Reaper.xml");
if ~isfile(xmlPath), return; end
txt = fileread(xmlPath);
x.aerorp_x_m = 0.0254 * local_regex_number(txt, ...
    '(?s)<location\s+name="AERORP"[^>]*>.*?<x>\s*([-+0-9.eE]+)\s*</x>');
x.empty_cg_x_m = 0.0254 * local_regex_number(txt, ...
    '(?s)<location\s+name="CG"[^>]*>.*?<x>\s*([-+0-9.eE]+)\s*</x>');
x.empty_iyy_slugs_ft2 = local_regex_number(txt, '<iyy[^>]*>\s*([-+0-9.eE]+)\s*</iyy>');
contents = regexp(txt, '<contents\s+unit="LBS">\s*([-+0-9.eE]+)\s*</contents>', 'tokens');
if ~isempty(contents)
    vals = cellfun(@(c) str2double(c{1}), contents);
    x.fuel_initial_kg = sum(vals, "omitnan") * 0.45359237;
end
end

function v = local_regex_number(txt, expr)
t = regexp(txt, expr, 'tokens', 'once');
if isempty(t), v = NaN; else, v = str2double(t{1}); end
end

function local_write_report(path, projectRoot, bankSource, ref, scan, massT, zeroCross, xml, ePred, tPred, opts)
fid = fopen(path, "w");
if fid < 0, error("Could not write %s", path); end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "AirdropX V55 cfg3/cfg4 longitudinal physics audit\n");
fprintf(fid, "Generated: %s\n", char(datetime("now")));
fprintf(fid, "Project: %s\n", projectRoot);
fprintf(fid, "Preparation bank source (read-only): %s\n\n", bankSource);
fprintf(fid, "Purpose: distinguish mass/inertia/CG model issues from a genuine cfg4@55 longitudinal trim/stability boundary.\n");
fprintf(fid, "This audit does not modify v32 learning memory.\n\n");

fprintf(fid, "STATIC XML SNAPSHOT\n");
fprintf(fid, "  AERORP x                = %.6f m\n", xml.aerorp_x_m);
fprintf(fid, "  empty CG x              = %.6f m\n", xml.empty_cg_x_m);
fprintf(fid, "  empty Iyy               = %.6f slug*ft^2\n", xml.empty_iyy_slugs_ft2);
fprintf(fid, "  initial fuel in XML     = %.3f kg\n\n", xml.fuel_initial_kg);

fprintf(fid, "RUNTIME MASS / CG / INERTIA\n");
for i = 1:height(massT)
    fprintf(fid, "  cfg%d: metadata mass %.3f kg, JSBSim mass %.3f kg, gap %+0.3f kg\n", ...
        massT.config_id(i), massT.metadata_mass_kg(i), massT.jsbsim_mass_kg(i), massT.mass_gap_kg(i));
    fprintf(fid, "        metadata CG %.6f m, JSBSim CG %.6f m, Iyy %.3f slug*ft^2, fuel %.3f kg\n", ...
        massT.metadata_cg_x_m(i), massT.jsbsim_cg_x_m(i), massT.iyy_slugs_ft2(i), massT.fuel_mass_kg(i));
end
if height(massT) >= 2
    fprintf(fid, "  cfg3->cfg4 Iyy change = %+0.3f %%\n", ...
        100*(massT.iyy_slugs_ft2(2)/massT.iyy_slugs_ft2(1)-1));
end
fprintf(fid, "\n");

fprintf(fid, "KNOWN-GOOD CFG3 REFERENCE\n");
fprintf(fid, "  external elevator = %+.6f, physical norm = %+.6f, physical rad = %+.6f\n", ...
    ref.requested_elevator(1), ref.physical_elevator_norm(1), ref.physical_elevator_rad(1));
fprintf(fid, "  early qdot = %+.6f deg/s^2, early aero pitch moment = %+.3f lb*ft\n", ...
    ref.qdot_early_deg_s2(1), ref.aero_pitch_moment_early_lbsft(1));
fprintf(fid, "  tail Va = %.3f m/s, tail vz = %+.3f m/s, tail q = %+.3f deg/s\n\n", ...
    ref.airspeed_tail_mean_mps(1), ref.vz_tail_mean_mps(1), ref.q_tail_mean_dps(1));

fprintf(fid, "CFG4 ELEVATOR / QDOT SCAN\n");
fprintf(fid, "  continuation center (not assumed correct): external elevator %+.6f, throttle %.6f\n", ePred, tPred);
fprintf(fid, "  estimated zero/nearest early-qdot external elevator = %+.6f\n", zeroCross);
fprintf(fid, "  columns: extElev, physicalElevNorm, alphaEarlyDeg, qdotEarlyDegS2, aeroMEarlyLbFt, tailVa, tailVz, tailQ\n");
for i = 1:height(scan)
    fprintf(fid, "  %+.6f  %+.6f  %+.4f  %+.5f  %+.2f  %.3f  %+.3f  %+.3f\n", ...
        scan.requested_elevator(i), scan.physical_elevator_norm(i), scan.alpha_early_deg(i), ...
        scan.qdot_early_deg_s2(i), scan.aero_pitch_moment_early_lbsft(i), ...
        scan.airspeed_tail_mean_mps(i), scan.vz_tail_mean_mps(i), scan.q_tail_mean_dps(i));
end
fprintf(fid, "\nINTERPRETATION GUIDE\n");
fprintf(fid, "  1) If cfg3->cfg4 Iyy changes only modestly, an inertia-collapse bug is ruled out.\n");
fprintf(fid, "  2) If JSBSim mass differs materially from metadata mass, fix the mass signal/controller context before final certification.\n");
fprintf(fid, "  3) If cfg4 early qdot changes sign inside the elevator scan, a pitch-moment balance exists in the tested control range; do not call 55 m/s physically untrimmable yet.\n");
fprintf(fid, "  4) If qdot never changes sign and the total/aero moments keep one sign across a broad physical-elevator range, control authority / moment balance is the stronger boundary hypothesis.\n");
fprintf(fid, "  5) Compare AERORP and runtime CG. A cfg3->cfg4 CG movement changes the aerodynamic force moment arm, so linear cfg continuation is not guaranteed.\n");
fprintf(fid, "\nAudit settings: V=%.3f m/s, H=%.1f m, prep drop start %.2f s, interval %.2f s, stop %.2f s.\n", ...
    opts.SpeedMps, opts.AltitudeM, opts.PrepDropStartS, opts.PrepDropIntervalS, opts.StopTimeS);
end

function slope = local_slope(t, y)
t = double(t(:)); y = double(y(:));
mask = isfinite(t) & isfinite(y);
t = t(mask); y = y(mask);
if numel(t) < 2 || max(t)-min(t) <= eps, slope = NaN; return; end
p = polyfit(t - t(1), y, 1);
slope = p(1);
end

function v = local_field(s, name, fallback)
v = double(fallback);
try
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
        v = double(s.(name));
    end
catch
end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.OutputRoot = "";
opts.Model = "airdropx_mpc_id";
opts.SpeedMps = 55.0;
opts.AltitudeM = 200.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.PrepDropStartS = 1.0;
opts.PrepDropIntervalS = 2.0;
opts.StopTimeS = 14.0;
opts.EarlyWindowS = [0.05 0.35];
opts.TailWindowS = 2.0;
opts.ElevatorLimits = [-0.75 0.45];
opts.ThrottleLimits = [0.35 0.88];
opts.ElevatorBelowSpan = 0.35;
opts.ElevatorAboveSpan = 0.25;
opts.ElevatorSamples = 9;
if mod(numel(varargin),2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i+1};
end
end
