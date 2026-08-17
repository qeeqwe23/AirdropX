function result = airdropx_auto_smoke_single_config(varargin)
%AIRDROPX_AUTO_SMOKE_SINGLE_CONFIG Short-horizon real-JSBSim MPC smoke tests.
%
% This freezes the identified v11 plants, builds four fixed short-horizon MPC
% banks, and tests cfg0..cfg2 as direct fixed configurations. It intentionally
% does not regenerate ID data, rerun n4sid, or launch bayesopt.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

outRoot = string(opts.OutputRoot);
if strlength(outRoot) == 0
    outRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_smoke_single_config_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outRoot), mkdir(outRoot); end

S = load(opts.IdentifiedMat, "result");
identified = S.result;
trimBank = identified.trim_bank;
physicalElevatorNominals = local_physical_elevator_nominals(opts, paths, trimBank);
writetable(table((0:numel(physicalElevatorNominals)-1).', physicalElevatorNominals(:), 'VariableNames', {'config_id','physical_elevator_nominal'}), fullfile(outRoot, 'physical_elevator_nominals.csv'));

candidates = local_candidates(opts);
caseDefs = local_cases(opts, trimBank);
rows = table();

for i = 1:numel(candidates)
    c = candidates(i);
    bankDir = fullfile(outRoot, char(c.name));
    if ~isfolder(bankDir), mkdir(bankDir); end
    bankMat = fullfile(bankDir, "airdropx_mpc_bank.mat");
    fprintf("\n[SMOKE] Building %s: Np=%d Nc=%d\n", c.name, c.Np, c.Nc);
    airdropx_auto_build_mpc_bank( ...
        "Identified", identified, ...
        "OutputMat", bankMat, ...
        "PredictionHorizon", c.Np, ...
        "ControlHorizon", c.Nc, ...
        "OutputWeights", opts.OutputWeights, ...
        "MVWeights", opts.MVWeights, ...
        "MVRateWeights", opts.MVRateWeights, ...
        "PitchMinDeg", opts.PitchMinDeg, ...
        "PitchMaxDeg", opts.PitchMaxDeg, ...
        "PhysicalElevatorNominals", physicalElevatorNominals);

    for j = 1:numel(caseDefs)
        cd = caseDefs(j);
        caseRoot = fullfile(bankDir, char(cd.name));
        fprintf("[SMOKE] %s / %s\n", c.name, cd.name);
        try
            hiddenTrim = local_calibrate_hidden_trim(opts, paths, cd, caseRoot);
            physicalElevatorNominal = physicalElevatorNominals(cd.config_id + 1);
            initialElevatorDelta = physicalElevatorNominal - hiddenTrim;
            local_assign_bridge_defaults();
            simResult = airdropx_auto_run_closed_loop( ...
                "ProjectRoot", paths.projectRoot, ...
                "MpcBankMat", bankMat, ...
                "OutputRoot", caseRoot, ...
                "CaseId", cd.name, ...
                "StopTimeS", opts.StopTimeS, ...
                "FixedConfigId", cd.config_id, ...
                "InitialDropCount", cd.config_id, ...
                "FixedDropTotal", 0, ...
                "FixedDropStartS", opts.StopTimeS + 100, ...
                "InitialAltitudeM", cd.initial_h_m, ...
                "InitialAirspeedMps", cd.initial_V_mps, ...
                "InitialPitchDeg", cd.initial_pitch_deg, ...
                "InitialFlightPathDeg", cd.initial_gamma_deg, ...
                "InitialElevatorDelta", initialElevatorDelta, ...
                "InitialThrottleCmd", cd.initial_throttle_cmd, ...
                "TargetAltitudeM", opts.TargetAltitudeM, ...
                "TargetAirspeedMps", opts.TargetAirspeedMps, ...
                "TargetPitchDeg", cd.trim_pitch_deg, ...
                "HiddenElevatorTrim", hiddenTrim);
            score = airdropx_auto_score_closed_loop(simResult.timeseries, ...
                "StartTimeS", opts.ScoreStartTimeS, ...
                "TargetAltitudeM", opts.TargetAltitudeM, ...
                "TargetAirspeedMps", opts.TargetAirspeedMps);
            hard = local_hard_fail(simResult.timeseries, opts);
            smokeStatus = local_status(score.metrics, hard, opts);
            row = local_row(c, cd, smokeStatus, "", simResult.timeseries_csv, score.metrics, hard);
            local_plot_timeseries(simResult.timeseries, fullfile(caseRoot, "closed_loop_curves.png"), cd.name);
        catch ME
            warning("AirdropX:AutoMPC:SmokeCaseFailed", "%s / %s failed: %s", c.name, cd.name, ME.message);
            row = local_failed_row(c, cd, ME);
        end
        rows = [rows; row]; %#ok<AGROW>
        writetable(rows, fullfile(outRoot, "single_config_smoke_results.csv"));
    end
end

local_plot_summary(rows, fullfile(outRoot, "single_config_smoke_summary.png"));
result = struct();
result.output_root = outRoot;
result.rows = rows;
save(fullfile(outRoot, "single_config_smoke_result.mat"), "result", "opts", "candidates", "caseDefs");
fprintf("\n[SMOKE] Results written to:\n  %s\n", outRoot);
end

function candidates = local_candidates(opts)
raw = [8 3; 10 3; 12 3; 10 4];
names = ["A_Np8_Nc3"; "B_Np10_Nc3"; "C_Np12_Nc3"; "D_Np10_Nc4"];
maxN = min(size(raw,1), double(opts.MaxCandidateBanks));
candidates = repmat(struct("name", "", "Np", 0, "Nc", 0), maxN, 1);
for i = 1:maxN
    candidates(i).name = names(i);
    candidates(i).Np = raw(i,1);
    candidates(i).Nc = raw(i,2);
end
end

function cases = local_cases(opts, trimBank)
cfgs = double(opts.ConfigIds(:)).';
cases = repmat(local_empty_case(), 2 * numel(cfgs), 1);
k = 0;
for cfg = cfgs
    trim = trimBank(cfg + 1);
    k = k + 1;
    cases(k) = local_case(sprintf("cfg%d_nominal", cfg), cfg, opts.TargetAltitudeM, opts.TargetAirspeedMps, ...
        trim.pitch_deg, local_trim_field(trim, "gamma_deg", 0.0), trim);
    k = k + 1;
    cases(k) = local_case(sprintf("cfg%d_perturbed", cfg), cfg, opts.TargetAltitudeM + opts.PerturbAltitudeM, ...
        opts.TargetAirspeedMps + opts.PerturbAirspeedMps, trim.pitch_deg, opts.PerturbFlightPathDeg, trim);
end
cases = cases(1:k);
end

function c = local_empty_case()
c = struct("name", "", "config_id", 0, "initial_h_m", 20.0, "initial_V_mps", 50.0, ...
    "initial_pitch_deg", 0.0, "initial_gamma_deg", 0.0, "trim_pitch_deg", 0.0, ...
    "initial_elevator_cmd", NaN, "initial_throttle_cmd", NaN);
end

function c = local_case(name, cfg, h, V, pitch, gamma, trim)
c = local_empty_case();
c.name = string(name);
c.config_id = double(cfg);
c.initial_h_m = double(h);
c.initial_V_mps = double(V);
c.initial_pitch_deg = double(pitch);
c.initial_gamma_deg = double(gamma);
c.trim_pitch_deg = double(pitch);
c.initial_elevator_cmd = double(trim.elevator_cmd);
c.initial_throttle_cmd = double(trim.throttle_cmd);
end

function row = local_row(c, cd, status, message, csvPath, metrics, hard)
row = metrics;
row = addvars(row, string(c.name), double(c.Np), double(c.Nc), string(cd.name), double(cd.config_id), ...
    string(status), string(message), string(csvPath), double(hard.min_altitude_m), ...
    double(hard.elevator_saturation_s), double(hard.throttle_saturation_s), ...
    'Before', 1, 'NewVariableNames', {'candidate','Np','Nc','case_name','config_id','status','message','timeseries_csv', ...
    'hard_min_altitude_m','hard_elevator_saturation_s','hard_throttle_saturation_s'});
end

function row = local_failed_row(c, cd, ME)
row = table(string(c.name), double(c.Np), double(c.Nc), string(cd.name), double(cd.config_id), ...
    "failed", string(ME.message), "", NaN, NaN, NaN, ...
    'VariableNames', {'candidate','Np','Nc','case_name','config_id','status','message','timeseries_csv', ...
    'hard_min_altitude_m','hard_elevator_saturation_s','hard_throttle_saturation_s'});
end

function status = local_status(metrics, hard, opts)
if hard.failed
    status = "hard_fail";
    return;
end
m = metrics(1,:);
smokeOk = m.steady_h_rms_m <= opts.SmokeMaxSteadyAltitudeRmsM && ...
    m.steady_h_max_abs_m <= opts.SmokeMaxSteadyAltitudeMaxAbsM && ...
    m.steady_airspeed_rms_mps <= opts.SmokeMaxSteadyAirspeedRmsMps && ...
    m.steady_vz_rms_mps <= opts.SmokeMaxSteadyVzRmsMps && ...
    m.steady_q_rms_dps <= opts.SmokeMaxSteadyQRmsDps;
formalOk = m.steady_h_rms_m <= opts.FormalMaxSteadyAltitudeRmsM && ...
    m.steady_h_max_abs_m <= opts.FormalMaxSteadyAltitudeMaxAbsM && ...
    m.steady_h_drift_m <= opts.FormalMaxSteadyAltitudeDriftM && ...
    m.steady_airspeed_rms_mps <= opts.FormalMaxSteadyAirspeedRmsMps && ...
    m.steady_vz_rms_mps <= opts.FormalMaxSteadyVzRmsMps && ...
    m.steady_q_rms_dps <= opts.FormalMaxSteadyQRmsDps && ...
    m.steady_pitch_std_deg <= opts.FormalMaxPitchStdDeg && ...
    m.steady_pitch_drift_degps <= opts.FormalMaxPitchDriftDegps;
if formalOk
    status = "formal_pass";
elseif smokeOk
    status = "smoke_pass";
else
    status = "needs_tuning";
end
end

function physical = local_physical_elevator_nominals(opts, paths, trimBank)
physical = arrayfun(@(s) double(s.elevator_cmd), trimBank(:));
dataRoot = string(opts.DataRoot);
if strlength(dataRoot) == 0
    dataRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_id_v11_clean_r1", "data"));
end
for cfg = 0:numel(trimBank)-1
    files = dir(fullfile(dataRoot, sprintf("cfg%d", cfg), "cfg*_run*", "auto_id_timeseries.csv"));
    values = [];
    for i = 1:numel(files)
        T = readtable(fullfile(files(i).folder, files(i).name));
        vars = string(T.Properties.VariableNames);
        if ~ismember("elevator_cmd_norm", vars) || ~ismember("time_s", vars)
            continue;
        end
        t = double(T.time_s);
        t0 = local_excitation_start(T);
        if isfinite(t0)
            mask = t >= t0 - 4.0 & t < t0 - 1e-9;
        else
            mask = t <= min(t) + 1.0;
        end
        if ismember("drop_count", vars)
            mask = mask & round(double(T.drop_count)) == cfg;
        end
        x = double(T.elevator_cmd_norm(mask));
        x = x(isfinite(x));
        if ~isempty(x)
            values(end+1,1) = median(x, "omitnan"); %#ok<AGROW>
        end
    end
    if ~isempty(values)
        physical(cfg+1) = median(values, "omitnan");
    end
end
end

function t0 = local_excitation_start(T)
t0 = NaN;
vars = string(T.Properties.VariableNames);
if ismember("elevator_excitation", vars) || ismember("throttle_excitation", vars)
    e = zeros(height(T),1); th = zeros(height(T),1);
    if ismember("elevator_excitation", vars), e = abs(double(T.elevator_excitation)); end
    if ismember("throttle_excitation", vars), th = abs(double(T.throttle_excitation)); end
    idx = find((e > 1e-8 | th > 1e-8) & isfinite(double(T.time_s)), 1, "first");
    if ~isempty(idx), t0 = double(T.time_s(idx)); end
end
end

function hiddenTrim = local_calibrate_hidden_trim(opts, paths, cd, caseRoot)
if ~logical(opts.CalibrateHiddenTrim)
    hiddenTrim = NaN;
    return;
end
calRoot = fullfile(caseRoot, "hidden_trim_calibration");
trim = struct();
trim.config_id = double(cd.config_id);
trim.altitude_m = double(opts.TargetAltitudeM);
trim.airspeed_mps = double(opts.TargetAirspeedMps);
trim.pitch_deg = double(cd.trim_pitch_deg);
trim.vz_up_mps = 0.0;
trim.q_dps = 0.0;
trim.elevator_cmd = 0.0;
trim.throttle_cmd = double(cd.initial_throttle_cmd);
cal = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputRoot", calRoot, ...
    "RunId", string(cd.name) + "_hidden_trim_cal", ...
    "ConfigId", double(cd.config_id), ...
    "Trim", trim, ...
    "StopTimeS", double(opts.CalibrationDurationS), ...
    "RecordStartS", 0, ...
    "ExportStartS", 0, ...
    "ExcitationStartS", 100, ...
    "KeepFixedConfigurationOnly", true, ...
    "DirectIdMode", true, ...
    "InitialAltitudeM", double(cd.initial_h_m), ...
    "InitialAirspeedMps", double(cd.initial_V_mps), ...
    "InitialPitchDeg", double(cd.initial_pitch_deg), ...
    "InitialFlightPathDeg", double(cd.initial_gamma_deg), ...
    "TargetAltitudeM", double(opts.TargetAltitudeM), ...
    "TargetAirspeedMps", double(opts.TargetAirspeedMps));
T = cal.timeseries;
mask = double(T.time_s) <= min(double(opts.CalibrationDurationS), 0.5);
if ~ismember("elevator_cmd_norm", string(T.Properties.VariableNames))
    error("AirdropX:AutoMPC:NoPhysicalElevatorLog", "Calibration run did not log elevator_cmd_norm.");
end
hiddenTrim = median(double(T.elevator_cmd_norm(mask)) - double(T.requested_elevator_cmd(mask)), "omitnan");
writetable(table(double(cd.config_id), string(cd.name), hiddenTrim, ...
    'VariableNames', {'config_id','case_name','hidden_elevator_trim'}), fullfile(calRoot, "hidden_trim_summary.csv"));
end

function local_assign_bridge_defaults()
assignin("base", "airdropx_auto_elevator_sign", 1.0);
assignin("base", "airdropx_auto_elevator_safety_gain", 0.0);
assignin("base", "airdropx_auto_elevator_sink_gain", 0.0);
assignin("base", "airdropx_auto_elevator_safety_max", 0.0);
assignin("base", "airdropx_auto_throttle_safety_gain", 0.0);
assignin("base", "airdropx_auto_throttle_sink_gain", 0.0);
assignin("base", "airdropx_auto_throttle_safety_max", 0.0);
assignin("base", "airdropx_auto_pitch_kp", 0.0);
assignin("base", "airdropx_auto_pitch_kq", 0.0);
assignin("base", "airdropx_auto_pitch_damp_max", 0.0);
assignin("base", "airdropx_auto_throttle_alt_high_gain", 0.0);
assignin("base", "airdropx_auto_throttle_climb_gain", 0.0);
end
function hard = local_hard_fail(T, opts)
time = double(T.time_s(:));
h = local_col(T, ["altitude_m"], NaN);
e = local_col(T, ["mpc_elevator_to_plant", "u_out", "elevator_cmd", "elevator_delta", "elevator_cmd_norm"], NaN);
th = local_col(T, ["mpc_throttle_to_plant", "throttle_cmd", "throttle_norm"], NaN);
hard = struct("failed", false, "min_altitude_m", min(h, [], "omitnan"), ...
    "elevator_saturation_s", local_sat_duration(time, e, -0.84, 0.84, 0.005), ...
    "throttle_saturation_s", local_sat_duration(time, th, 0.01, 0.99, 0.005));
if ~isfinite(hard.min_altitude_m) || hard.min_altitude_m < opts.HardFloorAltitudeM || ...
        hard.elevator_saturation_s >= opts.MaxElevatorSaturationDurationS || ...
        hard.throttle_saturation_s >= opts.MaxThrottleSaturationDurationS
    hard.failed = true;
end
end

function duration = local_sat_duration(time, u, lo, hi, tol)
mask = isfinite(time) & isfinite(u) & (u <= lo + tol | u >= hi - tol);
if ~any(mask), duration = 0.0; return; end
dt = median(diff(unique(time(isfinite(time)))), "omitnan");
if ~isfinite(dt) || dt <= 0, dt = 0.1; end
edges = diff([false; mask(:); false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
duration = max(stops - starts + 1) * dt;
end

function x = local_col(T, names, fallback)
for i = 1:numel(names)
    if ismember(names(i), string(T.Properties.VariableNames))
        x = double(T.(char(names(i)))(:));
        return;
    end
end
x = fallback * ones(height(T), 1);
end

function local_plot_timeseries(T, outputFile, caseName)
fig = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
tl = tiledlayout(7,1,'Padding','compact','TileSpacing','compact');
sigs = ["altitude_m", "airspeed_mps", "pitch_deg", "vz_up_mps", "q_dps", "u_out", "throttle_cmd"];
ylabels = ["h (m)", "Va (m/s)", "pitch (deg)", "vz (m/s)", "q (deg/s)", "elev", "thr"];
for i = 1:numel(sigs)
    nexttile; hold on; grid on;
    y = local_col(T, sigs(i), NaN);
    plot(T.time_s, y, 'LineWidth', 1.0);
    ylabel(ylabels(i));
    if i == 1
        title(sprintf('%s closed-loop smoke', caseName), 'Interpreter', 'none');
        if ismember("target_altitude_m", string(T.Properties.VariableNames))
            plot(T.time_s, T.target_altitude_m, '--');
        end
    elseif sigs(i) == "airspeed_mps" && ismember("target_airspeed_mps", string(T.Properties.VariableNames))
        plot(T.time_s, T.target_airspeed_mps, '--');
    end
    if i == numel(sigs), xlabel('time (s)'); end
end
exportgraphics(fig, outputFile, 'Resolution', 150);
close(fig);
end

function local_plot_summary(rows, outputFile)
if isempty(rows) || ~ismember("steady_h_rms_m", string(rows.Properties.VariableNames))
    return;
end
fig = figure('Visible','off','Color','w','Position',[100 100 1200 700]);
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
labels = categorical(strcat(string(rows.candidate), " / ", string(rows.case_name)));
nexttile; bar(labels, rows.steady_h_rms_m); ylabel('steady h RMS (m)'); grid on; title('Single-config smoke summary');
nexttile; bar(labels, rows.steady_airspeed_rms_mps); ylabel('steady Va RMS (m/s)'); grid on;
exportgraphics(fig, outputFile, 'Resolution', 150);
close(fig);
end

function value = local_trim_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function paths = local_paths(projectRoot)
projectRoot = string(projectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath"));
    matlabDir = fileparts(thisDir);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end
paths = struct("projectRoot", char(projectRoot), "matlabDir", char(matlabDir), ...
    "mpcDir", char(fullfile(matlabDir, "mpc")), "autoDir", char(fullfile(matlabDir, "mpc_auto")), ...
    "sfuncDir", char(fullfile(matlabDir, "sfunc_jsbsim")));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.IdentifiedMat = "matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.DataRoot = "matlab/results/mpc_auto_id_v11_clean_r1/data";
opts.OutputRoot = "matlab/results/mpc_auto_smoke_single_config_v11_r1";
opts.ConfigIds = [0;1;2];
opts.MaxCandidateBanks = 4;
opts.StopTimeS = 30.0;
opts.ScoreStartTimeS = 10.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.PerturbAltitudeM = 2.0;
opts.PerturbAirspeedMps = -2.0;
opts.PerturbFlightPathDeg = -0.5;
opts.CalibrateHiddenTrim = true;
opts.CalibrationDurationS = 1.0;
opts.OutputWeights = [12.0 8.0 0.05 5.0 5.0];
opts.MVWeights = [0.08 0.08];
opts.MVRateWeights = [1.5 0.8];
opts.PitchMinDeg = -10.0;
opts.PitchMaxDeg = 20.0;
opts.HardFloorAltitudeM = 5.0;
opts.MaxElevatorSaturationDurationS = 1.0;
opts.MaxThrottleSaturationDurationS = 2.0;
opts.SmokeMaxSteadyAltitudeRmsM = 2.0;
opts.SmokeMaxSteadyAltitudeMaxAbsM = 4.0;
opts.SmokeMaxSteadyAirspeedRmsMps = 2.0;
opts.SmokeMaxSteadyVzRmsMps = 1.0;
opts.SmokeMaxSteadyQRmsDps = 1.5;
opts.FormalMaxSteadyAltitudeRmsM = 1.0;
opts.FormalMaxSteadyAltitudeMaxAbsM = 2.0;
opts.FormalMaxSteadyAltitudeDriftM = 1.0;
opts.FormalMaxSteadyAirspeedRmsMps = 1.0;
opts.FormalMaxSteadyVzRmsMps = 0.7;
opts.FormalMaxSteadyQRmsDps = 1.0;
opts.FormalMaxPitchStdDeg = 0.75;
opts.FormalMaxPitchDriftDegps = 0.12;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end