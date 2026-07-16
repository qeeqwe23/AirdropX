function result = airdropx_mpc_make_excitation_data(varargin)
%AIRDROPX_MPC_MAKE_EXCITATION_DATA Create standalone MPC identification data.
%
% This runs the existing PD/JSBSim workflow with varied references and initial
% conditions. It does not connect MPC to Simulink and does not alter the
% existing controller files.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
mpcDir = string(fileparts(thisFile));
matlabDir = string(fileparts(mpcDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(mpcDir));

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_excitation_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

cases = local_cases(opts);
allT = table();
caseSummary = table();
updateModel = logical(opts.UpdateModel);

for i = 1:numel(cases)
    c = cases(i);
    runName = sprintf("%02d_%s", i, c.name);
    caseDir = fullfile(outputRoot, runName);
    fprintf("AirdropX MPC excitation case %d/%d: %s\n", i, numel(cases), c.name);

    simResult = airdropx_run_and_export( ...
        "ProjectRoot", projectRoot, ...
        "RunName", runName, ...
        "OutputDir", caseDir, ...
        "UpdateModel", updateModel, ...
        "Overrides", c.overrides);

    T = simResult.timeseries;
    T.case_id = repmat(string(runName), height(T), 1);
    T.target_altitude_m = repmat(c.target_altitude_m, height(T), 1);
    T.target_airspeed_mps = repmat(c.target_airspeed_mps, height(T), 1);
    T.target_pitch_deg = repmat(c.target_pitch_deg, height(T), 1);
    T.case_note = repmat(string(c.note), height(T), 1);

    allT = [allT; T]; %#ok<AGROW>

    row = table( ...
        string(runName), string(c.note), ...
        c.target_altitude_m, c.target_airspeed_mps, c.target_pitch_deg, ...
        simResult.report.drop_count_final, ...
        simResult.report.h_err_rms, ...
        simResult.report.min_altitude, ...
        simResult.report.max_altitude, ...
        'VariableNames', { ...
            'case_id', 'note', ...
            'target_altitude_m', 'target_airspeed_mps', 'target_pitch_deg', ...
            'drop_count_final', 'h_err_rms_m', 'min_altitude_m', 'max_altitude_m'});
    caseSummary = [caseSummary; row]; %#ok<AGROW>

    updateModel = false;
end

combinedCsv = fullfile(outputRoot, "excitation_timeseries.csv");
caseCsv = fullfile(outputRoot, "case_summary.csv");
metricCsv = fullfile(outputRoot, "mpc_summary.csv");
modelMat = fullfile(outputRoot, "identified_model.mat");

writetable(allT, combinedCsv);
writetable(caseSummary, caseCsv);
metrics = airdropx_mpc_evaluate_csv(combinedCsv, "OutputFile", metricCsv);
identified = airdropx_mpc_identify_from_csv(combinedCsv, ...
    "OutputMat", modelMat, ...
    "UseLastTrim", false);

result = struct();
result.output_root = outputRoot;
result.combined_csv = string(combinedCsv);
result.case_summary_csv = string(caseCsv);
result.metric_csv = string(metricCsv);
result.identified_model_mat = string(modelMat);
result.case_summary = caseSummary;
result.metrics = metrics;
result.identification = identified;

fprintf("AirdropX MPC excitation data written:\n");
fprintf("  %s\n", combinedCsv);
fprintf("  %s\n", caseCsv);
fprintf("  %s\n", modelMat);
end

function cases = local_cases(opts)
base = struct();
base.stop_time_s = opts.StopTimeS;
base.fixed_drop_start_s = opts.DropStartS;
base.fixed_drop_interval_s = 0.25;

caseDefs = {
    "base_nominal",      20.0, 45.0, 4.0,  0.00,  0.00, "nominal 20 m reference"
    "high_ref",          21.0, 45.0, 4.5,  0.00,  0.00, "higher altitude and pitch reference"
    "low_ref",           19.0, 45.0, 3.5,  0.00,  0.00, "lower altitude and pitch reference"
    "slow_ref",          20.0, 43.5, 4.2,  0.00, -0.02, "slower speed demand"
    "fast_ref",          20.0, 47.0, 3.8,  0.00,  0.03, "faster speed demand"
    "pitch_rich",        20.5, 44.5, 5.2,  0.02,  0.00, "pitch-rich case for elevator response"
    };

count = min(opts.NumCases, size(caseDefs, 1));
cases = repmat(struct( ...
    "name", "", ...
    "note", "", ...
    "target_altitude_m", 0.0, ...
    "target_airspeed_mps", 0.0, ...
    "target_pitch_deg", 0.0, ...
    "overrides", struct()), count, 1);

for i = 1:count
    name = string(caseDefs{i, 1});
    hRef = double(caseDefs{i, 2});
    vRef = double(caseDefs{i, 3});
    pitchRef = double(caseDefs{i, 4});
    elevOffset = double(caseDefs{i, 5});
    throttleOffset = double(caseDefs{i, 6});
    note = string(caseDefs{i, 7});

    overrides = base;
    overrides.target_altitude_m = hRef;
    overrides.v_ref_mps = vRef;
    overrides.pitch_ref_deg = pitchRef;
    overrides.initial_theta_deg = pitchRef;
    overrides.initial_elevator_delta = 0.10 + elevOffset;
    overrides.initial_throttle_cmd = 0.58 + throttleOffset;
    overrides.throttle_fixed = 0.58 + throttleOffset;

    cases(i).name = char(name);
    cases(i).note = char(note);
    cases(i).target_altitude_m = hRef;
    cases(i).target_airspeed_mps = vRef;
    cases(i).target_pitch_deg = pitchRef;
    cases(i).overrides = overrides;
end
end

function opts = local_options(varargin)
opts.OutputRoot = "";
opts.NumCases = 6;
opts.StopTimeS = 22.0;
opts.DropStartS = 8.0;
opts.UpdateModel = false;

if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end

for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i + 1};
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = value;
end
end
