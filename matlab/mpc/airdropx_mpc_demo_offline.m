function result = airdropx_mpc_demo_offline(csvPath, varargin)
%AIRDROPX_MPC_DEMO_OFFLINE Analyze CSV, identify model, replay MPC commands.
%
% This does not run or modify Simulink.

opts = local_options(varargin{:});

if nargin < 1 || strlength(string(csvPath)) == 0
    csvPath = fullfile("matlab", "results", "codex_stability_check", "timeseries.csv");
end

csvPath = string(csvPath);
runDir = string(fileparts(csvPath));
outDir = fullfile(runDir, "mpc");
if ~isfolder(outDir)
    mkdir(outDir);
end

summaryFile = fullfile(outDir, "summary.csv");
modelFile = fullfile(outDir, "identified_model.mat");
replayFile = fullfile(outDir, "replay_commands.csv");

summary = airdropx_mpc_evaluate_csv(csvPath, "OutputFile", summaryFile);
id = airdropx_mpc_identify_from_csv(csvPath, "OutputMat", modelFile);

cfg = id.cfg;
useIdentified = opts.UseIdentifiedForReplay && ...
    id.diagnostics.regressor_rank >= id.diagnostics.regressor_columns && ...
    id.diagnostics.regressor_condition <= opts.MaxIdentificationCondition;
if useIdentified
    cfg.model = id.model;
else
    cfg.model = airdropx_mpc_nominal_model(cfg);
end
cfg.prediction_horizon = opts.PredictionHorizon;
cfg.control_horizon = opts.ControlHorizon;

T = readtable(csvPath);
X = airdropx_mpc_state_from_table(T, cfg);
stride = max(1, round(cfg.dt_s / median(diff(T.time_s))));
idx = 1:stride:height(T);

ctrlState = [];
rows = table();
for k = 1:numel(idx)
    ii = idx(k);
    [u, ctrlState, diag] = airdropx_mpc_controller(X(ii, :).', ctrlState, cfg);
    row = table( ...
        T.time_s(ii), ...
        T.altitude_m(ii), ...
        T.airspeed_mps(ii), ...
        T.pitch_deg(ii), ...
        T.vz_up_mps(ii), ...
        T.elevator_cmd_norm(ii), ...
        T.throttle_norm(ii), ...
        u(1), ...
        u(2), ...
        diag.hessian_condition, ...
        'VariableNames', { ...
            'time_s', 'measured_altitude_m', 'measured_airspeed_mps', ...
            'measured_pitch_deg', 'measured_vz_up_mps', ...
            'logged_elevator_delta', 'logged_throttle_cmd', ...
            'mpc_elevator_delta', 'mpc_throttle_cmd', ...
            'mpc_hessian_condition'});
    rows = [rows; row]; %#ok<AGROW>
end

writetable(rows, replayFile);

result = struct();
result.summary = summary;
result.identification = id;
result.cfg = cfg;
result.used_identified_model_for_replay = useIdentified;
result.output_dir = string(outDir);
result.summary_csv = string(summaryFile);
result.identified_model_mat = string(modelFile);
result.replay_commands_csv = string(replayFile);
result.replay = rows;
end

function opts = local_options(varargin)
opts.PredictionHorizon = 25;
opts.ControlHorizon = 8;
opts.UseIdentifiedForReplay = false;
opts.MaxIdentificationCondition = 1.0e10;

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
