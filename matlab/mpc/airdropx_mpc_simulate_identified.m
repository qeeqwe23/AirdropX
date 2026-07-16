function result = airdropx_mpc_simulate_identified(modelMat, varargin)
%AIRDROPX_MPC_SIMULATE_IDENTIFIED Closed-loop MPC test on an identified model.
%
% This is an offline model simulation. It does not run Simulink.

opts = local_options(varargin{:});

loaded = load(modelMat);
if ~isfield(loaded, "model") || ~isfield(loaded, "cfg")
    error("MAT file must contain model and cfg variables: %s", modelMat);
end

cfg = loaded.cfg;
cfg.model = loaded.model;
cfg.prediction_horizon = opts.PredictionHorizon;
cfg.control_horizon = opts.ControlHorizon;

if strlength(string(opts.InitialCsv)) > 0
    T0 = readtable(opts.InitialCsv);
    X0 = airdropx_mpc_state_from_table(T0(1, :), cfg);
    x = X0(1, :).';
else
    x = double(opts.InitialState(:));
end

if isempty(x)
    x = [-2.5; 0.0; 1.0; 0.0; 0.0];
end

A = double(cfg.model.A);
B = double(cfg.model.B);
uTrim = double(cfg.model.u_trim(:));
if isfield(cfg.model, "c") && ~isempty(cfg.model.c)
    c = double(cfg.model.c(:));
else
    c = zeros(size(A, 1), 1);
end

steps = max(1, round(double(opts.StopTimeS) / cfg.dt_s));
ctrlState = [];
rows = table();
for k = 1:steps + 1
    t = (k - 1) * cfg.dt_s;
    [u, ctrlState, diag] = airdropx_mpc_controller(x, ctrlState, cfg);
    row = table(t, 'VariableNames', {'time_s'});
    for iState = 1:numel(cfg.state_names)
        row.(matlab.lang.makeValidName(cfg.state_names(iState))) = x(iState);
    end
    row.elevator_delta = u(1);
    row.throttle_cmd = u(2);
    row.hessian_condition = diag.hessian_condition;
    rows = [rows; row]; %#ok<AGROW>
    x = A * x + B * (u - uTrim) + c;
end

tailStart = max(rows.time_s) - opts.MetricWindowS;
tail = rows(rows.time_s >= tailStart, :);
metrics = struct();
metrics.h_err_rms_m = rms(tail.h_err_m);
metrics.v_err_rms_mps = rms(tail.v_err_mps);
metrics.pitch_err_rms_deg = rms(tail.pitch_err_deg);
metrics.q_rms_dps = rms(tail.q_dps);
metrics.final_h_err_m = rows.h_err_m(end);
metrics.final_v_err_mps = rows.v_err_mps(end);
metrics.final_pitch_err_deg = rows.pitch_err_deg(end);
metrics.elevator_sat_count = sum(rows.elevator_delta <= cfg.constraints.u_min(1) + 1e-9 | ...
    rows.elevator_delta >= cfg.constraints.u_max(1) - 1e-9);
metrics.throttle_sat_count = sum(rows.throttle_cmd <= cfg.constraints.u_min(2) + 1e-9 | ...
    rows.throttle_cmd >= cfg.constraints.u_max(2) - 1e-9);
metrics.total_steps = height(rows);

if strlength(string(opts.OutputCsv)) > 0
    outDir = fileparts(opts.OutputCsv);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    writetable(rows, opts.OutputCsv);
end

result = struct();
result.cfg = cfg;
result.trajectory = rows;
result.metrics = metrics;
result.output_csv = string(opts.OutputCsv);
end

function opts = local_options(varargin)
opts.InitialCsv = "";
opts.InitialState = [];
opts.StopTimeS = 25.0;
opts.MetricWindowS = 5.0;
opts.PredictionHorizon = 25;
opts.ControlHorizon = 8;
opts.OutputCsv = "";

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
