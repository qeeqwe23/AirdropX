function [u, state, diag] = airdropx_mpc_controller(x, state, cfg)
%AIRDROPX_MPC_CONTROLLER One grey-box longitudinal MPC step.
%
% x may contain either the five control states or the full seven-state vector:
%   [h*, Vz, Va*, theta*, q, mass*, cg*]'.

if nargin < 3 || isempty(cfg)
    cfg = airdropx_mpc_config();
end
if ~isfield(cfg, "model_bank")
    if isfield(cfg, "model") && ~isempty(cfg.model)
        cfg.model_bank = repmat({cfg.model}, 5, 1);
    else
        cfg.model_bank = airdropx_mpc_greybox_model(cfg);
    end
end

x = double(x(:));

dropCount = local_drop_count_from_state(x, cfg);
model = cfg.model_bank{dropCount + 1};
A = double(model.A);
B = double(model.B);
if isfield(model, "c")
    c = double(model.c(:));
elseif isfield(model, "d")
    c = double(model.d(:));
else
    c = zeros(size(A, 1), 1);
end
if isfield(model, "u_trim")
    uTrim = double(model.u_trim(:));
else
    uTrim = double(cfg.trim.u(:));
end

n = size(A, 1);
m = size(B, 2);
if numel(x) < n
    error("MPC state has %d elements, expected at least %d.", numel(x), n);
end
xCtrl = x(1:n);
N = double(cfg.prediction_horizon);
M = min(double(cfg.control_horizon), N);

if nargin < 2 || isempty(state)
    state = struct();
end
if ~isfield(state, "u_prev") || isempty(state.u_prev)
    state.u_prev = uTrim;
end
if ~isfield(state, "integral_error") || isempty(state.integral_error)
    state.integral_error = zeros(3, 1);
end

[state, integralBias] = local_integral_bias(xCtrl, state, cfg);
[Phi, GammaFull, affineOffset] = local_prediction_matrices_affine(A, B, c, N);
Gamma = local_hold_after_control_horizon(GammaFull, n, m, N, M);

Qbar = kron(eye(N), double(cfg.weights.Q));
terminalRows = (N - 1) * n + (1:n);
Qbar(terminalRows, terminalRows) = double(cfg.weights.terminal_scale) * double(cfg.weights.Q);
Rbar = kron(eye(M), double(cfg.weights.R));
Rdbar = kron(eye(M), double(cfg.weights.Rd));

[D, rateOffset] = local_rate_matrix(m, M, state.u_prev - uTrim);
freePrediction = Phi * xCtrl + affineOffset;
vRef = repmat(integralBias, M, 1);

H = Gamma' * Qbar * Gamma + Rbar + D' * Rdbar * D;
f = Gamma' * Qbar * freePrediction - Rbar * vRef - D' * Rdbar * rateOffset;
H = 0.5 * (H + H') + 1.0e-8 * eye(size(H));

[vStack, solverInfo] = local_solve_move(H, f, D, rateOffset, uTrim, cfg, M, Gamma, freePrediction, n);
vFirst = vStack(1:m);
uRaw = uTrim + vFirst + local_direct_output_bias(xCtrl, cfg);

du = uRaw - state.u_prev;
du = min(max(du, double(cfg.constraints.du_min(:))), double(cfg.constraints.du_max(:)));
u = state.u_prev + du;
u = min(max(u, double(cfg.constraints.u_min(:))), double(cfg.constraints.u_max(:)));

state.u_prev = u;
state.drop_count = dropCount;

xPred = freePrediction + Gamma * vStack;
diag = struct();
diag.drop_count = dropCount;
diag.model_source = string(model.source);
diag.u_trim = uTrim;
diag.u_raw = uRaw;
diag.v_first = vFirst;
diag.integral_bias = integralBias;
diag.direct_output_bias = local_direct_output_bias(xCtrl, cfg);
diag.solver = solverInfo;
diag.x_prediction = reshape(xPred, n, N).';
end

function bias = local_direct_output_bias(x, cfg)
bias = zeros(2, 1);
end

function dropCount = local_drop_count_from_state(x, cfg)
dropCount = 0;
if numel(x) < 6 || ~isfield(cfg, "mass")
    return;
end
cargoMass = mean(double(cfg.mass.cargo_mass_kg(:)), "omitnan");
if ~isfinite(cargoMass) || cargoMass <= 0
    cargoMass = 300.0;
end
massErr = double(x(6));
dropCount = round(max(0.0, -massErr / cargoMass));
dropCount = min(max(dropCount, 0), double(cfg.mass.drop_count_max));
end

function [state, bias] = local_integral_bias(x, state, cfg)
bias = zeros(2, 1);
if isfield(cfg, "integral_feedback") && logical(cfg.integral_feedback.enabled)
    dt = max(double(cfg.dt_s), eps);
    err = [x(1); x(3); x(4)];
    state.integral_error = 0.995 * state.integral_error + dt * err;
    limit = local_integrator_limit(cfg);
    state.integral_error = min(max(state.integral_error, -limit), limit);

    bias = [
        cfg.integral_feedback.h_gain_elevator * state.integral_error(1) + ...
            cfg.integral_feedback.pitch_gain_elevator * state.integral_error(3)
        cfg.integral_feedback.v_gain_throttle * state.integral_error(2) + ...
            cfg.integral_feedback.h_gain_throttle * state.integral_error(1)
        ];
    limit = double(cfg.integral_feedback.limit(:));
    bias = min(max(bias, -limit), limit);
    bias = bias + local_mass_cg_bias(x, cfg) + local_safety_bias(x, cfg);
    return;
end
if ~isfield(cfg, "integrator") || ~logical(cfg.integrator.enabled)
    bias = local_mass_cg_bias(x, cfg) + local_safety_bias(x, cfg);
    return;
end
err = [x(1); x(3); x(4)];
dt = max(double(cfg.dt_s), eps);
state.integral_error = double(cfg.integrator.leak) * state.integral_error + dt * err;
limit = double(cfg.integrator.limit(:));
state.integral_error = min(max(state.integral_error, -limit), limit);
bias = double(cfg.integrator.gain) * state.integral_error;
uMin = double(cfg.constraints.u_min(:)) - double(cfg.trim.u(:));
uMax = double(cfg.constraints.u_max(:)) - double(cfg.trim.u(:));
bias = min(max(bias, uMin), uMax);
bias = bias + local_mass_cg_bias(x, cfg) + local_safety_bias(x, cfg);
end

function limit = local_integrator_limit(cfg)
limit = [25.0; 15.0; 20.0];
if isfield(cfg, "integrator")
    if isfield(cfg.integrator, "limit") && numel(cfg.integrator.limit) >= 3
        limit = double(cfg.integrator.limit(1:3));
    else
        if isfield(cfg.integrator, "h_limit")
            limit(1) = double(cfg.integrator.h_limit);
        end
        if isfield(cfg.integrator, "v_limit")
            limit(2) = double(cfg.integrator.v_limit);
        end
    end
end
limit = abs(limit(:));
end

function uBias = local_safety_bias(x, cfg)
uBias = zeros(2, 1);
if ~isfield(cfg, "safety_feedback") || ~logical(cfg.safety_feedback.enabled) || numel(x) < 2
    return;
end

hLow = min(0.0, double(x(1)) + double(cfg.safety_feedback.h_deadband_m));
vzDown = min(0.0, double(x(2)) + double(cfg.safety_feedback.vz_deadband_mps));
uBias = [
    cfg.safety_feedback.h_gain_elevator * hLow + ...
        cfg.safety_feedback.vz_gain_elevator * vzDown
    cfg.safety_feedback.h_gain_throttle * hLow + ...
        cfg.safety_feedback.vz_gain_throttle * vzDown
    ];

limit = double(cfg.safety_feedback.limit(:));
uBias = min(max(uBias, -limit), limit);
end

function uBias = local_mass_cg_bias(x, cfg)
uBias = zeros(2, 1);
if ~isfield(cfg, "mass_feedback") || ~logical(cfg.mass_feedback.enabled) || numel(x) < 7
    return;
end

massErrKg = double(x(6));
cgErrM = double(x(7));
uBias = [
    cfg.mass_feedback.mass_gain_elevator * massErrKg + ...
        cfg.mass_feedback.cg_gain_elevator * cgErrM
    cfg.mass_feedback.mass_gain_throttle * massErrKg + ...
        cfg.mass_feedback.cg_gain_throttle * cgErrM
    ];

limit = double(cfg.mass_feedback.limit(:));
uBias = min(max(uBias, -limit), limit);
end

function [Phi, Gamma, affineOffset] = local_prediction_matrices_affine(A, B, c, horizon)
[Phi, Gamma] = airdropx_mpc_prediction_matrices(A, B, horizon);
n = size(A, 1);
affineOffset = zeros(horizon * n, 1);
acc = zeros(n, 1);
for k = 1:horizon
    acc = A * acc + c;
    affineOffset((k - 1) * n + (1:n)) = acc;
end
end

function [vStack, info] = local_solve_move(H, f, D, rateOffset, uTrim, cfg, M, Gamma, freePrediction, n)
info = struct("type", "unconstrained", "exitflag", NaN, "message", "", "used_fallback", false);
if local_use_quadprog(cfg)
    [lb, ub] = local_input_bounds(uTrim, cfg, M);
    [Aineq, bineq] = local_rate_inequalities(D, rateOffset, cfg, M);
    [Ax, bx] = local_state_inequalities(Gamma, freePrediction, cfg, n);
    Aineq = [Aineq; Ax];
    bineq = [bineq; bx];
    try
        options = optimoptions("quadprog", "Display", "off", ...
            "MaxIterations", double(cfg.solver.max_iterations));
        [candidate, ~, exitflag, output] = quadprog(H, f, Aineq, bineq, [], [], lb, ub, [], options);
        if exitflag > 0 && all(isfinite(candidate))
            vStack = candidate(:);
            info.type = "quadprog";
            info.exitflag = double(exitflag);
            info.message = string(output.message);
            return;
        end
        info.exitflag = double(exitflag);
        info.message = string(output.message);
    catch ME
        info.message = string(ME.message);
    end
    info.used_fallback = true;
    if isfield(cfg.solver, "fallback") && strcmpi(string(cfg.solver.fallback), "error")
        error("AirdropX:MPC:QpFailed", "MPC QP failed: %s", info.message);
    end
end
if rcond(H) < 1.0e-10
    vStack = -pinv(H) * f;
else
    vStack = -H \ f;
end
end

function tf = local_use_quadprog(cfg)
tf = isfield(cfg, "solver") && logical(cfg.solver.use_constraints) && ...
    strcmpi(string(cfg.solver.type), "quadprog") && exist("quadprog", "file") == 2;
end

function [lb, ub] = local_input_bounds(uTrim, cfg, M)
uMin = double(cfg.constraints.u_min(:));
uMax = double(cfg.constraints.u_max(:));
lb = repmat(uMin - uTrim, M, 1);
ub = repmat(uMax - uTrim, M, 1);
end

function [Aineq, bineq] = local_rate_inequalities(D, rateOffset, cfg, M)
duMin = repmat(double(cfg.constraints.du_min(:)), M, 1);
duMax = repmat(double(cfg.constraints.du_max(:)), M, 1);
Aineq = [D; -D];
bineq = [duMax + rateOffset; -duMin - rateOffset];
end

function [Aineq, bineq] = local_state_inequalities(Gamma, freePrediction, cfg, n)
Aineq = [];
bineq = [];
if ~isfield(cfg, "constraints") || isempty(cfg.constraints.x_min) || isempty(cfg.constraints.x_max)
    return;
end
N = size(Gamma, 1) / n;
if numel(cfg.constraints.x_min) ~= n || numel(cfg.constraints.x_max) ~= n
    return;
end
xMin = repmat(double(cfg.constraints.x_min(:)), N, 1);
xMax = repmat(double(cfg.constraints.x_max(:)), N, 1);
Aineq = [Gamma; -Gamma];
bineq = [xMax - freePrediction; -xMin + freePrediction];
end

function Gamma = local_hold_after_control_horizon(GammaFull, n, m, N, M)
Gamma = GammaFull(:, 1:(M * m));
if M >= N
    return;
end
lastCols = (M - 1) * m + (1:m);
for move = (M + 1):N
    sourceCols = (move - 1) * m + (1:m);
    Gamma(:, lastCols) = Gamma(:, lastCols) + GammaFull(:, sourceCols);
end
end

function [D, rateOffset] = local_rate_matrix(m, M, prevDeviation)
D = zeros(M * m, M * m);
rateOffset = zeros(M * m, 1);
for k = 1:M
    rows = (k - 1) * m + (1:m);
    cols = rows;
    D(rows, cols) = eye(m);
    if k == 1
        rateOffset(rows) = prevDeviation(:);
    else
        prevCols = (k - 2) * m + (1:m);
        D(rows, prevCols) = -eye(m);
    end
end
end
