function [u, state, diag] = airdropx_mpc_controller(x, state, cfg)
%AIRDROPX_MPC_CONTROLLER One standalone MPC step.
%
% Inputs:
%   x     - current state vector in cfg.state_names order
%   state - controller memory, pass [] on first call
%   cfg   - airdropx_mpc_config output
%
% This function is not connected to Simulink. It solves an unconstrained MPC
% move and then applies input/rate clipping for review and offline replay.

if nargin < 3 || isempty(cfg)
    cfg = airdropx_mpc_config();
end
if ~isfield(cfg, "model")
    cfg.model = airdropx_mpc_nominal_model(cfg);
end

x = double(x(:));
A = double(cfg.model.A);
B = double(cfg.model.B);
uTrim = double(cfg.model.u_trim(:));
if isfield(cfg.model, "c") && ~isempty(cfg.model.c)
    c = double(cfg.model.c(:));
else
    c = zeros(size(A, 1), 1);
end

n = size(A, 1);
m = size(B, 2);
N = double(cfg.prediction_horizon);
M = min(double(cfg.control_horizon), N);

if numel(x) ~= n
    error("MPC state has %d elements, expected %d.", numel(x), n);
end

if nargin < 2 || isempty(state)
    state = struct();
end
if ~isfield(state, "u_prev") || isempty(state.u_prev)
    state.u_prev = uTrim;
end
if ~isfield(state, "integral_error") || isempty(state.integral_error)
    state.integral_error = zeros(3, 1); % [h_err; v_err; pitch_err]
end

integralBias = local_integral_bias(x, state, cfg);
state.integral_error = integralBias.integral_error;

[Phi, GammaFull, affineOffset] = local_prediction_matrices_affine(A, B, c, N);
Gamma = local_hold_after_control_horizon(GammaFull, n, m, N, M);
duRef = local_equilibrium_delta(B, c, cfg, M, integralBias.input_bias);

Qbar = kron(eye(N), cfg.weights.Q);
terminalRows = (N - 1) * n + (1:n);
Qbar(terminalRows, terminalRows) = cfg.weights.terminal_scale * cfg.weights.Q;
Rbar = kron(eye(M), cfg.weights.R);
Rdbar = kron(eye(M), cfg.weights.Rd);
[D, b] = local_rate_matrix(m, M, state.u_prev - uTrim);

freePrediction = Phi * x + affineOffset;
H = Gamma' * Qbar * Gamma + Rbar + D' * Rdbar * D;
f = Gamma' * Qbar * freePrediction - Rbar * duRef - D' * Rdbar * b;
H = 0.5 * (H + H');
reg = max(1.0e-7, 1.0e-8 * trace(H) / max(size(H, 1), 1));
H = H + reg * eye(size(H));

if rcond(H) < 1.0e-10
    duStack = -pinv(H) * f;
else
duStack = -H \ f;
end
duFirst = duStack(1:m);
directSafetyBias = local_safety_bias(x, cfg);
uRaw = uTrim + duFirst + directSafetyBias;

uPrev = state.u_prev;
duRate = uRaw - state.u_prev;
duRate = min(max(duRate, cfg.constraints.du_min(:)), cfg.constraints.du_max(:));
uRateLimited = state.u_prev + duRate;
u = min(max(uRateLimited, cfg.constraints.u_min(:)), cfg.constraints.u_max(:));

xPred = freePrediction + Gamma * duStack;
state.u_prev = u;

diag = struct();
diag.u_raw = uRaw;
diag.du_raw = uRaw - uPrev;
diag.u_trim = uTrim;
diag.u_equilibrium = uTrim + duRef(1:m);
diag.integral_input_bias = integralBias.input_bias;
diag.du_first = duFirst;
diag.x_prediction = reshape(xPred, n, N).';
diag.cost_gradient_norm = norm(f);
diag.hessian_condition = cond(H);
diag.hessian_rcond = rcond(H);
diag.hessian_regularization = reg;
diag.used_model_source = string(cfg.model.source);
end

function [Phi, Gamma, affineOffset] = local_prediction_matrices_affine(A, B, c, horizon)
[Phi, Gamma] = airdropx_mpc_prediction_matrices(A, B, horizon);

n = size(A, 1);
N = double(horizon);
affineOffset = zeros(N * n, 1);
acc = zeros(n, 1);
for i = 1:N
    acc = A * acc + c;
    affineOffset((i - 1) * n + (1:n)) = acc;
end
end

function bias = local_integral_bias(x, state, cfg)
bias = struct();
bias.integral_error = state.integral_error;
bias.input_bias = zeros(2, 1);

if ~isfield(cfg, "integral_feedback") || ~cfg.integral_feedback.enabled
    bias.input_bias = local_mass_cg_bias(x, cfg) + local_safety_bias(x, cfg);
    return;
end

dt = double(cfg.dt_s);
if ~isfinite(dt) || dt <= 0.0
    dt = 0.1;
end

err = [x(1); x(3); x(4)];
newIntegral = 0.995 * state.integral_error + dt * err;
newIntegral(1) = min(max(newIntegral(1), -cfg.integrator.h_limit), cfg.integrator.h_limit);
newIntegral(2) = min(max(newIntegral(2), -cfg.integrator.v_limit), cfg.integrator.v_limit);
newIntegral(3) = min(max(newIntegral(3), -20.0), 20.0);

uBias = [
    cfg.integral_feedback.h_gain_elevator * newIntegral(1) + ...
        cfg.integral_feedback.pitch_gain_elevator * newIntegral(3)
    cfg.integral_feedback.v_gain_throttle * newIntegral(2) + ...
        cfg.integral_feedback.h_gain_throttle * newIntegral(1)
    ];
limit = double(cfg.integral_feedback.limit(:));
uBias = min(max(uBias, -limit), limit);

bias.integral_error = newIntegral;
bias.input_bias = uBias + local_mass_cg_bias(x, cfg) + local_safety_bias(x, cfg);
end

function uBias = local_safety_bias(x, cfg)
uBias = zeros(2, 1);
if ~isfield(cfg, "safety_feedback") || ~cfg.safety_feedback.enabled || numel(x) < 2
    return;
end

hLow = min(0.0, double(x(1)) + cfg.safety_feedback.h_deadband_m);
vzDown = min(0.0, double(x(2)) + cfg.safety_feedback.vz_deadband_mps);
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
if ~isfield(cfg, "mass_feedback") || ~cfg.mass_feedback.enabled || numel(x) < 7
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

function duRef = local_equilibrium_delta(B, c, cfg, M, inputBias)
m = size(B, 2);
if isfield(cfg.model, "u_equilibrium_delta") && ~isempty(cfg.model.u_equilibrium_delta)
    duEq = double(cfg.model.u_equilibrium_delta(:));
else
    W = double(cfg.weights.Q);
    rho = 1.0e-3;
    duEq = -((B' * W * B) + rho * eye(m)) \ (B' * W * c);
end
if nargin >= 5 && ~isempty(inputBias)
    duEq = duEq + double(inputBias(:));
end

uTrim = double(cfg.model.u_trim(:));
uEq = uTrim + duEq;
uEq = min(max(uEq, cfg.constraints.u_min(:)), cfg.constraints.u_max(:));
duEq = uEq - uTrim;
duRef = repmat(duEq, M, 1);
end

function Gamma = local_hold_after_control_horizon(GammaFull, n, m, N, M)
Gamma = GammaFull(:, 1:(M * m));
if M >= N
    return;
end

% The final optimized move is held constant after the control horizon.
for move = (M + 1):N
    sourceCols = (move - 1) * m + (1:m);
    lastCols = (M - 1) * m + (1:m);
    Gamma(:, lastCols) = Gamma(:, lastCols) + GammaFull(:, sourceCols);
end
end

function [D, b] = local_rate_matrix(m, M, duPrev)
D = zeros(M * m, M * m);
b = zeros(M * m, 1);
for i = 1:M
    rows = (i - 1) * m + (1:m);
    cols = rows;
    D(rows, cols) = eye(m);
    if i == 1
        b(rows) = duPrev(:);
    else
        prevCols = (i - 2) * m + (1:m);
        D(rows, prevCols) = -eye(m);
    end
end
end
