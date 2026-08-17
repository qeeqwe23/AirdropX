function result = airdropx_auto_mpc_200m_autotune(varargin)
%AIRDROPX_AUTO_MPC_200M_AUTOTUNE Validate the automatic MPC path at 200 m.
%
% v15 purpose:
%   Freeze the proven v11 identified cfg0 plant and the v14 physical-elevator
%   bridge, move only the MPC operating/reference altitude to 200 m, then run:
%
%     1) hidden-trim calibration at 200 m
%     2) bridge-only preflight
%     3) guarded Bayesian MPC optimization on cfg0
%     4) independent 60 s final validation
%
% This function deliberately does NOT regenerate trim/ID/n4sid and does NOT
% test cfg1/cfg2.  The goal is to prove the MPC automatic-optimization road on
% the altitude where the real JSBSim cfg0 operating point is already known to
% be self-consistent.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

outRoot = string(opts.OutputRoot);
if strlength(outRoot) == 0
    outRoot = string(fullfile(paths.matlabDir, "results", ...
        "mpc_auto_mpc_200m_autotune_v15_" + string(datetime("now","Format","yyyyMMdd_HHmmss"))));
end
if ~isfolder(outRoot), mkdir(outRoot); end

S = load(opts.IdentifiedMat, "result");
identified = S.result;
if isempty(identified.plant_bank{1})
    error("AirdropX:AutoMPC200:MissingCfg0", "Identified cfg0 Plant is empty.");
end

% Create an MPC-only 200 m operating-point view.  The identified dynamics are
% run-centered deviations, so changing the altitude origin from 20 m to 200 m
% does not change A/B/C/D.  It only makes the online deviation coordinate and
% reference consistent with this 200 m validation experiment.
identified200 = identified;
for k = 1:numel(identified200.trim_bank)
    identified200.trim_bank(k).altitude_m = double(opts.TargetAltitudeM);
    identified200.trim_bank(k).airspeed_mps = double(opts.TargetAirspeedMps);
    identified200.trim_bank(k).vz_up_mps = 0.0;
    identified200.trim_bank(k).q_dps = 0.0;
end
% v15 proves cfg0 first.  Empty the other plants so bank construction cannot
% accidentally require/touch cfg1/cfg2 physical nominals.
for k = 2:numel(identified200.plant_bank)
    identified200.plant_bank{k} = [];
end
trimSource = identified.trim_bank(1);
trim200 = identified200.trim_bank(1);

physicalNom = local_resolve_physical_elevator_nominal(identified, 0, opts);
physicalNominals = NaN(5,1);
physicalNominals(1) = physicalNom;

save(fullfile(outRoot, "identified_cfg0_200m_overlay.mat"), "identified200", "physicalNom");

% Build a safe seed bank only so that run_closed_loop can execute bridge-only
% preflight before optimization starts.
seed = local_seed_candidate(opts);
seedBankMat = fullfile(outRoot, "seed_mpc_bank_200m.mat");
local_build_bank(identified200, physicalNominals, seedBankMat, seed, opts);

% -------------------------------------------------------------------------
% Phase 0A: calibrate the hidden S-function elevator trim for each fixed
% initial condition used later.  This keeps the bridge exact even if JSBSim's
% internal reset trim changes slightly with h0/V0.
% -------------------------------------------------------------------------
hiddenTrim = local_calibrate_hidden_trim(paths, trimSource, ...
    opts.TargetAltitudeM, opts.TargetAirspeedMps, trim200.pitch_deg, ...
    fullfile(outRoot, "hidden_trim_calibration_nominal"), "cfg0_200m_nominal", opts);
initialDelta = physicalNom - hiddenTrim;

hiddenTrimHighSlow = local_calibrate_hidden_trim(paths, trimSource, ...
    opts.TargetAltitudeM + 2.0, opts.TargetAirspeedMps - 2.0, trim200.pitch_deg, ...
    fullfile(outRoot, "hidden_trim_calibration_high_slow"), "cfg0_202m_48mps", opts);
initialDeltaHighSlow = physicalNom - hiddenTrimHighSlow;

hiddenTrimLowFast = local_calibrate_hidden_trim(paths, trimSource, ...
    opts.TargetAltitudeM - 2.0, opts.TargetAirspeedMps + 2.0, trim200.pitch_deg, ...
    fullfile(outRoot, "hidden_trim_calibration_low_fast"), "cfg0_198m_52mps", opts);
initialDeltaLowFast = physicalNom - hiddenTrimLowFast;

calSummary = table( ...
    [opts.TargetAltitudeM; opts.TargetAltitudeM+2.0; opts.TargetAltitudeM-2.0], ...
    [opts.TargetAirspeedMps; opts.TargetAirspeedMps-2.0; opts.TargetAirspeedMps+2.0], ...
    repmat(physicalNom,3,1), ...
    [hiddenTrim; hiddenTrimHighSlow; hiddenTrimLowFast], ...
    [initialDelta; initialDeltaHighSlow; initialDeltaLowFast], ...
    repmat(double(trim200.throttle_cmd),3,1), ...
    'VariableNames', {'initial_h_m','initial_V_mps','physical_elevator_nominal', ...
    'hidden_elevator_trim','initial_external_delta','throttle_nominal'});
writetable(calSummary, fullfile(outRoot, "hidden_trim_summary.csv"));

fprintf("\n[V15-200m] physical elevator nominal = %.6f\n", physicalNom);
fprintf("[V15-200m] hidden trim nominal        = %.6f\n", hiddenTrim);
fprintf("[V15-200m] external delta nominal     = %.6f\n", initialDelta);
fprintf("[V15-200m] hidden trim high/slow      = %.6f\n", hiddenTrimHighSlow);
fprintf("[V15-200m] hidden trim low/fast       = %.6f\n", hiddenTrimLowFast);

if abs(physicalNom - double(opts.ExpectedPhysicalElevatorNominal)) > ...
        double(opts.PhysicalNominalWarningTolerance)
    warning("AirdropX:AutoMPC200:PhysicalNominalShift", ...
        "Resolved physical elevator %.6f differs from the proven 200 m probe %.6f.", ...
        physicalNom, double(opts.ExpectedPhysicalElevatorNominal));
end

% -------------------------------------------------------------------------
% Phase 0B: bridge-only preflight.  No MPC authority is allowed here.
% -------------------------------------------------------------------------
pre = local_run_case(paths, seedBankMat, outRoot, "bridge_only_preflight", ...
    opts.TargetAltitudeM, opts.TargetAirspeedMps, trim200.pitch_deg, ...
    hiddenTrim, initialDelta, trim200.throttle_cmd, 0.0, ...
    opts.PreflightStopTimeS, opts.PreflightStopTimeS + 100, opts);
preM = local_metrics(pre.timeseries, physicalNom, trim200.throttle_cmd, opts, ...
    opts.TargetAltitudeM, opts.TargetAirspeedMps);
preM = addvars(preM, "bridge_only_preflight", 'Before', 1, 'NewVariableNames', {'case_name'});
writetable(preM, fullfile(outRoot, "bridge_only_preflight_summary.csv"));
local_plot(pre.timeseries, physicalNom, trim200.throttle_cmd, ...
    fullfile(outRoot, "bridge_only_preflight.png"), "v15 200m bridge-only preflight");

if preM.hard_fail || preM.max_bridge_elevator_error > opts.MaxBridgeError || ...
        preM.max_bridge_throttle_error > opts.MaxBridgeError || ...
        preM.steady_h_rms_m > opts.PreflightMaxAltitudeRmsM
    error("AirdropX:AutoMPC200:PreflightFailed", ...
        ['200 m bridge-only preflight failed. Stop before Bayesopt. ' ...
         'hRMS=%.3f bridgeE=%.4g bridgeT=%.4g'], ...
        preM.steady_h_rms_m, preM.max_bridge_elevator_error, preM.max_bridge_throttle_error);
end

fprintf("[V15-200m] Bridge-only preflight PASS: hRMS=%.3f m, tail h err=%.3f m\n", ...
    preM.steady_h_rms_m, preM.tail_h_error_m);

% -------------------------------------------------------------------------
% Phase 1: guarded Bayesian optimization.  Keep variables intentionally narrow.
% -------------------------------------------------------------------------
historyCsv = fullfile(outRoot, "optimization_history.csv");
ctx = struct();
ctx.paths = paths;
ctx.identified200 = identified200;
ctx.physicalNominals = physicalNominals;
ctx.physicalNom = physicalNom;
ctx.hiddenTrim = hiddenTrim;
ctx.initialDelta = initialDelta;
ctx.hiddenTrimHighSlow = hiddenTrimHighSlow;
ctx.initialDeltaHighSlow = initialDeltaHighSlow;
ctx.hiddenTrimLowFast = hiddenTrimLowFast;
ctx.initialDeltaLowFast = initialDeltaLowFast;
ctx.trim200 = trim200;
ctx.outRoot = outRoot;
ctx.historyCsv = string(historyCsv);
ctx.opts = opts;

vars = [ ...
    optimizableVariable("Np", [6 14], "Type", "integer")
    optimizableVariable("Nc", [2 4], "Type", "integer")
    optimizableVariable("Wh", [4.0 40.0], "Transform", "log")
    optimizableVariable("Wvz", [1.0 15.0], "Transform", "log")
    optimizableVariable("Wq", [1.0 15.0], "Transform", "log")
    optimizableVariable("RateScale", [0.5 3.0], "Transform", "log")
    optimizableVariable("Authority", [0.50 1.00])
    ];

initialX = local_initial_points(historyCsv, opts);
objective = @(x) local_objective(x, ctx);

fprintf("\n[V15-200m] Starting Bayesopt: %d evaluations, cfg0 only, no cfg1/cfg2.\n", ...
    double(opts.MaxObjectiveEvaluations));
bo = bayesopt(objective, vars, ...
    "MaxObjectiveEvaluations", double(opts.MaxObjectiveEvaluations), ...
    "InitialX", initialX, ...
    "IsObjectiveDeterministic", true, ...
    "UseParallel", logical(opts.UseParallel), ...
    "AcquisitionFunctionName", "expected-improvement-plus", ...
    "Verbose", double(opts.BayesoptVerbose));

save(fullfile(outRoot, "bayesopt_result.mat"), "bo", "opts", "hiddenTrim", "physicalNom");
bestX = bo.XAtMinObjective;
best = local_candidate_from_x(bestX, opts);

bestTable = table(best.Np, best.Nc, best.Wh, best.Wvz, best.Wq, best.RateScale, ...
    best.Authority, double(bo.MinObjective), ...
    'VariableNames', {'Np','Nc','Wh','Wvz','Wq','RateScale','Authority','objective'});
writetable(bestTable, fullfile(outRoot, "best_candidate.csv"));

fprintf("\n[V15-200m] Best candidate: Np=%d Nc=%d Wh=%.3g Wvz=%.3g Wq=%.3g rate=%.3g authority=%.3f obj=%.4g\n", ...
    best.Np, best.Nc, best.Wh, best.Wvz, best.Wq, best.RateScale, best.Authority, bo.MinObjective);

% -------------------------------------------------------------------------
% Phase 2: independent, longer final validation.  This is the road gate.
% -------------------------------------------------------------------------
bestBankMat = fullfile(outRoot, "best_mpc_bank_200m.mat");
local_build_bank(identified200, physicalNominals, bestBankMat, best, opts);

finalCases = [ ...
    struct("name","final_nominal",  "h0",opts.TargetAltitudeM,     "V0",opts.TargetAirspeedMps, ...
        "hidden",hiddenTrim, "delta",initialDelta)
    struct("name","final_high_slow","h0",opts.TargetAltitudeM+2.0, "V0",opts.TargetAirspeedMps-2.0, ...
        "hidden",hiddenTrimHighSlow, "delta",initialDeltaHighSlow)
    struct("name","final_low_fast", "h0",opts.TargetAltitudeM-2.0, "V0",opts.TargetAirspeedMps+2.0, ...
        "hidden",hiddenTrimLowFast, "delta",initialDeltaLowFast)
    ];

finalRows = table();
for i = 1:numel(finalCases)
    c = finalCases(i);
    simResult = local_run_case(paths, bestBankMat, outRoot, c.name, ...
        c.h0, c.V0, trim200.pitch_deg, c.hidden, c.delta, ...
        trim200.throttle_cmd, best.Authority, opts.FinalStopTimeS, ...
        opts.MpcEnableTimeS, opts);
    m = local_metrics(simResult.timeseries, physicalNom, trim200.throttle_cmd, opts, ...
        opts.TargetAltitudeM, opts.TargetAirspeedMps);
    m = addvars(m, string(c.name), double(c.h0), double(c.V0), 'Before', 1, ...
        'NewVariableNames', {'case_name','initial_h_m','initial_V_mps'});
    finalRows = [finalRows; m]; %#ok<AGROW>
    local_plot(simResult.timeseries, physicalNom, trim200.throttle_cmd, ...
        fullfile(outRoot, string(c.name) + "_curves.png"), "v15 200m " + string(c.name));
end
writetable(finalRows, fullfile(outRoot, "final_validation_summary.csv"));

roadPass = all(finalRows.formal_pass) && ~any(finalRows.hard_fail);
roadSummary = table(logical(roadPass), double(opts.TargetAltitudeM), ...
    double(opts.TargetAirspeedMps), physicalNom, hiddenTrim, double(bo.MinObjective), ...
    'VariableNames', {'road_pass','target_altitude_m','target_airspeed_mps', ...
    'physical_elevator_nominal','hidden_elevator_trim','best_objective'});
writetable(roadSummary, fullfile(outRoot, "road_validation_summary.csv"));

result = struct();
result.output_root = outRoot;
result.target_altitude_m = double(opts.TargetAltitudeM);
result.physical_elevator_nominal = physicalNom;
result.hidden_trim = hiddenTrim;
result.best_candidate = bestTable;
result.best_mpc_bank = string(bestBankMat);
result.final_validation = finalRows;
result.road_pass = logical(roadPass);
result.bayesopt = bo;
save(fullfile(outRoot, "v15_200m_autotune_result.mat"), "result", "opts", "-v7.3");

if roadPass
    fprintf("\n[V15-200m] SUCCESS: cfg0 automatic MPC optimization road PASSED at 200 m.\n");
else
    fprintf("\n[V15-200m] NOT YET PASS: optimization completed safely, but final validation missed formal gates.\n");
end
fprintf("[V15-200m] Results: %s\n", outRoot);
end

function hiddenTrim = local_calibrate_hidden_trim(paths, trimSource, h0, V0, pitch0, outDir, runId, opts)
cal = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputRoot", outDir, ...
    "RunId", runId, ...
    "ConfigId", 0, "Trim", trimSource, ...
    "StopTimeS", opts.CalibrationStopTimeS, ...
    "RecordStartS", 0.0, "ExportStartS", 0.0, ...
    "ExcitationStartS", 100.0, ...
    "ElevatorAmplitude", 0.0, "ThrottleAmplitude", 0.0, ...
    "DirectIdMode", true, "KeepFixedConfigurationOnly", true, ...
    "InitialAltitudeM", h0, ...
    "InitialAirspeedMps", V0, ...
    "InitialPitchDeg", pitch0, ...
    "InitialFlightPathDeg", 0.0, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps);
T = cal.timeseries;
external = local_col(T, "requested_elevator_cmd");
physical = local_col(T, "elevator_cmd_norm");
mask = isfinite(external) & isfinite(physical) & ...
    double(T.time_s) <= double(opts.CalibrationUseUntilS);
if nnz(mask) < 3
    error("AirdropX:AutoMPC200:HiddenTrimCalibration", ...
        "Not enough samples to calibrate hidden trim for h0=%.1f V0=%.1f.", h0, V0);
end
hiddenTrim = median(physical(mask) - external(mask), "omitnan");
end

function score = local_objective(x, ctx)
opts = ctx.opts;
c = local_candidate_from_x(x, opts);
[~, token] = fileparts(tempname);
tag = "eval_" + string(datetime("now","Format","yyyyMMdd_HHmmss_SSS")) + "_" + string(token);
evalRoot = fullfile(ctx.outRoot, "optimization", tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
bankMat = fullfile(evalRoot, "mpc_bank.mat");

try
    local_build_bank(ctx.identified200, ctx.physicalNominals, bankMat, c, opts);
    defs = [ ...
        struct("name","nominal", "h0",opts.TargetAltitudeM,     "V0",opts.TargetAirspeedMps, ...
            "hidden",ctx.hiddenTrim, "delta",ctx.initialDelta, "factor",1.0)
        struct("name","perturb", "h0",opts.TargetAltitudeM+2.0, "V0",opts.TargetAirspeedMps-2.0, ...
            "hidden",ctx.hiddenTrimHighSlow, "delta",ctx.initialDeltaHighSlow, "factor",1.25)
        ];
    rows = table();
    score = 0.0;
    anyHard = false;
    allFormal = true;
    for i = 1:numel(defs)
        d = defs(i);
        simResult = local_run_case(ctx.paths, bankMat, evalRoot, d.name, ...
            d.h0, d.V0, ctx.trim200.pitch_deg, d.hidden, d.delta, ...
            ctx.trim200.throttle_cmd, c.Authority, opts.OptimizationStopTimeS, ...
            opts.MpcEnableTimeS, opts);
        m = local_metrics(simResult.timeseries, ctx.physicalNom, ctx.trim200.throttle_cmd, ...
            opts, opts.TargetAltitudeM, opts.TargetAirspeedMps);
        m = addvars(m, string(d.name), 'Before', 1, 'NewVariableNames', {'case_name'});
        rows = [rows; m]; %#ok<AGROW>
        if m.hard_fail
            anyHard = true;
            score = score + double(opts.HardFailScore);
        else
            score = score + double(d.factor) * double(m.rank_score);
        end
        allFormal = allFormal && logical(m.formal_pass);
    end
    if anyHard || ~isfinite(score)
        score = double(opts.HardFailScore);
    elseif allFormal
        score = score * 0.50;
    end
    writetable(rows, fullfile(evalRoot, "case_metrics.csv"));
catch ME
    score = double(opts.HardFailScore);
    fid = fopen(fullfile(evalRoot, "error.txt"), "w");
    if fid >= 0
        fprintf(fid, "%s\n\n%s\n", ME.message, getReport(ME, "extended", "hyperlinks", "off"));
        fclose(fid);
    end
end

local_append_history(ctx.historyCsv, c, score, tag);
fprintf("[V15-200m] %s obj=%.5g Np=%d Nc=%d Wh=%.3g Wvz=%.3g Wq=%.3g rate=%.3g auth=%.2f\n", ...
    tag, score, c.Np, c.Nc, c.Wh, c.Wvz, c.Wq, c.RateScale, c.Authority);
end

function c = local_candidate_from_x(x, opts)
c = struct();
c.Np = round(double(x.Np));
c.Nc = min(round(double(x.Nc)), c.Np);
c.Wh = double(x.Wh);
c.Wvz = double(x.Wvz);
c.Wq = double(x.Wq);
c.RateScale = double(x.RateScale);
c.Authority = double(x.Authority);
c.Wva = double(opts.FixedAirspeedWeight);
c.Wpitch = double(opts.FixedPitchWeight);
c.MVWeights = double(opts.MVWeights(:)).';
c.MVRateWeights = double(opts.MVRateWeights(:)).' * c.RateScale;
c.disableOD = true;
end

function c = local_seed_candidate(opts)
c = struct();
c.Np = 8;
c.Nc = 3;
c.Wh = 8.0;
c.Wva = double(opts.FixedAirspeedWeight);
c.Wpitch = double(opts.FixedPitchWeight);
c.Wvz = 3.0;
c.Wq = 4.0;
c.RateScale = 1.0;
c.Authority = 0.70;
c.MVWeights = double(opts.MVWeights(:)).';
c.MVRateWeights = double(opts.MVRateWeights(:)).';
c.disableOD = true;
end

function local_build_bank(identified200, physicalNominals, bankMat, c, opts)
airdropx_auto_build_mpc_bank( ...
    "Identified", identified200, "OutputMat", bankMat, ...
    "PredictionHorizon", c.Np, "ControlHorizon", c.Nc, ...
    "InputCoordinateMode", "deviation_physical", ...
    "PhysicalElevatorNominals", physicalNominals, ...
    "RequirePhysicalElevatorNominals", false, ...
    "DerivePhysicalElevatorNominalsFromIdData", false, ...
    "ElevatorDeviationLimit", opts.ElevatorDeviationLimit, ...
    "ThrottleDeviationLimit", opts.ThrottleDeviationLimit, ...
    "ElevatorDeviationRateLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDeviationRateLimit", opts.ThrottleDeviationRateLimit, ...
    "OutputWeights", [c.Wh c.Wva c.Wpitch c.Wvz c.Wq], ...
    "MVWeights", c.MVWeights, ...
    "MVRateWeights", c.MVRateWeights, ...
    "DisableOutputDisturbanceModel", c.disableOD);
end

function simResult = local_run_case(paths, bankMat, parentRoot, caseName, h0, V0, pitch0, ...
    hiddenTrim, initialDelta, throttle0, authority, stopTimeS, enableTimeS, opts)
caseRoot = fullfile(parentRoot, string(caseName));
simResult = airdropx_auto_run_closed_loop( ...
    "ProjectRoot", paths.projectRoot, ...
    "MpcBankMat", bankMat, ...
    "OutputRoot", caseRoot, ...
    "CaseId", string(caseName), ...
    "StopTimeS", stopTimeS, ...
    "FixedConfigId", 0, ...
    "FixedDropTotal", 0, ...
    "FixedDropStartS", stopTimeS + 100, ...
    "InitialAltitudeM", h0, ...
    "InitialAirspeedMps", V0, ...
    "InitialPitchDeg", pitch0, ...
    "InitialFlightPathDeg", 0.0, ...
    "InitialElevatorDelta", initialDelta, ...
    "InitialThrottleCmd", throttle0, ...
    "HiddenElevatorTrim", hiddenTrim, ...
    "MpcEnableTimeS", enableTimeS, ...
    "MpcAuthorityScale", authority, ...
    "ElevatorDevStepLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDevStepLimit", opts.ThrottleDeviationRateLimit, ...
    "TrustAltitudeM", 1.0e6, ...
    "TrustAirspeedMps", opts.TrustAirspeedMps, ...
    "TrustPitchDeg", opts.TrustPitchDeg, ...
    "TrustVzMps", opts.TrustVzMps, ...
    "TrustQDps", opts.TrustQDps, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", pitch0, ...
    "UseTrimPitchReference", 1);
end

function M = local_metrics(T, eNom, tNom, opts, targetH, targetV)
t = double(T.time_s(:));
h = local_col(T, "altitude_m");
V = local_col(T, "airspeed_mps");
vz = local_col(T, "vz_up_mps");
q = local_col(T, "q_dps");
pitch = local_col(T, "pitch_deg");
e = local_col(T, "elevator_cmd_norm");
th = local_col(T, "throttle_norm");
be = local_col(T, "bridge_elevator_error");
bt = local_col(T, "bridge_throttle_error");

steady = isfinite(t) & t >= double(opts.ScoreStartTimeS);
if nnz(steady) < 10, steady = isfinite(t); end
tail = isfinite(t) & t >= max(max(t(isfinite(t))) - double(opts.TailWindowS), 0.0);
if nnz(tail) < 10, tail = steady; end
idx = find(steady);
nEdge = max(3, round(0.15*numel(idx)));
headIdx = idx(1:min(nEdge,numel(idx)));
tailIdx = idx(max(1,numel(idx)-nEdge+1):numel(idx));

hErr = h - double(targetH);
vErr = V - double(targetV);
hRms = local_rms(hErr(steady));
hMax = max(abs(hErr(steady)), [], "omitnan");
hDrift = median(h(tailIdx), "omitnan") - median(h(headIdx), "omitnan");
vaRms = local_rms(vErr(steady));
vzRms = local_rms(vz(steady));
qRms = local_rms(q(steady));
tailHErr = median(hErr(tail), "omitnan");
tailVz = median(vz(tail), "omitnan");
tailQ = median(q(tail), "omitnan");
minH = min(h, [], "omitnan");
maxH = max(h, [], "omitnan");
maxPitch = max(abs(pitch), [], "omitnan");
maxED = max(abs(e - double(eNom)), [], "omitnan");
maxTD = max(abs(th - double(tNom)), [], "omitnan");
maxBE = max(abs(be), [], "omitnan");
maxBT = max(abs(bt), [], "omitnan");

hard = ~isfinite(minH) || ~isfinite(maxH) || ...
    max(abs([minH maxH] - double(targetH))) > double(opts.HardMaxAltitudeErrorM) || ...
    qRms > double(opts.HardMaxQRmsDps) || ...
    maxPitch > double(opts.HardMaxAbsPitchDeg) || ...
    maxBE > double(opts.MaxBridgeError) || maxBT > double(opts.MaxBridgeError);

formal = ~hard && ...
    hRms <= double(opts.PassAltitudeRmsM) && ...
    hMax <= double(opts.PassAltitudeMaxM) && ...
    abs(hDrift) <= double(opts.PassAltitudeDriftM) && ...
    vaRms <= double(opts.PassAirspeedRmsMps) && ...
    vzRms <= double(opts.PassVzRmsMps) && ...
    qRms <= double(opts.PassQRmsDps) && ...
    abs(tailHErr) <= double(opts.PassTailAltitudeErrorM) && ...
    abs(tailVz) <= double(opts.PassTailVzMps);

rankScore = ...
    12.0*hRms + ...
    6.0*abs(hDrift) + ...
    4.0*abs(tailHErr) + ...
    3.0*vaRms + ...
    4.0*vzRms + ...
    2.0*qRms + ...
    4.0*abs(tailVz) + ...
    0.5*abs(tailQ) + ...
    2.0*(maxED/max(double(opts.ElevatorDeviationLimit),eps))^2 + ...
    1.0*(maxTD/max(double(opts.ThrottleDeviationLimit),eps))^2;

M = table(minH,maxH,hRms,hMax,hDrift,vaRms,vzRms,qRms,tailHErr,tailVz,tailQ, ...
    maxED,maxTD,maxBE,maxBT,logical(hard),logical(formal),rankScore, ...
    'VariableNames', {'min_altitude_m','max_altitude_m','steady_h_rms_m', ...
    'steady_h_max_abs_m','steady_h_drift_m','steady_Va_rms_mps', ...
    'steady_vz_rms_mps','steady_q_rms_dps','tail_h_error_m','tail_vz_mps', ...
    'tail_q_dps','max_physical_elevator_deviation','max_throttle_deviation', ...
    'max_bridge_elevator_error','max_bridge_throttle_error','hard_fail', ...
    'formal_pass','rank_score'});
end

function initialX = local_initial_points(historyCsv, opts)
seed = [ ...
    8  3  8.0  3.0  4.0  1.00 0.70
    10 3 12.0  4.0  5.0  1.00 0.85
    12 3 16.0  5.0  6.0  0.80 0.90
    8  2 20.0  6.0  5.0  1.50 0.80
    10 4 10.0  3.0  8.0  0.70 1.00
    ];
rows = seed;
if logical(opts.ReuseHistory) && isfile(historyCsv)
    try
        H = readtable(historyCsv);
        good = isfinite(H.objective) & H.objective < double(opts.HardFailScore);
        H = H(good,:);
        H = sortrows(H, "objective", "ascend");
        n = min(double(opts.ResumeBestPointCount), height(H));
        if n > 0
            prior = [double(H.Np(1:n)) double(H.Nc(1:n)) double(H.Wh(1:n)) ...
                double(H.Wvz(1:n)) double(H.Wq(1:n)) double(H.RateScale(1:n)) ...
                double(H.Authority(1:n))];
            rows = [prior; rows]; %#ok<AGROW>
        end
    catch
    end
end
rows(:,1) = round(rows(:,1));
rows(:,2) = round(rows(:,2));
rows = unique(rows, "rows", "stable");
maxRows = min(size(rows,1), max(1,double(opts.MaxObjectiveEvaluations)-1));
rows = rows(1:maxRows,:);
initialX = array2table(rows, 'VariableNames', ...
    {'Np','Nc','Wh','Wvz','Wq','RateScale','Authority'});
end

function local_append_history(historyCsv, c, score, tag)
row = table(string(tag), datetime("now"), c.Np, c.Nc, c.Wh, c.Wvz, c.Wq, ...
    c.RateScale, c.Authority, double(score), ...
    'VariableNames', {'eval_tag','timestamp','Np','Nc','Wh','Wvz','Wq', ...
    'RateScale','Authority','objective'});
try
    if isfile(historyCsv)
        old = readtable(historyCsv);
        old = [old; row]; %#ok<AGROW>
        writetable(old, historyCsv);
    else
        writetable(row, historyCsv);
    end
catch ME
    warning("AirdropX:AutoMPC200:HistoryWrite", "Could not update optimization history: %s", ME.message);
end
end

function physicalNom = local_resolve_physical_elevator_nominal(identified, cfgId, opts)
if isfinite(double(opts.PhysicalElevatorNominal))
    physicalNom = double(opts.PhysicalElevatorNominal);
    return;
end
vals = [];
files = strings(0,1);
try
    if isfield(identified, "data") && isfield(identified.data, "csv_files")
        files = string(identified.data.csv_files(:));
    end
catch
end
for i = 1:numel(files)
    f = files(i);
    if ~isfile(f), continue; end
    try
        T = readtable(f);
        if ~ismember("config_id", string(T.Properties.VariableNames)) || ...
                round(median(double(T.config_id), "omitnan")) ~= cfgId || ...
                ~ismember("elevator_cmd_norm", string(T.Properties.VariableNames))
            continue;
        end
        mask = true(height(T),1);
        if ismember("elevator_excitation", string(T.Properties.VariableNames))
            exc = abs(double(T.elevator_excitation));
            firstActive = find(exc > 1.0e-7, 1, "first");
            if ~isempty(firstActive) && firstActive > 3
                mask = false(height(T),1);
                mask(1:firstActive-1) = true;
            end
        end
        v = double(T.elevator_cmd_norm(mask));
        v = v(isfinite(v));
        if ~isempty(v), vals(end+1,1) = median(v, "omitnan"); end %#ok<AGROW>
    catch
    end
end
if ~isempty(vals)
    physicalNom = median(vals, "omitnan");
else
    physicalNom = double(opts.FallbackPhysicalElevatorNominal);
    warning("AirdropX:AutoMPC200:PhysicalNominalFallback", ...
        "Could not read v11 ID CSV physical nominal; using 200 m probe fallback %.6f.", physicalNom);
end
end

function local_plot(T, eNom, tNom, outFile, plotTitle)
fig = figure('Visible','off','Color','w','Position',[100 100 1300 1000]);
tl = tiledlayout(7,1,'Padding','compact','TileSpacing','compact');
t = T.time_s;
nexttile; plot(t,T.altitude_m); hold on; plot(t,T.target_altitude_m,'--'); grid on; ylabel('h m');
nexttile; plot(t,T.airspeed_mps); hold on; plot(t,T.target_airspeed_mps,'--'); grid on; ylabel('Va');
nexttile; plot(t,T.pitch_deg); grid on; ylabel('pitch');
nexttile; plot(t,T.vz_up_mps); yline(0,'--'); grid on; ylabel('vz');
nexttile; plot(t,T.q_dps); yline(0,'--'); grid on; ylabel('q');
nexttile; plot(t,T.elevator_cmd_norm); hold on; yline(eNom,'--'); grid on; ylabel('elev physical');
nexttile; plot(t,T.throttle_norm); hold on; yline(tNom,'--'); grid on; ylabel('throttle'); xlabel('time s');
title(tl, plotTitle, 'Interpreter','none');
exportgraphics(fig,outFile,'Resolution',160);
close(fig);
end

function x = local_col(T, name)
if ismember(string(name), string(T.Properties.VariableNames))
    x = double(T.(char(name))(:));
else
    x = NaN(height(T),1);
end
end

function v = local_rms(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = sqrt(mean(x.^2)); end
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
paths = struct("projectRoot",char(projectRoot), "matlabDir",char(matlabDir), ...
    "mpcDir",char(fullfile(matlabDir,"mpc")), ...
    "autoDir",char(fullfile(matlabDir,"mpc_auto")), ...
    "sfuncDir",char(fullfile(matlabDir,"sfunc_jsbsim")));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.IdentifiedMat = "matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.OutputRoot = "matlab/results/mpc_auto_mpc_200m_autotune_v15";
opts.TargetAltitudeM = 200.0;
opts.TargetAirspeedMps = 50.0;

opts.PhysicalElevatorNominal = NaN;
opts.FallbackPhysicalElevatorNominal = -0.338803334723381;
opts.ExpectedPhysicalElevatorNominal = -0.338803334723381;
opts.PhysicalNominalWarningTolerance = 0.02;

opts.CalibrationStopTimeS = 0.5;
opts.CalibrationUseUntilS = 0.25;
opts.PreflightStopTimeS = 20.0;
opts.PreflightMaxAltitudeRmsM = 1.0;
opts.OptimizationStopTimeS = 35.0;
opts.FinalStopTimeS = 60.0;
opts.ScoreStartTimeS = 8.0;
opts.TailWindowS = 6.0;
opts.MpcEnableTimeS = 3.0;

opts.MaxObjectiveEvaluations = 24;
opts.UseParallel = false;
opts.BayesoptVerbose = 1;
opts.ReuseHistory = true;
opts.ResumeBestPointCount = 5;
opts.HardFailScore = 1.0e8;

opts.FixedAirspeedWeight = 5.0;
opts.FixedPitchWeight = 0.03;
opts.MVWeights = [0.20 0.15];
opts.MVRateWeights = [8.0 4.0];
opts.ElevatorDeviationLimit = 0.035;
opts.ThrottleDeviationLimit = 0.060;
opts.ElevatorDeviationRateLimit = 0.006;
opts.ThrottleDeviationRateLimit = 0.010;

opts.TrustAirspeedMps = 4.0;
opts.TrustPitchDeg = 4.0;
opts.TrustVzMps = 2.5;
opts.TrustQDps = 4.0;

opts.HardMaxAltitudeErrorM = 50.0;
opts.HardMaxQRmsDps = 10.0;
opts.HardMaxAbsPitchDeg = 30.0;
opts.MaxBridgeError = 0.01;

opts.PassAltitudeRmsM = 1.0;
opts.PassAltitudeMaxM = 2.0;
opts.PassAltitudeDriftM = 1.0;
opts.PassAirspeedRmsMps = 1.0;
opts.PassVzRmsMps = 0.7;
opts.PassQRmsDps = 1.0;
opts.PassTailAltitudeErrorM = 1.0;
opts.PassTailVzMps = 0.35;

if mod(numel(varargin),2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name) = varargin{i+1};
end
end
