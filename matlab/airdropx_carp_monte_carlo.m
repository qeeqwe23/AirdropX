function mc = airdropx_carp_monte_carlo(logs, varargin)
%AIRDROPX_CARP_MONTE_CARLO Monte Carlo impact scatter for CARP.
%
% Usage:
%   mc = airdropx_carp_monte_carlo(out.logsout)
%   mc = airdropx_carp_monte_carlo(out.logsout, "Mode", "fourdrop")

cfg = airdropx_sim_params();
p = inputParser;
addParameter(p, "Mode", "single");
addParameter(p, "Samples", cfg.monte_carlo.samples);
addParameter(p, "TargetN", cfg.carp.target_n_m);
addParameter(p, "TargetE", cfg.carp.target_e_m);
addParameter(p, "Seed", cfg.monte_carlo.seed);
addParameter(p, "AltStdM", cfg.monte_carlo.alt_std_m);
addParameter(p, "AirspeedStdMps", cfg.monte_carlo.airspeed_std_mps);
addParameter(p, "HeadingStdDeg", cfg.monte_carlo.heading_std_deg);
addParameter(p, "WindStdMps", cfg.monte_carlo.wind_std_mps);
parse(p, varargin{:});

switch lower(string(p.Results.Mode))
    case {"single", "last", "carp"}
        mc = local_single_release(logs, p.Results);
    case {"fourdrop", "4drop", "drops"}
        mc = local_four_drop(logs, p.Results, cfg);
    otherwise
        error("Unknown Monte Carlo mode: %s", string(p.Results.Mode));
end
end

function mc = local_single_release(logs, opts)
nSamples = max(1, round(double(opts.Samples)));
targetN = double(opts.TargetN);
targetE = double(opts.TargetE);

[~, relN] = local_signal(logs, "actual_release_n_m");
[~, relE] = local_signal(logs, "actual_release_e_m");
[~, relH] = local_signal(logs, "actual_release_alt_m");
[~, airspeed] = local_signal(logs, "release_airspeed_mps");
[~, heading] = local_signal(logs, "release_heading_deg");
[~, windN] = local_signal(logs, "release_wind_n_mps");
[~, windE] = local_signal(logs, "release_wind_e_mps");

releaseN = local_last_nonzero(relN);
releaseE = local_last_nonzero(relE);
releaseH = local_last_nonzero(relH);
v0 = local_last_finite(airspeed);
hdg0 = local_last_finite(heading);
wn0 = local_last_finite(windN);
we0 = local_last_finite(windE);

if ~isfinite(releaseN) || ~isfinite(releaseE) || ~isfinite(releaseH)
    error("No actual release point found. Log actual_release_n_m/e_m/alt_m from CARP_CEP.");
end

[v0, hdg0, wn0, we0] = local_fill_state_defaults(logs, v0, hdg0, wn0, we0);
[impactE, impactN, ptsE, ptsN] = local_sample_impacts( ...
    nSamples, double(opts.Seed), releaseN, releaseE, releaseH, v0, hdg0, wn0, we0, ...
    targetN, targetE, opts);

mc = local_mc_summary(ptsE, ptsN);
mc.target_n_m = targetN;
mc.target_e_m = targetE;
mc.release_n_m = releaseN;
mc.release_e_m = releaseE;
mc.release_alt_m = releaseH;
mc.impact_e_m = impactE;
mc.impact_n_m = impactN;
mc.offset_e_m = ptsE;
mc.offset_n_m = ptsN;
mc.samples = nSamples;
mc.mode = "single";

fprintf("=== AirdropX CARP Monte Carlo ===\n");
fprintf("samples       : %d\n", nSamples);
fprintf("release N/E/H : %.2f / %.2f / %.2f\n", releaseN, releaseE, releaseH);
fprintf("CEP50         : %.3f m\n", mc.cep50_m);
fprintf("CEP95         : %.3f m\n", mc.cep95_m);
fprintf("mean offset E/N: %.3f / %.3f m\n", mc.mean_e_m, mc.mean_n_m);
end

function mc = local_four_drop(logs, opts, cfg)
nSamples = max(1, round(double(opts.Samples)));
targetN = double(opts.TargetN);
targetE = double(opts.TargetE);

[tDrop, dropCount] = local_signal(logs, "drop_count");
[tN, posN] = local_signal(logs, "pos_n_m");
[tE, posE] = local_signal(logs, "pos_e_m");
[tH, alt] = local_signal(logs, "altitude_m");
[tV, airspeed] = local_signal(logs, "airspeed_mps");
[tPsi, heading] = local_signal(logs, "heading_deg");
[tWn, windN] = local_signal(logs, "wind_n_mps");
[tWe, windE] = local_signal(logs, "wind_e_mps");

if isempty(dropCount)
    error("Could not find drop_count in logs.");
end
if isempty(posN) || isempty(posE)
    error("Log pos_n_m and pos_e_m from S-Function outputs 12 and 13.");
end

edgeIdx = find([0; diff(dropCount(:))] > 0.5);
if isempty(edgeIdx)
    error("No drop_count rising edges found.");
end

nDrops = numel(edgeIdx);
batches = repmat(local_empty_batch(), nDrops, 1);
allE = [];
allN = [];

for k = 1:nDrops
    tk = tDrop(edgeIdx(k));
    releaseN = local_sample_at(tN, posN, tk);
    releaseE = local_sample_at(tE, posE, tk);
    releaseH = local_sample_at(tH, alt, tk);
    v0 = local_sample_at(tV, airspeed, tk);
    hdg0 = local_sample_at(tPsi, heading, tk);
    wn0 = local_sample_at(tWn, windN, tk);
    we0 = local_sample_at(tWe, windE, tk);

    if ~isfinite(releaseH), releaseH = cfg.control.target_altitude_m; end
    if ~isfinite(v0), v0 = cfg.ballistics.calibration_airspeed_mps; end
    if ~isfinite(hdg0), hdg0 = 0.0; end
    if ~isfinite(wn0), wn0 = 0.0; end
    if ~isfinite(we0), we0 = 0.0; end

    [impactE, impactN, ptsE, ptsN] = local_sample_impacts( ...
        nSamples, double(opts.Seed) + 1000 * k, releaseN, releaseE, releaseH, ...
        v0, hdg0, wn0, we0, targetN, targetE, opts);

    batchSummary = local_mc_summary(ptsE, ptsN);
    batches(k).drop_index = k;
    batches(k).release_time_s = tk;
    batches(k).release_n_m = releaseN;
    batches(k).release_e_m = releaseE;
    batches(k).release_alt_m = releaseH;
    batches(k).offset_e_m = ptsE;
    batches(k).offset_n_m = ptsN;
    batches(k).impact_e_m = impactE;
    batches(k).impact_n_m = impactN;
    batches(k).radial_error_m = batchSummary.radial_error_m;
    batches(k).cep50_m = batchSummary.cep50_m;
    batches(k).cep95_m = batchSummary.cep95_m;
    batches(k).mean_e_m = batchSummary.mean_e_m;
    batches(k).mean_n_m = batchSummary.mean_n_m;

    allE = [allE; ptsE]; %#ok<AGROW>
    allN = [allN; ptsN]; %#ok<AGROW>
end

mc = local_mc_summary(allE, allN);
mc.target_n_m = targetN;
mc.target_e_m = targetE;
mc.batches = batches;
mc.offset_e_m = allE;
mc.offset_n_m = allN;
mc.samples_per_drop = nSamples;
mc.samples = numel(mc.radial_error_m);
mc.drop_count = nDrops;
mc.mode = "fourdrop";

fprintf("=== AirdropX CARP Monte Carlo 4-drop ===\n");
fprintf("drops              : %d\n", nDrops);
fprintf("samples/drop       : %d\n", nSamples);
fprintf("combined CEP50     : %.3f m\n", mc.cep50_m);
fprintf("combined CEP95     : %.3f m\n", mc.cep95_m);
for k = 1:nDrops
    fprintf("drop %d t=%.3f release N/E/H=%.2f/%.2f/%.2f CEP50=%.2f mean E/N=%.2f/%.2f\n", ...
        k, batches(k).release_time_s, batches(k).release_n_m, batches(k).release_e_m, ...
        batches(k).release_alt_m, batches(k).cep50_m, batches(k).mean_e_m, batches(k).mean_n_m);
end
end

function [impactE, impactN, ptsE, ptsN] = local_sample_impacts(nSamples, seed, releaseN, releaseE, releaseH, v0, hdg0, wn0, we0, targetN, targetE, opts)
rng(seed, "twister");

ptsE = zeros(nSamples, 1);
ptsN = zeros(nSamples, 1);
impactE = zeros(nSamples, 1);
impactN = zeros(nSamples, 1);

for i = 1:nSamples
    h = max(0.1, releaseH + double(opts.AltStdM) * randn());
    v = max(1.0, v0 + double(opts.AirspeedStdMps) * randn());
    hdg = hdg0 + double(opts.HeadingStdDeg) * randn();
    wn = wn0 + double(opts.WindStdMps) * randn();
    we = we0 + double(opts.WindStdMps) * randn();

    r = airdropx_carp_release_point(targetE, targetN, h, v, we, wn, 0.0, hdg);
    impactN(i) = releaseN + r.ballistic_n_m + r.wind_drift_n_m;
    impactE(i) = releaseE + r.ballistic_e_m + r.wind_drift_e_m;
    ptsN(i) = impactN(i) - targetN;
    ptsE(i) = impactE(i) - targetE;
end
end

function s = local_mc_summary(ptsE, ptsN)
dist = hypot(ptsE, ptsN);
distSorted = sort(dist);
idx50 = max(1, min(numel(distSorted), ceil(0.50 * numel(distSorted))));
idx95 = max(1, min(numel(distSorted), ceil(0.95 * numel(distSorted))));

s = struct();
s.radial_error_m = dist;
s.cep50_m = distSorted(idx50);
s.cep95_m = distSorted(idx95);
s.mean_e_m = mean(ptsE);
s.mean_n_m = mean(ptsN);
end

function [v0, hdg0, wn0, we0] = local_fill_state_defaults(logs, v0, hdg0, wn0, we0)
cfg = airdropx_sim_params();
if ~isfinite(v0)
    [~, airspeed] = local_signal(logs, "airspeed_mps");
    v0 = local_last_finite(airspeed);
end
if ~isfinite(hdg0)
    [~, heading] = local_signal(logs, "heading_deg");
    hdg0 = local_last_finite(heading);
end
if ~isfinite(wn0)
    [~, windN] = local_signal(logs, "wind_n_mps");
    wn0 = local_last_finite(windN);
end
if ~isfinite(we0)
    [~, windE] = local_signal(logs, "wind_e_mps");
    we0 = local_last_finite(windE);
end
if ~isfinite(v0), v0 = cfg.ballistics.calibration_airspeed_mps; end
if ~isfinite(hdg0), hdg0 = 0.0; end
if ~isfinite(wn0), wn0 = 0.0; end
if ~isfinite(we0), we0 = 0.0; end
end

function b = local_empty_batch()
b = struct('drop_index', 0, 'release_time_s', NaN, 'release_n_m', NaN, ...
    'release_e_m', NaN, 'release_alt_m', NaN, 'offset_e_m', [], ...
    'offset_n_m', [], 'impact_e_m', [], 'impact_n_m', [], ...
    'radial_error_m', [], 'cep50_m', NaN, 'cep95_m', NaN, ...
    'mean_e_m', NaN, 'mean_n_m', NaN);
end

function v = local_sample_at(t, y, tq)
if isempty(t) || isempty(y)
    v = NaN;
    return;
end
t = t(:);
y = y(:);
[~, idx] = min(abs(t - tq));
v = y(idx);
end

function v = local_last_nonzero(x)
if isempty(x), v = NaN; return; end
idx = find(abs(x(:)) > 0, 1, "last");
if isempty(idx), v = NaN; else, v = x(idx); end
end

function v = local_last_finite(x)
if isempty(x), v = NaN; return; end
idx = find(isfinite(x(:)), 1, "last");
if isempty(idx), v = NaN; else, v = x(idx); end
end

function [t, y] = local_signal(logs, name)
t = [];
y = [];
if isa(logs, "Simulink.SimulationOutput")
    try
        [t, y] = local_signal(logs.logsout, name);
    catch
    end
    return;
end
if isa(logs, "Simulink.SimulationData.Dataset")
    for i = 1:logs.numElements
        el = logs.get(i);
        if string(el.Name) == string(name)
            ts = el.Values;
            t = ts.Time(:);
            y = squeeze(ts.Data);
            y = y(:);
            return;
        end
    end
    return;
end
if isstruct(logs)
    if isfield(logs, "time"), t = logs.time(:); end
    if isfield(logs, name)
        y = logs.(name)(:);
        if isempty(t), t = (0:numel(y)-1).'; end
    end
end
end
