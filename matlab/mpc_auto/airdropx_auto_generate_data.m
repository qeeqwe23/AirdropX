function result = airdropx_auto_generate_data(varargin)
%AIRDROPX_AUTO_GENERATE_DATA Generate clean fixed-configuration JSBSim ID CSVs.
%
% v11 adds a measured pre-excitation baseline and refuses to keep a run whose
% baseline is still drifting.  If necessary the excitation start is delayed
% and the run is repeated.  This is especially important for cfg2+ after a
% long payload-transition phugoid.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
rng(double(opts.Seed));

trimBank = opts.TrimBank;
if isempty(trimBank)
    trimBank = airdropx_auto_default_trim_bank( ...
        "TargetAltitudeM", opts.TargetAltitudeM, ...
        "TargetAirspeedMps", opts.TargetAirspeedMps);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", ...
        "mpc_auto_data_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot), mkdir(outputRoot); end

runs = struct("config_id", {}, "run_id", {}, "csv", {}, ...
    "excitation_start_s", {}, "baseline_pass", {}, "settle_retry", {});

for cfgId = double(opts.ConfigIds(:)).'
    trim = trimBank(cfgId + 1);
    initialFlightPathDeg = local_trim_field(trim, "initial_flight_path_deg", opts.InitialFlightPathDeg);
    configReachS = local_config_reach_time(cfgId, opts);
    defaultSettleS = double(opts.ConfigSettleBaseS) + ...
        double(opts.ConfigSettlePerConfigS) * max(0, cfgId);
    settleS = max(defaultSettleS, local_trim_field(trim, "id_settle_s", defaultSettleS));
    effectiveMaxSettleRetries = round(double(opts.MaxSettleRetries));
    if logical(opts.AdaptiveSettleRecoveryEnabled)
        stepS = max(1e-6, double(opts.SettleRetryStepS));
        adaptiveRetries = ceil(max(0, double(opts.AdaptiveSettleRecoveryMaxS) - settleS) / stepS);
        effectiveMaxSettleRetries = max(effectiveMaxSettleRetries, adaptiveRetries);
    end

    for r = 1:double(opts.RunsPerConfig)
        runId = sprintf("cfg%d_run%03d", cfgId, r);
        runDir = fullfile(outputRoot, sprintf("cfg%d", cfgId), runId);
        seed = double(opts.Seed) + 1000 * cfgId + r;

        baselineOk = false;
        one = [];
        usedExcitationStartS = NaN;
        usedRetry = NaN;
        % Keep the same initial condition across settle retries.  A retry must
        % test the effect of waiting longer, not silently draw a new/easier IC.
        initialAirspeedMps = trim.airspeed_mps + opts.InitialAirspeedSpreadMps * local_unit_rand();
        initialAltitudeM = opts.IdentificationAltitudeM + opts.InitialAltitudeSpreadM * local_unit_rand();

        % v20: cfg1+ reaches the target through real payload drops.  The
        % aircraft therefore starts in cfg0, so its release pitch/gamma must
        % come from cfg0 rather than the target cfg trim.  The target trim is
        % only applied after its configuration is reached by the base-command
        % schedule in run_id_experiment.
        releasePitchDeg = trim.pitch_deg;
        releaseGammaDeg = initialFlightPathDeg;
        if cfgId > 0 && ~isempty(opts.PreparationTrimBank) && logical(opts.UsePreparationTrimSchedule)
            try
                releasePitchDeg = local_trim_field(opts.PreparationTrimBank(1), "pitch_deg", releasePitchDeg);
                releaseGammaDeg = local_trim_field(opts.PreparationTrimBank(1), "initial_flight_path_deg", 0.0);
            catch
            end
        end
        initialPitchDeg = releasePitchDeg + opts.InitialPitchSpreadDeg * local_unit_rand();
        initialGammaDeg = releaseGammaDeg + opts.InitialFlightPathSpreadDeg * local_unit_rand();

        for retry = 0:effectiveMaxSettleRetries
            extraSettleS = retry * double(opts.SettleRetryStepS);
            excitationStartS = max(double(opts.RecordStartS), configReachS + settleS + extraSettleS);
            exportStartS = max(configReachS, excitationStartS - double(opts.BaselineDurationS));
            runStopTimeS = max(double(opts.StopTimeS), ...
                excitationStartS + double(opts.IdentificationDurationS));

            fprintf(['Auto MPC data run %s attempt %d (cfg reach %.2fs, baseline starts %.2fs, ' ...
                'ID starts %.2fs, stop %.2fs)\n'], ...
                runId, retry + 1, configReachS, exportStartS, excitationStartS, runStopTimeS);

            one = airdropx_auto_run_id_experiment( ...
                "ProjectRoot", paths.projectRoot, "Model", opts.Model, "OutputRoot", runDir, ...
                "RunId", runId, "ConfigId", cfgId, "Trim", trim, ...
                "StopTimeS", runStopTimeS, "RecordStartS", excitationStartS, ...
                "ExportStartS", exportStartS, "ExcitationStartS", excitationStartS, "Seed", seed, ...
                "PrepDropStartS", opts.PrepDropStartS, "PrepDropIntervalS", opts.PrepDropIntervalS, ...
                "InitialAirspeedMps", initialAirspeedMps, ...
                "ReferenceMassKg", opts.ReferenceMassKg, "CargoMassKg", opts.CargoMassKg, ...
                "InitialAltitudeM", initialAltitudeM, ...
                "InitialPitchDeg", initialPitchDeg, ...
                "InitialFlightPathDeg", initialGammaDeg, ...
                "ElevatorAmplitude", opts.ElevatorAmplitude, "ThrottleAmplitude", opts.ThrottleAmplitude, ...
                "ElevatorHoldTimeRangeS", opts.ElevatorHoldTimeRangeS, ...
                "ThrottleHoldTimeRangeS", opts.ThrottleHoldTimeRangeS, ...
                "PreparationTrimBank", opts.PreparationTrimBank, ...
                "UsePreparationTrimSchedule", opts.UsePreparationTrimSchedule, ...
                "DirectIdMode", true);

            metrics = local_baseline_metrics(one.timeseries, excitationStartS, trim, opts);
            fprintf(['  baseline cfg%d: V=%.3f (err %.3f), pitch=%.3f (err %.3f), ' ...
                'vz=%.3f, q=%.3f, hSlope=%.3f, VaSlope=%.4f -> %s [%s]\n'], ...
                cfgId, metrics.V, metrics.Verr, metrics.pitch, metrics.pitchErr, ...
                metrics.vz, metrics.q, metrics.hSlope, metrics.vaSlope, string(metrics.pass), string(metrics.failReason));

            if metrics.pass
                baselineOk = true;
                usedExcitationStartS = excitationStartS;
                usedRetry = retry;
                break;
            end

            if retry < effectiveMaxSettleRetries
                fprintf("  cfg%d baseline not settled; delaying excitation by %.1f s and retrying.\n", ...
                    cfgId, double(opts.SettleRetryStepS));
            end
        end

        if ~baselineOk && logical(opts.RequireStableBaseline)
            error("AirdropX:AutoMPC:IdBaselineNotSettled", ...
                "Config %d run %d failed the pre-excitation baseline gate after %d attempts. " + ...
                "Last baseline: Verr=%.3f, pitchErr=%.3f, vz=%.3f, q=%.3f, hSlope=%.4f, VaSlope=%.4f; failed=%s.", ...
                cfgId, r, effectiveMaxSettleRetries + 1, metrics.Verr, metrics.pitchErr, metrics.vz, metrics.q, ...
                metrics.hSlope, metrics.vaSlope, char(string(metrics.failReason)));
        end

        runs(end + 1) = struct( ... %#ok<AGROW>
            "config_id", cfgId, "run_id", string(runId), "csv", string(one.timeseries_csv), ...
            "excitation_start_s", usedExcitationStartS, "baseline_pass", baselineOk, ...
            "settle_retry", usedRetry);
    end
end

manifest = struct2table(runs);
manifestFile = fullfile(outputRoot, "manifest.csv");
writetable(manifest, manifestFile);
result = struct("output_root", outputRoot, "manifest_csv", string(manifestFile), ...
    "manifest", manifest, "trim_bank", trimBank);
end

function metrics = local_baseline_metrics(T, excitationStartS, trim, opts)
metrics = struct("pass", false, "V", NaN, "Verr", NaN, "pitch", NaN, ...
    "pitchErr", NaN, "vz", NaN, "q", NaN, "hSlope", NaN, "vaSlope", NaN, "failReason", "");
if isempty(T) || height(T) < 5, return; end

t = double(T.time_s);
startS = excitationStartS - double(opts.BaselineDurationS);
mask = isfinite(t) & t >= startS & t < excitationStartS - 1e-9;
if ismember("config_id", string(T.Properties.VariableNames))
    mask = mask & round(double(T.config_id)) == double(trim.config_id);
end
if nnz(mask) < double(opts.BaselineMinSamples), return; end

metrics.V = local_median(T.airspeed_mps(mask));
metrics.pitch = local_median(T.pitch_deg(mask));
metrics.vz = local_median(T.vz_up_mps(mask));
metrics.q = local_median(T.q_dps(mask));
metrics.Verr = metrics.V - double(trim.airspeed_mps);
metrics.pitchErr = metrics.pitch - double(trim.pitch_deg);
metrics.hSlope = local_slope(t(mask), double(T.altitude_m(mask)));
metrics.vaSlope = local_slope(t(mask), double(T.airspeed_mps(mask)));

metrics.pass = ...
    abs(metrics.Verr) <= double(opts.BaselineMaxAirspeedErrorMps) && ...
    abs(metrics.pitchErr) <= double(opts.BaselineMaxPitchErrorDeg) && ...
    abs(metrics.vz) <= double(opts.BaselineMaxAbsVzMps) && ...
    abs(metrics.q) <= double(opts.BaselineMaxAbsQDps) && ...
    abs(metrics.hSlope) <= double(opts.BaselineMaxHeightSlopeMps) && ...
    abs(metrics.vaSlope) <= double(opts.BaselineMaxAirspeedSlopeMps2);
metrics.failReason = local_baseline_fail_reason(metrics, opts);
end

function s = local_baseline_fail_reason(m, o)
parts = strings(0,1);
if ~isfinite(m.Verr) || abs(m.Verr)>double(o.BaselineMaxAirspeedErrorMps), parts(end+1)="Va_error"; end %#ok<AGROW>
if ~isfinite(m.pitchErr) || abs(m.pitchErr)>double(o.BaselineMaxPitchErrorDeg), parts(end+1)="pitch_error"; end %#ok<AGROW>
if ~isfinite(m.vz) || abs(m.vz)>double(o.BaselineMaxAbsVzMps), parts(end+1)="vz"; end %#ok<AGROW>
if ~isfinite(m.q) || abs(m.q)>double(o.BaselineMaxAbsQDps), parts(end+1)="q"; end %#ok<AGROW>
if ~isfinite(m.hSlope) || abs(m.hSlope)>double(o.BaselineMaxHeightSlopeMps), parts(end+1)="h_slope"; end %#ok<AGROW>
if ~isfinite(m.vaSlope) || abs(m.vaSlope)>double(o.BaselineMaxAirspeedSlopeMps2), parts(end+1)="Va_slope"; end %#ok<AGROW>
if isempty(parts), s="PASS"; else, s=strjoin(parts,"|"); end
end

function value = local_median(x)
x = double(x(:)); x = x(isfinite(x));
if isempty(x), value = NaN; else, value = median(x); end
end

function value = local_slope(t, y)
t = double(t(:)); y = double(y(:));
mask = isfinite(t) & isfinite(y);
if nnz(mask) < 3, value = NaN; return; end
t0 = t(find(mask,1,"first"));
p = polyfit(t(mask) - t0, y(mask), 1);
value = p(1);
end

function t = local_config_reach_time(cfgId, opts)
cfgId = max(0, round(double(cfgId)));
if cfgId == 0
    t = 0.0;
else
    t = double(opts.PrepDropStartS) + double(opts.PrepDropIntervalS) * max(0, cfgId - 1);
end
end

function value = local_trim_field(trim, name, defaultValue)
if isfield(trim, name) && ~isempty(trim.(name)) && isfinite(double(trim.(name)))
    value = double(trim.(name));
else
    value = double(defaultValue);
end
end

function x = local_unit_rand()
x = -1.0 + 2.0 * rand();
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
paths.projectRoot = char(projectRoot);
paths.matlabDir = char(matlabDir);
paths.mpcDir = char(fullfile(matlabDir, "mpc"));
paths.autoDir = char(fullfile(matlabDir, "mpc_auto"));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.Model = "airdropx_mpc_id";
opts.OutputRoot = "";
opts.TrimBank = [];
opts.ConfigIds = (0:4).';
opts.RunsPerConfig = 5;
opts.StopTimeS = 30.0;
opts.RecordStartS = 8.0;
opts.Seed = 42;
opts.PrepDropStartS = 1.0;
opts.PrepDropIntervalS = 2.0;
opts.ConfigSettleBaseS = 4.0;
opts.ConfigSettlePerConfigS = 2.0;
% 22 s was short compared with the longitudinal/phugoid time scale seen in
% cfg2.  Use a longer local ID record after the baseline is proven stable.
opts.IdentificationDurationS = 36.0;
opts.BaselineDurationS = 4.0;
opts.BaselineMinSamples = 40;
opts.MaxSettleRetries = 2;
opts.SettleRetryStepS = 10.0;
% Optional generic recovery used by v32 clean-slate training. When enabled,
% the exact same initial condition is held while the pre-ID settle period is
% extended up to the hard maximum before declaring the trim unsuitable.
opts.AdaptiveSettleRecoveryEnabled = false;
opts.AdaptiveSettleRecoveryMaxS = 90.0;
opts.RequireStableBaseline = true;
opts.BaselineMaxAirspeedErrorMps = 1.0;
opts.BaselineMaxPitchErrorDeg = 1.25;
opts.BaselineMaxAbsVzMps = 0.30;
opts.BaselineMaxAbsQDps = 0.30;
opts.BaselineMaxHeightSlopeMps = 0.30;
opts.BaselineMaxAirspeedSlopeMps2 = 0.15;
% v20: preserve cfg0 -> ... -> cfgN base trims during preparatory drops.
opts.PreparationTrimBank = [];
opts.UsePreparationTrimSchedule = true;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.IdentificationAltitudeM = 200.0;
opts.InitialFlightPathDeg = 0.0;
% Keep initial condition perturbations small; actuator excitation should be the
% deliberate source of ID dynamics.
opts.InitialAltitudeSpreadM = 0.25;
opts.InitialAirspeedSpreadMps = 0.50;
opts.InitialPitchSpreadDeg = 0.25;
opts.InitialFlightPathSpreadDeg = 0.10;
opts.ElevatorAmplitude = 0.03;
opts.ThrottleAmplitude = 0.06;
opts.ElevatorHoldTimeRangeS = [0.40 1.60];
opts.ThrottleHoldTimeRangeS = [0.80 3.00];
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
