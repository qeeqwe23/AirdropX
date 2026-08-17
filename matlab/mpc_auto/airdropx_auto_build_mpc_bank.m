function bank = airdropx_auto_build_mpc_bank(varargin)
%AIRDROPX_AUTO_BUILD_MPC_BANK Build MPC bank from identified deviation models.
%
% v13 default behavior is explicit deviation-coordinate MPC:
%   plant model:    delta-u -> delta-y
%   MPC move:       delta-u
%   bridge command: physical elevator nominal + delta-u - hidden JSBSim trim
%
% This avoids mixing three different coordinates:
%   1) identified input deviation,
%   2) absolute/nominal MPC MV,
%   3) JSBSim S-function external elevator delta.

opts = local_options(varargin{:});
learned = opts.Identified;
if isempty(learned)
    S = load(opts.IdentifiedMat);
    if isfield(S, "result")
        learned = S.result;
    else
        error("IdentifiedMat must contain variable result.");
    end
end

plantBank = learned.plant_bank;
trimBank = learned.trim_bank;
Ts = double(opts.Ts);
if isempty(Ts) || ~isfinite(Ts)
    Ts = double(learned.Ts);
end

physicalElevatorNominals = local_physical_elevator_nominals(opts, learned, trimBank);
controllers = cell(5, 1);
configIds = unique(round(double(opts.ConfigIds(:).')), "stable");
configIds = configIds(configIds >= 0 & configIds <= 4);
if isempty(configIds), configIds = 0:4; end

for k = configIds + 1
    plant = plantBank{k};
    if isempty(plant)
        continue;
    end
    plant = ss(plant);
    if plant.Ts == 0
        plant = c2d(plant, Ts);
    end

    ctrl = mpc(plant, Ts, double(opts.PredictionHorizon), double(opts.ControlHorizon));

    if string(opts.InputCoordinateMode) == "deviation_physical"
        % The identified model was built from run-centered Udev/Ydev. Keep the
        % controller in exactly those coordinates instead of assigning an
        % arbitrary absolute nominal to the identified state-space model.
        ctrl.Model.Nominal.U = zeros(2,1);
        ctrl.Model.Nominal.Y = zeros(5,1);

        ctrl.MV(1).Min = -double(opts.ElevatorDeviationLimit);
        ctrl.MV(1).Max =  double(opts.ElevatorDeviationLimit);
        ctrl.MV(2).Min = -double(opts.ThrottleDeviationLimit);
        ctrl.MV(2).Max =  double(opts.ThrottleDeviationLimit);
        ctrl.MV(1).RateMin = -double(opts.ElevatorDeviationRateLimit);
        ctrl.MV(1).RateMax =  double(opts.ElevatorDeviationRateLimit);
        ctrl.MV(2).RateMin = -double(opts.ThrottleDeviationRateLimit);
        ctrl.MV(2).RateMax =  double(opts.ThrottleDeviationRateLimit);
    else
        ctrl.Model.Nominal.U = [trimBank(k).elevator_cmd; trimBank(k).throttle_cmd];
        ctrl.Model.Nominal.Y = [trimBank(k).altitude_m; trimBank(k).airspeed_mps; trimBank(k).pitch_deg; ...
            local_trim_field(trimBank(k), "vz_up_mps", 0.0); local_trim_field(trimBank(k), "q_dps", 0.0)];
        ctrl.MV(1).Min = double(opts.ElevatorMin);
        ctrl.MV(1).Max = double(opts.ElevatorMax);
        ctrl.MV(2).Min = double(opts.ThrottleMin);
        ctrl.MV(2).Max = double(opts.ThrottleMax);
        ctrl.MV(1).RateMin = double(opts.ElevatorRateMin);
        ctrl.MV(1).RateMax = double(opts.ElevatorRateMax);
        ctrl.MV(2).RateMin = double(opts.ThrottleRateMin);
        ctrl.MV(2).RateMax = double(opts.ThrottleRateMax);
    end

    ctrl.Weights.OutputVariables = double(opts.OutputWeights(:)).';
    ctrl.Weights.ManipulatedVariables = double(opts.MVWeights(:)).';
    ctrl.Weights.ManipulatedVariablesRate = double(opts.MVRateWeights(:)).';
    for j = 1:5
        ctrl.OV(j).ScaleFactor = double(opts.OutputScaleFactors(j));
    end
    for j = 1:2
        ctrl.MV(j).ScaleFactor = double(opts.MVScaleFactors(j));
    end
    ctrl.MV(1).RateMinECR = double(opts.MVRateECR);
    ctrl.MV(1).RateMaxECR = double(opts.MVRateECR);
    ctrl.MV(2).RateMinECR = double(opts.MVRateECR);
    ctrl.MV(2).RateMaxECR = double(opts.MVRateECR);

    % For the first real-JSBSim smoke test, disable default integral output
    % disturbance states. They are useful later for offset-free tracking, but
    % with a local identified model they can drive estimator wind-up before the
    % plant/bridge mapping has been proven.
    if logical(opts.DisableOutputDisturbanceModel)
        try
            setoutdist(ctrl, 'model', tf(zeros(5,1)));
        catch ME
            warning("AirdropX:AutoMPC:OutDistDisableFailed", ...
                "Could not disable MPC output-disturbance model: %s", ME.message);
        end
    end

    controllers{k} = ctrl;
end

mpcMeta = struct();
mpcMeta.version = 17;
mpcMeta.input_coordinate_mode = string(opts.InputCoordinateMode);
mpcMeta.physical_elevator_nominals = physicalElevatorNominals(:);
mpcMeta.throttle_nominals = arrayfun(@(s) double(s.throttle_cmd), trimBank(:));
mpcMeta.elevator_deviation_limit = double(opts.ElevatorDeviationLimit);
mpcMeta.throttle_deviation_limit = double(opts.ThrottleDeviationLimit);
mpcMeta.elevator_deviation_rate_limit = double(opts.ElevatorDeviationRateLimit);
mpcMeta.throttle_deviation_rate_limit = double(opts.ThrottleDeviationRateLimit);
mpcMeta.disable_output_disturbance_model = logical(opts.DisableOutputDisturbanceModel);

bank = struct();
bank.controllers = controllers;
bank.plant_bank = plantBank;
bank.trim_bank = trimBank;
bank.mpc_meta = mpcMeta;
bank.options = opts;

out = struct();
out.plant_bank = plantBank;
out.trim_bank = trimBank;
out.controllers = controllers;
out.mpc_meta = mpcMeta;
for k = 1:5
    out.(sprintf("MPC%d", k - 1)) = controllers{k};
    out.(sprintf("Plant%d", k - 1)) = plantBank{k};
end

if strlength(string(opts.OutputMat)) > 0
    save(opts.OutputMat, "-struct", "out");
end
end

function physical = local_physical_elevator_nominals(opts, learned, trimBank)
physical = NaN(5,1);
if ~isempty(opts.PhysicalElevatorNominals)
    values = double(opts.PhysicalElevatorNominals(:));
    physical(1:min(5,numel(values))) = values(1:min(5,numel(values)));
end

if any(~isfinite(physical)) && logical(opts.DerivePhysicalElevatorNominalsFromIdData)
    files = strings(0,1);
    try
        if isfield(learned, "data") && isfield(learned.data, "csv_files")
            files = string(learned.data.csv_files(:));
        end
    catch
    end
    for cfg = 0:4
        if isfinite(physical(cfg+1)), continue; end
        vals = [];
        for i = 1:numel(files)
            f = files(i);
            if ~isfile(f), continue; end
            try
                T = readtable(f);
                if ~ismember("config_id", string(T.Properties.VariableNames)) || ...
                        round(median(double(T.config_id), "omitnan")) ~= cfg
                    continue;
                end
                if ~ismember("elevator_cmd_norm", string(T.Properties.VariableNames))
                    continue;
                end
                mask = true(height(T),1);
                if ismember("elevator_excitation", string(T.Properties.VariableNames))
                    e = abs(double(T.elevator_excitation));
                    firstActive = find(e > 1.0e-7, 1, "first");
                    if ~isempty(firstActive) && firstActive > 3
                        mask = false(height(T),1);
                        mask(1:firstActive-1) = true;
                    end
                elseif ismember("requested_elevator_cmd", string(T.Properties.VariableNames)) && ...
                        ismember("requested_elevator_trim", string(T.Properties.VariableNames))
                    mask = abs(double(T.requested_elevator_cmd) - double(T.requested_elevator_trim)) < 1.0e-7;
                end
                v = double(T.elevator_cmd_norm(mask));
                v = v(isfinite(v));
                if ~isempty(v)
                    vals(end+1,1) = median(v, "omitnan"); %#ok<AGROW>
                end
            catch
            end
        end
        if ~isempty(vals)
            physical(cfg+1) = median(vals, "omitnan");
        end
    end
end

for k = 1:5
    if ~isempty(learned.plant_bank{k}) && ~isfinite(physical(k))
        if logical(opts.RequirePhysicalElevatorNominals)
            error("AirdropX:AutoMPC:MissingPhysicalElevatorNominal", ...
                ['No physical elevator nominal was found for cfg%d. ' ...
                 'Pass PhysicalElevatorNominals or keep the v11 clean ID CSV files accessible.'], k-1);
        else
            physical(k) = double(trimBank(k).elevator_cmd);
        end
    end
end
end

function value = local_trim_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function opts = local_options(varargin)
opts.Identified = [];
opts.IdentifiedMat = "";
opts.OutputMat = "";
opts.Ts = 0.1;
opts.PredictionHorizon = 8;
opts.ControlHorizon = 3;
opts.InputCoordinateMode = "deviation_physical";
opts.PhysicalElevatorNominals = [];
opts.DerivePhysicalElevatorNominalsFromIdData = true;
opts.RequirePhysicalElevatorNominals = true;
opts.ConfigIds = (0:4).';

% Conservative first-smoke tuning. These limits deliberately stay close to
% the v11 ID excitation envelope (elevator +/-0.03, throttle +/-0.06).
opts.OutputWeights = [8.0 5.0 0.05 3.0 4.0];
opts.MVWeights = [0.20 0.15];
opts.MVRateWeights = [8.0 4.0];
opts.OutputScaleFactors = [2.0 2.0 3.0 1.0 2.0];
opts.MVScaleFactors = [0.03 0.06];
opts.MVRateECR = 0.0;
opts.ElevatorDeviationLimit = 0.035;
opts.ThrottleDeviationLimit = 0.060;
opts.ElevatorDeviationRateLimit = 0.006;
opts.ThrottleDeviationRateLimit = 0.010;
opts.DisableOutputDisturbanceModel = true;

% Legacy absolute-coordinate limits retained only for compatibility mode.
opts.ElevatorMin = -0.75;
opts.ElevatorMax = 0.45;
opts.ElevatorRateMin = -0.045;
opts.ElevatorRateMax = 0.045;
opts.ThrottleMin = 0.35;
opts.ThrottleMax = 0.88;
opts.ThrottleRateMin = -0.035;
opts.ThrottleRateMax = 0.035;

if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
if isempty(opts.Identified) && strlength(string(opts.IdentifiedMat)) == 0
    error("Identified or IdentifiedMat is required.");
end
end
