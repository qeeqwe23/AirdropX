function result = airdropx_auto_estimate_trim_from_data(varargin)
%AIRDROPX_AUTO_ESTIMATE_TRIM_FROM_DATA Estimate operating points from ID CSVs.
%
% This fixes the practical trim used by the learned MPC to the actual
% command/output levels seen in the JSBSim identification data.

opts = local_options(varargin{:});
files = local_csv_files(opts.InputRoot);
trimBank = repmat(struct("config_id", 0, "altitude_m", NaN, "airspeed_mps", NaN, ...
    "pitch_deg", NaN, "vz_up_mps", NaN, "q_dps", NaN, "elevator_cmd", NaN, ...
    "throttle_cmd", NaN, "score", NaN), 5, 1);
rows = table();

for cfgId = double(opts.ConfigIds(:)).'
    cfgFiles = local_files_for_config(files, cfgId);
    Tall = table();
    for i = 1:numel(cfgFiles)
        T = readtable(cfgFiles(i));
        if height(T) == 0
            continue;
        end
        T = local_fill_aliases(T);
        if ismember("time_s", string(T.Properties.VariableNames))
            T = T(double(T.time_s) >= double(opts.RecordStartS), :);
        end
        if height(T) > 0
            Tall = [Tall; T]; %#ok<AGROW>
        end
    end
    if height(Tall) == 0
        warning("AirdropX:AutoMPC:NoTrimRows", "No rows for config %d.", cfgId);
        continue;
    end

    trim = trimBank(cfgId + 1);
    trim.config_id = cfgId;
    trim.altitude_m = local_center(Tall.altitude_m, opts.CenterMethod);
    trim.airspeed_mps = local_center(Tall.airspeed_mps, opts.CenterMethod);
    trim.pitch_deg = local_center(Tall.pitch_deg, opts.CenterMethod);
    trim.vz_up_mps = local_center(Tall.vz_up_mps, opts.CenterMethod);
    trim.q_dps = local_center(Tall.q_dps, opts.CenterMethod);
    trim.elevator_cmd = local_center(Tall.elevator_cmd, opts.CenterMethod);
    trim.throttle_cmd = local_center(Tall.throttle_cmd, opts.CenterMethod);
    trim.score = local_std(Tall.altitude_m) + local_std(Tall.pitch_deg) + local_std(Tall.vz_up_mps);
    trimBank(cfgId + 1) = trim;

    row = struct2table(trim, "AsArray", true);
    row.rows_used = height(Tall);
    rows = [rows; row]; %#ok<AGROW>
end

result = struct("trim_bank", trimBank, "table", rows, "input_root", string(opts.InputRoot));
if strlength(string(opts.OutputMat)) > 0
    outDir = fileparts(opts.OutputMat);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    save(opts.OutputMat, "result", "trimBank", "rows");
end
if strlength(string(opts.OutputCsv)) > 0
    outDir = fileparts(opts.OutputCsv);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    writetable(rows, opts.OutputCsv);
end
end

function files = local_csv_files(root)
root = string(root);
if strlength(root) == 0 || ~isfolder(root)
    error("AirdropX:AutoMPC:BadInputRoot", "InputRoot not found: %s", root);
end
d = dir(fullfile(root, "**", "auto_id_timeseries.csv"));
files = strings(numel(d), 1);
for i = 1:numel(d)
    files(i) = string(fullfile(d(i).folder, d(i).name));
end
end

function cfgFiles = local_files_for_config(files, cfgId)
keep = false(numel(files), 1);
for i = 1:numel(files)
    try
        T = readtable(files(i));
        keep(i) = height(T) > 0 && ismember("config_id", string(T.Properties.VariableNames)) && round(T.config_id(1)) == cfgId;
    catch
        keep(i) = false;
    end
end
cfgFiles = files(keep);
end

function T = local_fill_aliases(T)
vars = string(T.Properties.VariableNames);
if ~ismember("elevator_cmd", vars)
    if ismember("elevator_delta", vars)
        T.elevator_cmd = T.elevator_delta;
    elseif ismember("elevator_cmd_norm", vars)
        T.elevator_cmd = T.elevator_cmd_norm;
    else
        T.elevator_cmd = NaN(height(T), 1);
    end
end
if ~ismember("throttle_cmd", vars)
    if ismember("throttle_norm", vars)
        T.throttle_cmd = T.throttle_norm;
    else
        T.throttle_cmd = NaN(height(T), 1);
    end
end
if ~ismember("q_dps", vars)
    if ismember("pitch_deg", vars) && ismember("time_s", vars)
        T.q_dps = gradient(double(T.pitch_deg), double(T.time_s));
    else
        T.q_dps = zeros(height(T), 1);
    end
end
end

function value = local_center(x, method)
x = double(x(:));
x = x(isfinite(x));
if isempty(x)
    value = NaN;
elseif strcmpi(string(method), "median")
    value = median(x);
else
    value = mean(x);
end
end

function value = local_std(x)
x = double(x(:));
x = x(isfinite(x));
if numel(x) < 2
    value = 0.0;
else
    value = std(x);
end
end

function opts = local_options(varargin)
opts.InputRoot = "";
opts.OutputMat = "";
opts.OutputCsv = "";
opts.ConfigIds = (0:4).';
opts.RecordStartS = 0.0;
opts.CenterMethod = "median";
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
