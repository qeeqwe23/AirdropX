function T = airdropx_drop_table(logs, varargin)
%AIRDROPX_DROP_TABLE Extract per-drop snapshot data from Simulink logs.
%
% Usage:
%   T = airdropx_drop_table(out.logsout)
%   T = airdropx_drop_table(out)
%   T = airdropx_drop_table(out.logsout, "OutputFile", "drop_table.csv")

p = inputParser;
addParameter(p, "OutputFile", "");
parse(p, varargin{:});

[tDrop, dropCount] = local_signal(logs, "drop_count");
if isempty(dropCount)
    error("Could not find drop_count in logs.");
end

dropEdges = find([0; diff(dropCount(:))] > 0.5);
dropIndex = (1:numel(dropEdges)).';
timeS = tDrop(dropEdges);

T = table(dropIndex, timeS, 'VariableNames', {'drop_index', 'time_s'});

signals = [
    "altitude_m"
    "vz_up_mps"
    "airspeed_mps"
    "groundspeed_mps"
    "pitch_deg"
    "roll_deg"
    "heading_deg"
    "mass_kg"
    "cg_x_m"
    "pos_n_m"
    "pos_e_m"
    "elevator_cmd_norm"
    "throttle_norm"
    "h_err"
    "delta_m_signal"
    "drop_trim_bias"
    "u_total"
    "u_out"
    ];

for i = 1:numel(signals)
    name = signals(i);
    [t, y] = local_signal(logs, name);
    T.(matlab.lang.makeValidName(name)) = local_sample_at_times(t, y, timeS);
end

outputFile = string(p.Results.OutputFile);
if strlength(outputFile) > 0
    writetable(T, outputFile);
end
end

function yq = local_sample_at_times(t, y, tq)
if isempty(t) || isempty(y)
    yq = NaN(size(tq));
    return;
end

t = t(:);
y = y(:);
yq = NaN(size(tq));
for i = 1:numel(tq)
    [~, idx] = min(abs(t - tq(i)));
    yq(i) = y(idx);
end
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
    if isfield(logs, "time_s")
        t = logs.time_s(:);
    elseif isfield(logs, "time")
        t = logs.time(:);
    end
    if isfield(logs, name)
        y = logs.(name)(:);
        if isempty(t)
            t = (0:numel(y)-1).';
        end
    end
end
end
