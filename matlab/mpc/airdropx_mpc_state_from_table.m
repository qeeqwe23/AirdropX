function X = airdropx_mpc_state_from_table(T, cfg)
%AIRDROPX_MPC_STATE_FROM_TABLE Convert exported timeseries rows to MPC states.

if nargin < 2 || isempty(cfg)
    cfg = airdropx_mpc_config();
end

required = ["time_s", "altitude_m", "vz_up_mps", "airspeed_mps", "pitch_deg"];
for i = 1:numel(required)
    if ~ismember(required(i), string(T.Properties.VariableNames))
        error("CSV table is missing required column: %s", required(i));
    end
end

t = double(T.time_s(:));
h = double(T.altitude_m(:));
vz = double(T.vz_up_mps(:));
v = double(T.airspeed_mps(:));
pitch = double(T.pitch_deg(:));
massKg = local_reference_column(T, "mass_kg", cfg.reference.mass_kg);
cgXM = local_reference_column(T, "cg_x_m", cfg.reference.cg_x_m);

hRef = local_reference_column(T, "target_altitude_m", cfg.reference.h_m);
vRef = local_reference_column(T, "target_airspeed_mps", cfg.reference.v_mps);
pitchRef = local_reference_column(T, "target_pitch_deg", cfg.reference.pitch_deg);
massRef = local_reference_column(T, "target_mass_kg", cfg.reference.mass_kg);
cgRef = local_reference_column(T, "target_cg_x_m", cfg.reference.cg_x_m);

q = zeros(size(pitch));
hErr = h - hRef;
vErr = v - vRef;
pitchErr = pitch - pitchRef;
massErr = massKg - massRef;
cgErr = cgXM - cgRef;

groups = local_group_indices(T);
for g = 1:numel(groups)
    idx = groups{g};
    if numel(idx) >= 2
        tg = t(idx);
        pg = pitch(idx);
        dt = [max(tg(2) - tg(1), eps); max(diff(tg), eps)];
        q(idx) = [0; diff(pg)] ./ dt;
    end
end

X = [hErr, vz, vErr, pitchErr, q, massErr, cgErr];
end

function ref = local_reference_column(T, name, fallback)
if ismember(name, string(T.Properties.VariableNames))
    ref = double(T.(name)(:));
else
    ref = fallback * ones(height(T), 1);
end
end

function groups = local_group_indices(T)
if ismember("case_id", string(T.Properties.VariableNames))
    ids = string(T.case_id(:));
else
    ids = repmat("case_001", height(T), 1);
end

uniqueIds = unique(ids, "stable");
groups = cell(numel(uniqueIds), 1);
for i = 1:numel(uniqueIds)
    groups{i} = find(ids == uniqueIds(i));
end
end
