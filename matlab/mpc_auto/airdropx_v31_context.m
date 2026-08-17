function C = airdropx_v31_context(varargin)
%AIRDROPX_V31_CONTEXT Canonical v31 mission/physics/controller context.
%
% v31 deliberately separates absolute mission altitude from the aerodynamic
% context.  Physics/controller transfer is primarily a function of airspeed,
% mass, CG and payload state.  Target altitude remains part of the mission
% signature and qualification result, but is NOT part of the physical
% signature used to transfer Plant/controller knowledge.

opts = local_options(varargin{:});
cfgId = round(double(opts.ConfigId));
if cfgId < 0 || cfgId > round(double(opts.TotalDropCount))
    error("AirdropX:V31:BadConfigId","ConfigId must be in 0..TotalDropCount.");
end
mass = double(opts.EstimatedMassKg);
if ~isfinite(mass)
    mass = double(opts.ReferenceMassKg) - cfgId * double(opts.CargoMassKg);
end
cg = double(opts.CgXM);
cgKnown = isfinite(cg);
if ~cgKnown, cg = 0.0; end
payloadRemaining = max(0,(double(opts.TotalDropCount)-cfgId)*double(opts.CargoMassKg));
dropFraction = cfgId / max(1,double(opts.TotalDropCount));

C = struct();
C.schema_version = "v31.0";
C.target_altitude_m = double(opts.TargetAltitudeM);
C.target_airspeed_mps = double(opts.TargetAirspeedMps);
C.reference_mass_kg = double(opts.ReferenceMassKg);
C.cargo_mass_kg = double(opts.CargoMassKg);
C.total_drop_count = round(double(opts.TotalDropCount));
C.config_id = cfgId;
C.estimated_mass_kg = mass;
C.cg_x_m = cg;
C.cg_known = logical(cgKnown);
C.payload_remaining_kg = payloadRemaining;
C.drop_fraction = dropFraction;

% Mission identity contains target altitude.  It is used only for mission
% bookkeeping/certification, never to claim that two H values are different
% aerodynamic plants.
C.mission_signature = string(sprintf( ...
    'H%.4f_V%.4f_RefM%.3f_Cargo%.3f_Ndrop%d', ...
    C.target_altitude_m,C.target_airspeed_mps,C.reference_mass_kg, ...
    C.cargo_mass_kg,C.total_drop_count));

% Mission-level physical signature intentionally excludes H.
C.physics_signature = string(sprintf( ...
    'V%.4f_RefM%.3f_Cargo%.3f_Ndrop%d', ...
    C.target_airspeed_mps,C.reference_mass_kg,C.cargo_mass_kg,C.total_drop_count));

% cfg controller transfer signature also excludes H.  CG is included only
% when known so imported legacy rows with unknown CG do not invent precision.
C.controller_physical_signature = string(sprintf( ...
    'V%.4f_M%.3f_Cargo%.3f_CG%.5f_CGk%d_cfg%d', ...
    C.target_airspeed_mps,C.estimated_mass_kg,C.cargo_mass_kg,C.cg_x_m, ...
    C.cg_known,C.config_id));

% Dimensionless features used by v31-side nearest-neighbour reporting.
% Altitude is absent by design.
C.physics_features = [ ...
    C.target_airspeed_mps/10.0, ...
    C.reference_mass_kg/500.0, ...
    C.cargo_mass_kg/300.0, ...
    C.total_drop_count/4.0];
C.controller_features = [ ...
    C.target_airspeed_mps/10.0, ...
    C.estimated_mass_kg/500.0, ...
    C.cargo_mass_kg/300.0, ...
    C.payload_remaining_kg/600.0, ...
    C.drop_fraction, ...
    C.config_id/4.0, ...
    C.cg_x_m/0.20*double(C.cg_known), ...
    double(C.cg_known)];
end

function opts = local_options(varargin)
opts = struct();
opts.TargetAltitudeM = 200.0;
opts.TargetAirspeedMps = 50.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.TotalDropCount = 4;
opts.ConfigId = 0;
opts.EstimatedMassKg = NaN;
opts.CgXM = NaN;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    name=string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name)=varargin{i+1};
end
end
