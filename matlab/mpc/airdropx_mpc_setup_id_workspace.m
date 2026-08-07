function cfg = airdropx_mpc_setup_id_workspace(varargin)
%AIRDROPX_MPC_SETUP_ID_WORKSPACE Prepare base workspace for preserved SLX files.

opts = local_options(varargin{:});
addpath(fileparts(mfilename("fullpath")));
addpath(fileparts(fileparts(mfilename("fullpath"))));

cfg = airdropx_sim_params();
assignin("base", "airdropx_mpc_reference_mass_kg", double(opts.ReferenceMassKg));
assignin("base", "airdropx_mpc_reference_cg_x_m", double(opts.ReferenceCgXM));
assignin("base", "airdropx_mpc_control_altitude_bias_m", double(opts.ControlAltitudeBiasM));
assignin("base", "airdropx_mpc_config_overrides", opts.ConfigOverrides);

if logical(opts.UseExcitation)
    profile = airdropx_mpc_excitation_profile("Dt", cfg.sim.dt_s, "StopTimeS", cfg.sim.stop_time_s);
    assignin("base", "airdropx_mpc_elevator_excitation", profile.elevator);
    assignin("base", "airdropx_mpc_throttle_excitation", profile.throttle);
else
    assignin("base", "airdropx_mpc_elevator_excitation", [0.0 0.0; cfg.sim.stop_time_s 0.0]);
    assignin("base", "airdropx_mpc_throttle_excitation", [0.0 0.0; cfg.sim.stop_time_s 0.0]);
end

if strlength(string(opts.Model)) > 0
    try
        if bdIsLoaded(char(opts.Model))
            set_param(char(opts.Model), "StopTime", num2str(cfg.sim.stop_time_s));
        end
    catch
    end
end
end

function opts = local_options(varargin)
opts.Model = "";
opts.ReferenceMassKg = 3423.0;
opts.ReferenceCgXM = 5.28048992112182;
opts.ControlAltitudeBiasM = 0.0;
opts.ConfigOverrides = struct();
opts.UseExcitation = true;
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i + 1};
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = value;
end
end
