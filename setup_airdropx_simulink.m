function cfg = setup_airdropx_simulink(varargin)
%SETUP_AIRDROPX_SIMULINK Project-root shortcut for MATLAB/Simulink setup.
%
% Usage from the AirdropX project root:
%   setup_airdropx_simulink
%   cfg = setup_airdropx_simulink("OpenModel", true)

projectRoot = string(fileparts(mfilename("fullpath")));
matlabDir = fullfile(projectRoot, "matlab");

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

opts = local_options(varargin{:});

cfgArgs = { ...
    "ProjectRoot", projectRoot, ...
    "AircraftName", opts.AircraftName, ...
    "IcName", opts.IcName, ...
    "Model", opts.Model, ...
    "AssignBase", true};
if ~isempty(opts.Dt)
    cfgArgs = [cfgArgs, {"Dt", opts.Dt}];
end

cfg = airdropx_sim_params(cfgArgs{:});

if opts.OpenModel
    modelPath = fullfile(matlabDir, string(opts.Model) + ".slx");
    if isfile(modelPath)
        open_system(modelPath);
    else
        warning("AirdropX setup: model not found: %s", modelPath);
    end
end

fprintf("AirdropX Simulink workspace initialized.\n");
fprintf("  projectRoot : %s\n", cfg.projectRoot);
fprintf("  aircraftName: %s\n", cfg.aircraftName);
fprintf("  icName      : %s\n", cfg.icName);
fprintf("  initial V   : %.3f m/s\n", cfg.initial.airspeed_mps);
fprintf("  initial theta: %.3f deg\n", cfg.initial.pitch_deg);
fprintf("  dt          : %.10g\n", cfg.sim.dt_s);
fprintf("  drop_mode   : %.0f (1=fixed, 2=CARP)\n", cfg.drop_mode);
end

function opts = local_options(varargin)
opts.AircraftName = "MQ9_Reaper";
opts.IcName = "";
opts.Dt = [];
opts.Model = "untitled1";
opts.OpenModel = false;

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
