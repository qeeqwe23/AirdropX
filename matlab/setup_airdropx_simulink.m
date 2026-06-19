function cfg = setup_airdropx_simulink(varargin)
%SETUP_AIRDROPX_SIMULINK Initialize MATLAB workspace for AirdropX Simulink.
%
% Usage:
%   setup_airdropx_simulink
%   cfg = setup_airdropx_simulink("OpenModel", true)
%   cfg = setup_airdropx_simulink("ProjectRoot", "C:\path\to\AirdropX")
%
% This script creates the base-workspace variables used by the Simulink
% model and S-Function from airdropx_sim_params.m.

opts = local_options(varargin{:});

if strlength(string(opts.ProjectRoot)) == 0
    thisFile = mfilename('fullpath');
    matlabDir = fileparts(thisFile);
    projectRoot = char(fileparts(matlabDir));
else
    projectRoot = char(opts.ProjectRoot);
end

if ~isfolder(projectRoot)
    error("AirdropX setup: project root does not exist: %s", projectRoot);
end

matlabDir = fullfile(projectRoot, "matlab");
sfuncDir = fullfile(matlabDir, "sfunc_jsbsim");
vrDir = fullfile(matlabDir, "vr");

addpath(matlabDir);
addpath(sfuncDir);
addpath(vrDir);

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

if opts.ChangeDirectory
    cd(projectRoot);
end

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
opts.ProjectRoot = "";
opts.AircraftName = "MQ9_Reaper";
opts.IcName = "";
opts.Dt = [];
opts.Model = "untitled1";
opts.OpenModel = false;
opts.ChangeDirectory = true;

if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end

for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i+1};
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = value;
end
end
