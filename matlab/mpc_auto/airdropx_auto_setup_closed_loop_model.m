function modelPath = airdropx_auto_setup_closed_loop_model(varargin)
%AIRDROPX_AUTO_SETUP_CLOSED_LOOP_MODEL Create the auto-MPC closed-loop SLX.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

sourcePath = fullfile(paths.mpcDir, "airdropx_mpc_closed_loop.slx");
modelPath = fullfile(paths.autoDir, char(opts.ModelName) + ".slx");
if ~isfile(sourcePath)
    error("AirdropX:AutoMPC:MissingSourceModel", "Missing source SLX: %s", sourcePath);
end
if ~isfile(modelPath) || opts.Overwrite
    copyfile(sourcePath, modelPath, "f");
end

modelName = char(opts.ModelName);
load_system(modelPath);
cleanup = onCleanup(@() local_close_model(modelName));

mpcBlock = [modelName, '/MPC_Controller'];
set_param(mpcBlock, "FunctionName", "sfun_airdropx_auto_mpc_controller");
set_param(mpcBlock, "Parameters", "''");
set_param(modelName, "InitFcn", "");
save_system(modelName, modelPath);
set_param(modelName, "Dirty", "off");
fprintf("Auto MPC closed-loop model ready:\n  %s\n", modelPath);
end

function local_close_model(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
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
paths.sfuncDir = char(fullfile(matlabDir, "sfunc_jsbsim"));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.ModelName = "airdropx_auto_mpc_closed_loop";
opts.Overwrite = false;
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = varargin{i + 1};
end
end

