function result = airdropx_mpc_create_id_model(varargin)
%AIRDROPX_MPC_CREATE_ID_MODEL Create a separate Simulink model for ID data.
%
% The generated model is a copy of matlab/untitled1.slx with elevator and
% throttle excitation injection inserted before the JSBSim input Mux. The
% original model is not modified.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
mpcDir = string(fileparts(thisFile));
matlabDir = string(fileparts(mpcDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(mpcDir));

sourceModel = string(opts.SourceModel);
targetModel = string(opts.TargetModel);
sourcePath = fullfile(matlabDir, sourceModel + ".slx");
targetPath = fullfile(mpcDir, targetModel + ".slx");

if ~isfile(sourcePath)
    error("Source model not found: %s", sourcePath);
end

if bdIsLoaded(sourceModel)
    close_system(char(sourceModel), 0);
end
if bdIsLoaded(targetModel)
    close_system(char(targetModel), 0);
end

load_system(char(sourcePath));
save_system(char(sourceModel), char(targetPath));
close_system(char(sourceModel), 0);

load_system(char(targetPath));
model = char(targetModel);

set_param(model, ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "PreLoadFcn", "airdropx_mpc_setup_id_workspace('Model','airdropx_mpc_id');", ...
    "PostLoadFcn", "airdropx_mpc_setup_id_workspace('Model','airdropx_mpc_id');", ...
    "InitFcn", "airdropx_mpc_setup_id_workspace('Model','airdropx_mpc_id');");

local_insert_channel(model, 1, "elevator", ...
    "airdropx_mpc_elevator_excitation", -0.85, 0.85, [250 70 330 160]);
local_insert_channel(model, 2, "throttle", ...
    "airdropx_mpc_throttle_excitation", 0.0, 1.0, [250 180 330 270]);
local_disable_vr_blocks(model);

save_system(model);
close_system(model, 0);

result = struct();
result.project_root = projectRoot;
result.source_model = sourceModel;
result.target_model = targetModel;
result.target_path = string(targetPath);

fprintf("AirdropX MPC identification model created:\n");
fprintf("  %s\n", targetPath);
end

function local_insert_channel(model, muxPortIndex, name, excitationVariable, lowerLimit, upperLimit, pos)
muxPath = string(model) + "/Mux";
if getSimulinkBlockHandle(muxPath) < 0
    error("Could not find Mux block in model: %s", muxPath);
end

ports = get_param(char(muxPath), "PortHandles");
if numel(ports.Inport) < muxPortIndex
    error("Mux has only %d input ports; requested port %d.", numel(ports.Inport), muxPortIndex);
end

dstPort = ports.Inport(muxPortIndex);
oldLine = get_param(dstPort, "Line");
if oldLine == -1
    error("Mux port %d has no existing source line.", muxPortIndex);
end
srcPort = get_param(oldLine, "SrcPortHandle");
delete_line(oldLine);

sumPath = string(model) + "/MPC_ID_" + name + "_sum";
satPath = string(model) + "/MPC_ID_" + name + "_saturation";
excPath = string(model) + "/MPC_ID_" + name + "_excitation";

local_delete_block_if_exists(sumPath);
local_delete_block_if_exists(satPath);
local_delete_block_if_exists(excPath);

add_block("simulink/Math Operations/Sum", char(sumPath), ...
    "Inputs", "++", ...
    "Position", pos);
add_block("simulink/Discontinuities/Saturation", char(satPath), ...
    "LowerLimit", num2str(lowerLimit, "%.15g"), ...
    "UpperLimit", num2str(upperLimit, "%.15g"), ...
    "Position", pos + [130 10 130 10]);
add_block("simulink/Sources/From Workspace", char(excPath), ...
    "VariableName", excitationVariable, ...
    "SampleTime", "dt", ...
    "Position", pos + [-165 55 -165 55]);

sumPorts = get_param(char(sumPath), "PortHandles");
satPorts = get_param(char(satPath), "PortHandles");
excPorts = get_param(char(excPath), "PortHandles");

baseLine = add_line(model, srcPort, sumPorts.Inport(1), "autorouting", "on");
set_param(baseLine, "Name", "mpc_" + name + "_base");

excLine = add_line(model, excPorts.Outport(1), sumPorts.Inport(2), "autorouting", "on");
set_param(excLine, "Name", "mpc_" + name + "_excitation");

sumLine = add_line(model, sumPorts.Outport(1), satPorts.Inport(1), "autorouting", "on");
set_param(sumLine, "Name", "mpc_" + name + "_sum");

plantLine = add_line(model, satPorts.Outport(1), dstPort, "autorouting", "on");
set_param(plantLine, "Name", "mpc_" + name + "_to_plant");
end

function local_delete_block_if_exists(blockPath)
try
    if getSimulinkBlockHandle(blockPath) >= 0
        delete_block(blockPath);
    end
catch
end
end

function local_disable_vr_blocks(model)
blocks = find_system(model, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "RegExp", "on", ...
    "Name", ".*VR.*");
for i = 1:numel(blocks)
    if string(blocks{i}) == string(model)
        continue;
    end
    try
        set_param(blocks{i}, "Commented", "on");
    catch
    end
end

types = ["vrsfunc", "vrsink"];
for iType = 1:numel(types)
    try
        byType = find_system(model, ...
            "LookUnderMasks", "all", ...
            "FollowLinks", "on", ...
            "BlockType", "S-Function", ...
            "FunctionName", char(types(iType)));
        for i = 1:numel(byType)
            try
                set_param(byType{i}, "Commented", "on");
            catch
            end
        end
    catch
    end
end
end

function opts = local_options(varargin)
opts.SourceModel = "untitled1";
opts.TargetModel = "airdropx_mpc_id";

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
