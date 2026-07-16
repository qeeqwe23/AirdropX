function result = airdropx_mpc_create_closed_loop_model(varargin)
%AIRDROPX_MPC_CREATE_CLOSED_LOOP_MODEL Create standalone MPC closed-loop model.
%
% The generated model is separate from matlab/untitled1.slx. The Simulink
% model is an execution wrapper for the standalone MPC: measured plant states
% feed the MPC block, and MPC actuator commands feed the JSBSim input Mux.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
mpcDir = string(fileparts(thisFile));
matlabDir = string(fileparts(mpcDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(mpcDir));

sourceModel = string(opts.SourceModel);
targetModel = string(opts.TargetModel);
sourcePath = local_source_path(sourceModel, matlabDir, mpcDir);
targetPath = fullfile(mpcDir, targetModel + ".slx");

if ~isfile(sourcePath)
    error("Source model not found: %s", sourcePath);
end

if bdIsLoaded(targetModel)
    close_system(char(targetModel), 0);
end

copyfile(char(sourcePath), char(targetPath), "f");

load_system(char(targetPath));
model = char(targetModel);

set_param(model, ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "PreLoadFcn", local_setup_callback(targetModel), ...
    "PostLoadFcn", local_setup_callback(targetModel), ...
    "InitFcn", local_setup_callback(targetModel));

if opts.DisableVR
    local_disable_vr_blocks(model);
else
    local_configure_vr_blocks(model, matlabDir);
end
local_add_mpc_controller(model, string(opts.ModelMat));

save_system(model);
close_system(model, 0);

result = struct();
result.project_root = projectRoot;
result.target_model = targetModel;
result.target_path = string(targetPath);
result.model_mat = string(opts.ModelMat);

fprintf("AirdropX MPC closed-loop model created:\n");
fprintf("  %s\n", targetPath);
end

function sourcePath = local_source_path(sourceModel, matlabDir, mpcDir)
sourcePath = fullfile(matlabDir, sourceModel + ".slx");
if ~isfile(sourcePath)
    sourcePath = fullfile(mpcDir, sourceModel + ".slx");
end
end

function local_add_mpc_controller(model, modelMat)
muxPath = string(model) + "/Mux";
demuxPath = string(model) + "/Demux";
if getSimulinkBlockHandle(muxPath) < 0
    error("Could not find plant input Mux: %s", muxPath);
end
if getSimulinkBlockHandle(demuxPath) < 0
    error("Could not find JSBSim output Demux: %s", demuxPath);
end

stateMuxPath = string(model) + "/MPC_StateMux";
mpcPath = string(model) + "/MPC_Controller";
cmdDemuxPath = string(model) + "/MPC_CommandDemux";
elevDelayPath = string(model) + "/MPC_ElevatorDelay";
thrDelayPath = string(model) + "/MPC_ThrottleDelay";
local_delete_block_if_exists(stateMuxPath);
local_delete_block_if_exists(mpcPath);
local_delete_block_if_exists(cmdDemuxPath);
local_delete_block_if_exists(elevDelayPath);
local_delete_block_if_exists(thrDelayPath);

add_block("simulink/Signal Routing/Mux", char(stateMuxPath), ...
    "Inputs", "6", ...
    "Position", [520 55 545 175]);
add_block("simulink/User-Defined Functions/Level-2 MATLAB S-Function", char(mpcPath), ...
    "FunctionName", "sfun_airdropx_mpc_controller", ...
    "Parameters", "'" + char(modelMat) + "'", ...
    "Position", [590 75 735 150]);
add_block("simulink/Signal Routing/Demux", char(cmdDemuxPath), ...
    "Outputs", "2", ...
    "Position", [780 80 805 145]);
add_block("simulink/Discrete/Unit Delay", char(elevDelayPath), ...
    "InitialCondition", "airdropx_initial_elevator_delta", ...
    "SampleTime", "dt", ...
    "Position", [850 70 890 100]);
add_block("simulink/Discrete/Unit Delay", char(thrDelayPath), ...
    "InitialCondition", "airdropx_initial_throttle_cmd", ...
    "SampleTime", "dt", ...
    "Position", [850 125 890 155]);

demuxPorts = get_param(char(demuxPath), "PortHandles");
stateMuxPorts = get_param(char(stateMuxPath), "PortHandles");
mpcPorts = get_param(char(mpcPath), "PortHandles");
cmdDemuxPorts = get_param(char(cmdDemuxPath), "PortHandles");
elevDelayPorts = get_param(char(elevDelayPath), "PortHandles");
thrDelayPorts = get_param(char(thrDelayPath), "PortHandles");

% JSBSim output indices: altitude=2, vz=3, airspeed=4, pitch=6,
% mass=10, cg_x=11. Keep these as measured disturbances for MPC.
srcIdx = [2 3 4 6 10 11];
for i = 1:numel(srcIdx)
    line = add_line(model, demuxPorts.Outport(srcIdx(i)), stateMuxPorts.Inport(i), ...
        "autorouting", "on");
    set_param(line, "Name", "mpc_state_" + string(i));
end
add_line(model, stateMuxPorts.Outport(1), mpcPorts.Inport(1), "autorouting", "on");
add_line(model, mpcPorts.Outport(1), cmdDemuxPorts.Inport(1), "autorouting", "on");

muxPorts = get_param(char(muxPath), "PortHandles");
local_delete_dst_line(muxPorts.Inport(1));
local_delete_dst_line(muxPorts.Inport(2));
add_line(model, cmdDemuxPorts.Outport(1), elevDelayPorts.Inport(1), "autorouting", "on");
add_line(model, cmdDemuxPorts.Outport(2), thrDelayPorts.Inport(1), "autorouting", "on");
elevLine = add_line(model, elevDelayPorts.Outport(1), muxPorts.Inport(1), "autorouting", "on");
thrLine = add_line(model, thrDelayPorts.Outport(1), muxPorts.Inport(2), "autorouting", "on");
set_param(elevLine, "Name", "mpc_elevator_to_plant");
set_param(thrLine, "Name", "mpc_throttle_to_plant");
end

function callback = local_setup_callback(targetModel)
callback = sprintf("airdropx_mpc_setup_closed_loop_workspace('Model','%s');", char(targetModel));
end

function local_delete_dst_line(dstPort)
try
    line = get_param(dstPort, "Line");
    if line ~= -1
        delete_line(line);
    end
catch
end
end

function local_delete_block_if_exists(blockPath)
try
    if getSimulinkBlockHandle(blockPath) >= 0
        delete_block(blockPath);
    end
catch
end
end

function local_configure_vr_blocks(model, matlabDir)
worldPath = fullfile(matlabDir, "vr", "airdropx_scene.wrl");
blocks = local_vr_sink_blocks(model);
for i = 1:numel(blocks)
    try
        set_param(blocks{i}, "WorldFileName", char(worldPath));
    catch
    end
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

try
    byType = local_vr_sink_blocks(model);
    for i = 1:numel(byType)
        try
            set_param(byType{i}, "Commented", "on");
        catch
        end
    end
catch
end
end

function blocks = local_vr_sink_blocks(model)
blocks = {};
try
    blocks = find_system(model, ...
        "LookUnderMasks", "all", ...
        "FollowLinks", "on", ...
        "BlockType", "S-Function", ...
        "FunctionName", "vrsfunc");
catch
end
end

function opts = local_options(varargin)
opts.SourceModel = "untitled1";
opts.TargetModel = "airdropx_mpc_closed_loop";
opts.ModelMat = "";
opts.DisableVR = true;

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
