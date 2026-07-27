function result = airdropx_mpc_outer_pd_create_model(varargin)
%AIRDROPX_MPC_OUTER_PD_CREATE_MODEL Create MPC outer + PD inner Simulink model.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
outerDir = string(fileparts(thisFile));
matlabDir = string(fileparts(outerDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "mpc")));
addpath(char(outerDir));

sourceModel = string(opts.SourceModel);
targetModel = string(opts.TargetModel);
sourcePath = fullfile(matlabDir, sourceModel + ".slx");
targetPath = fullfile(outerDir, targetModel + ".slx");

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
local_add_outer_pd_controller(model);

save_system(model);
close_system(model, 0);

result = struct();
result.project_root = projectRoot;
result.target_model = targetModel;
result.target_path = string(targetPath);

fprintf("AirdropX MPC outer + PD inner model created:\n");
fprintf("  %s\n", targetPath);
end

function local_add_outer_pd_controller(model)
muxPath = string(model) + "/Mux";
demuxPath = string(model) + "/Demux";
if getSimulinkBlockHandle(muxPath) < 0
    error("Could not find plant input Mux: %s", muxPath);
end
if getSimulinkBlockHandle(demuxPath) < 0
    error("Could not find JSBSim output Demux: %s", demuxPath);
end

stateMuxPath = string(model) + "/MPCOuterPD_StateMux";
ctrlPath = string(model) + "/MPC_Outer_PD_Controller";
cmdDemuxPath = string(model) + "/MPCOuterPD_CommandDemux";
elevDelayPath = string(model) + "/MPCOuterPD_ElevatorDelay";
thrDelayPath = string(model) + "/MPCOuterPD_ThrottleDelay";
pitchRefTermPath = string(model) + "/MPCOuterPD_PitchRefTerminator";

for p = [stateMuxPath, ctrlPath, cmdDemuxPath, elevDelayPath, thrDelayPath, pitchRefTermPath]
    local_delete_block_if_exists(p);
end

add_block("simulink/Signal Routing/Mux", char(stateMuxPath), ...
    "Inputs", "6", ...
    "Position", [520 55 545 175]);
add_block("simulink/User-Defined Functions/Level-2 MATLAB S-Function", char(ctrlPath), ...
    "FunctionName", "sfun_airdropx_mpc_outer_pd_controller", ...
    "Parameters", "''", ...
    "Position", [590 75 755 150]);
add_block("simulink/Signal Routing/Demux", char(cmdDemuxPath), ...
    "Outputs", "3", ...
    "Position", [800 72 825 157]);
add_block("simulink/Discrete/Unit Delay", char(elevDelayPath), ...
    "InitialCondition", "airdropx_initial_elevator_delta", ...
    "SampleTime", "dt", ...
    "Position", [870 65 910 95]);
add_block("simulink/Discrete/Unit Delay", char(thrDelayPath), ...
    "InitialCondition", "airdropx_initial_throttle_cmd", ...
    "SampleTime", "dt", ...
    "Position", [870 120 910 150]);
add_block("simulink/Sinks/Terminator", char(pitchRefTermPath), ...
    "Position", [870 170 890 190]);

demuxPorts = get_param(char(demuxPath), "PortHandles");
stateMuxPorts = get_param(char(stateMuxPath), "PortHandles");
ctrlPorts = get_param(char(ctrlPath), "PortHandles");
cmdDemuxPorts = get_param(char(cmdDemuxPath), "PortHandles");
elevDelayPorts = get_param(char(elevDelayPath), "PortHandles");
thrDelayPorts = get_param(char(thrDelayPath), "PortHandles");
pitchRefTermPorts = get_param(char(pitchRefTermPath), "PortHandles");

srcIdx = [2 3 4 6 10 11]; % altitude, vz, airspeed, pitch, mass, cg_x
for i = 1:numel(srcIdx)
    line = add_line(model, demuxPorts.Outport(srcIdx(i)), stateMuxPorts.Inport(i), ...
        "autorouting", "on");
    set_param(line, "Name", "mpc_outer_pd_state_" + string(i));
end
add_line(model, stateMuxPorts.Outport(1), ctrlPorts.Inport(1), "autorouting", "on");
add_line(model, ctrlPorts.Outport(1), cmdDemuxPorts.Inport(1), "autorouting", "on");

muxPorts = get_param(char(muxPath), "PortHandles");
local_delete_dst_line(muxPorts.Inport(1));
local_delete_dst_line(muxPorts.Inport(2));

add_line(model, cmdDemuxPorts.Outport(1), elevDelayPorts.Inport(1), "autorouting", "on");
add_line(model, cmdDemuxPorts.Outport(2), thrDelayPorts.Inport(1), "autorouting", "on");
pitchLine = add_line(model, cmdDemuxPorts.Outport(3), pitchRefTermPorts.Inport(1), "autorouting", "on");
elevLine = add_line(model, elevDelayPorts.Outport(1), muxPorts.Inport(1), "autorouting", "on");
thrLine = add_line(model, thrDelayPorts.Outport(1), muxPorts.Inport(2), "autorouting", "on");

set_param(elevLine, "Name", "mpc_outer_pd_elevator_to_plant");
set_param(thrLine, "Name", "mpc_outer_pd_throttle_to_plant");
set_param(pitchLine, "Name", "mpc_outer_pd_pitch_ref_deg");
end

function callback = local_setup_callback(targetModel)
callback = sprintf("airdropx_mpc_outer_pd_setup_workspace('Model','%s');", char(targetModel));
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
        set_param(blocks{i}, "Commented", "off");
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
opts.TargetModel = "airdropx_mpc_outer_pd_closed_loop";
opts.DisableVR = false;

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
