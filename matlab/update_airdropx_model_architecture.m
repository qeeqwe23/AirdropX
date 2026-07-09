function update_airdropx_model_architecture(modelName)
%UPDATE_AIRDROPX_MODEL_ARCHITECTURE Normalize the Simulink model wiring.
%
% This keeps tunable numbers out of the .slx model. The model references
% variables published by setup_airdropx_simulink/airdropx_sim_params instead.

if nargin < 1 || strlength(string(modelName)) == 0
    modelName = "untitled1";
else
    modelName = string(modelName);
end

cfg = setup_airdropx_simulink("Model", modelName, "OpenModel", false);
modelPath = fullfile(cfg.matlabDir, modelName + ".slx");
load_system(modelPath);
model = char(modelName);

set_param(model, ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");

set_param(model + "/wind_speed_mps", "Value", "airdropx_wind_speed_mps");
set_param(model + "/wind_dir_from_deg", "Value", "airdropx_wind_dir_from_deg");
set_param(model + "/reset_cmd", "Value", "airdropx_reset_cmd");
set_param(model + "/elevator_delta", "Value", "airdropx_initial_elevator_delta");
set_param(model + "/throttle_cmd", "Value", "airdropx_initial_throttle_cmd");

set_param(model + "/Unit Delay1", ...
    "InitialCondition", "airdropx_initial_elevator_delta", ...
    "SampleTime", "dt");
set_param(model + "/Unit Delay2", ...
    "InitialCondition", "airdropx_initial_throttle_cmd", ...
    "SampleTime", "dt");
set_param(model + "/Unit Delay3", ...
    "InitialCondition", "airdropx_initial_drop_cmd", ...
    "SampleTime", "dt");

local_set_emchart_script(model, "DropSchedule", [
    "function [drop_cmd, next_drop_index, schedule_done] = DropSchedule(t)"
    "%#codegen"
    "[drop_cmd, next_drop_index, schedule_done] = airdropx_four_drop_schedule(t, ..."
    "    airdropx_fixed_drop_start_s, airdropx_fixed_drop_interval_s, ..."
    "    airdropx_fixed_drop_pulse_s, airdropx_fixed_drop_total);"
    "end"]);
local_ensure_chart_parameters(model, "DropSchedule", [
    "airdropx_fixed_drop_start_s"
    "airdropx_fixed_drop_interval_s"
    "airdropx_fixed_drop_pulse_s"
    "airdropx_fixed_drop_total"]);

local_set_emchart_script(model, "PD_NW20", [
    "function [elevator_delta, throttle_cmd, delta_m_signal, h_err, u_pd, drop_trim_bias, u_total, u_out, saturated] = PD_NW20(altitude_m, vz_up_mps, airspeed_mps, mass_kg, drop_count, pitch_deg)"
    "%#codegen"
    ""
    "[elevator_delta, throttle_cmd, delta_m_signal, h_err, u_pd, drop_trim_bias, u_total, u_out, saturated] = ..."
    "    airdropx_pd_nw20_block(altitude_m, vz_up_mps, airspeed_mps, pitch_deg, mass_kg, drop_count, ..."
    "        airdropx_target_altitude_m, airdropx_pd_Kp, airdropx_pd_Kd, ..."
    "        airdropx_pd_u_limit, airdropx_pd_u_rate_limit, airdropx_pd_K_mass, ..."
    "        airdropx_pd_bias_rate_limit, airdropx_pd_throttle_kp, ..."
    "        airdropx_pd_throttle_fixed, airdropx_pd_throttle_alt_kp, ..."
    "        airdropx_pd_throttle_vz_kd, airdropx_pd_v_ref_mps, ..."
    "        airdropx_drop_mass_signal_kg, airdropx_initial_elevator_delta, ..."
    "        airdropx_pd_pitch_ref_deg, airdropx_pd_pitch_kp, airdropx_pd_pitch_limit, ..."
    "        airdropx_pd_pitch_rate_kd, airdropx_pd_pitch_rate_limit, airdropx_pd_dt_s);"
    ""
    "end"]);
local_ensure_chart_parameters(model, "PD_NW20", [
    "airdropx_target_altitude_m"
    "airdropx_pd_Kp"
    "airdropx_pd_Kd"
    "airdropx_pd_u_limit"
    "airdropx_pd_u_rate_limit"
    "airdropx_pd_K_mass"
    "airdropx_pd_bias_rate_limit"
    "airdropx_pd_throttle_kp"
    "airdropx_pd_throttle_fixed"
    "airdropx_pd_throttle_alt_kp"
    "airdropx_pd_throttle_vz_kd"
    "airdropx_pd_v_ref_mps"
    "airdropx_pd_pitch_ref_deg"
    "airdropx_pd_pitch_kp"
    "airdropx_pd_pitch_limit"
    "airdropx_pd_pitch_rate_kd"
    "airdropx_pd_pitch_rate_limit"
    "airdropx_pd_dt_s"
    "airdropx_drop_mass_signal_kg"
    "airdropx_initial_elevator_delta"]);
local_set_chart_data_scope(model, "PD_NW20", "pitch_deg", "Input");
set_param(model, "SimulationCommand", "update");
local_connect_src_to_chart_input(model, "Demux", 6, "PD_NW20", 6, "pitch_deg");

local_set_emchart_script(model, "CARP_CEP", [
    "function [drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = CARP_CEP(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count)"
    "%#codegen"
    ""
    "[drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = ..."
    "    airdropx_carp_cep_block(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, ..."
    "        wind_n_mps, wind_e_mps, drop_count, ..."
    "        airdropx_carp_target_n_m, airdropx_carp_target_e_m, ..."
    "        airdropx_carp_release_window_s, airdropx_carp_interval_s, ..."
    "        airdropx_carp_drop_total, airdropx_carp_min_safe_alt_m, ..."
    "        airdropx_ballistics_gravity_mps2, airdropx_ballistics_k_drag, ..."
    "        airdropx_ballistics_side_wind_gain);"
    ""
    "end"]);
local_ensure_chart_parameters(model, "CARP_CEP", [
    "airdropx_carp_target_n_m"
    "airdropx_carp_target_e_m"
    "airdropx_carp_release_window_s"
    "airdropx_carp_interval_s"
    "airdropx_carp_drop_total"
    "airdropx_carp_min_safe_alt_m"
    "airdropx_ballistics_gravity_mps2"
    "airdropx_ballistics_k_drag"
    "airdropx_ballistics_side_wind_gain"]);

local_set_emchart_script(model, "CargoVR", [
    "function [cargo1_translation, cargo2_translation, cargo3_translation, cargo4_translation, cargo1_scale, cargo2_scale, cargo3_scale, cargo4_scale] = CargoVR(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count)"
    "%#codegen"
    ""
    "[cargo1_translation, cargo2_translation, cargo3_translation, cargo4_translation, cargo1_scale, cargo2_scale, cargo3_scale, cargo4_scale] = ..."
    "    airdropx_vr_cargo_pose(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, ..."
    "        wind_n_mps, wind_e_mps, drop_count, ..."
    "        airdropx_ballistics_gravity_mps2, airdropx_ballistics_k_drag);"
    ""
    "end"]);
local_ensure_chart_parameters(model, "CargoVR", [
    "airdropx_ballistics_gravity_mps2"
    "airdropx_ballistics_k_drag"]);

selectorPath = model + "/DropCommandSelect";
if getSimulinkBlockHandle(selectorPath) < 0
    add_block("simulink/User-Defined Functions/MATLAB Function", selectorPath, ...
        "Position", [430 360 570 430]);
end
local_set_emchart_script(model, "DropCommandSelect", [
    "function drop_cmd = DropCommandSelect(fixed_drop_cmd, carp_drop_cmd)"
    "%#codegen"
    "if airdropx_drop_mode >= 1.5"
    "    drop_cmd = carp_drop_cmd;"
    "else"
    "    drop_cmd = fixed_drop_cmd;"
    "end"
    "end"]);
local_ensure_chart_parameters(model, "DropCommandSelect", "airdropx_drop_mode");

local_delete_line(model, "MATLAB Function/1", "Mux/5");
local_delete_line(model, "MATLAB Function1/1", "Unit Delay3/1");
local_delete_line(model, "DropCommandSelect/1", "Unit Delay3/1");
local_delete_line(model, "Unit Delay3/1", "Mux/5");
local_delete_port_lines(model + "/DropCommandSelect");
local_delete_port_lines(model + "/Unit Delay3");
local_delete_dst_line(model + "/Mux", 5);

fixedLine = add_line(model, "MATLAB Function/1", "DropCommandSelect/1", "autorouting", "on");
set_param(fixedLine, "Name", "fixed_drop_cmd");
carpLine = add_line(model, "MATLAB Function1/1", "DropCommandSelect/2", "autorouting", "on");
set_param(carpLine, "Name", "carp_drop_cmd");
selectedLine = add_line(model, "DropCommandSelect/1", "Unit Delay3/1", "autorouting", "on");
set_param(selectedLine, "Name", "selected_drop_cmd");
dropLine = add_line(model, "Unit Delay3/1", "Mux/5", "autorouting", "on");
set_param(dropLine, "Name", "drop_cmd");

save_system(model);
bdclose(model);
fprintf("Updated %s model architecture.\n", modelPath);
end

function local_set_emchart_script(model, chartName, lines)
chart = local_find_emchart(model, chartName);
chart.Script = strjoin(lines, newline);
end

function chart = local_find_emchart(model, chartName)
rt = sfroot;
charts = rt.find("-isa", "Stateflow.EMChart");
needle = "function";
for i = 1:numel(charts)
    path = string(charts(i).Path);
    script = string(charts(i).Script);
    hasFunction = contains(script, needle + " ") && contains(script, string(chartName) + "(");
    hasBlockName = string(charts(i).Name) == string(chartName);
    if startsWith(path, string(model) + "/") && (hasFunction || hasBlockName)
        chart = charts(i);
        return;
    end
end
error("Could not find MATLAB Function chart '%s' in model '%s'.", chartName, model);
end

function local_ensure_chart_parameters(model, chartName, names)
chart = local_find_emchart(model, chartName);
names = string(names);
existing = chart.find("-isa", "Stateflow.Data");
existingNames = strings(numel(existing), 1);
for i = 1:numel(existing)
    existingNames(i) = string(existing(i).Name);
end
for i = 1:numel(names)
    if any(existingNames == names(i))
        data = existing(existingNames == names(i));
        data = data(1);
    else
        data = Stateflow.Data(chart);
        data.Name = char(names(i));
    end
    data.Scope = "Parameter";
end
end

function local_set_chart_data_scope(model, chartName, dataName, scope)
chart = local_find_emchart(model, chartName);
data = chart.find("-isa", "Stateflow.Data", "Name", char(dataName));
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = char(dataName);
else
    data = data(1);
end
data.Scope = char(scope);
end

function local_connect_src_to_chart_input(model, srcBlockName, srcPortIndex, chartName, dstPortIndex, lineName)
chart = local_find_emchart(model, chartName);
srcPath = char(string(model) + "/" + string(srcBlockName));
srcPorts = get_param(srcPath, "PortHandles");
dstPorts = get_param(char(chart.Path), "PortHandles");

if numel(srcPorts.Outport) < srcPortIndex || numel(dstPorts.Inport) < dstPortIndex
    error("Cannot connect %s/%d to %s/%d: port index out of range.", ...
        srcBlockName, srcPortIndex, chartName, dstPortIndex);
end

dstLine = get_param(dstPorts.Inport(dstPortIndex), "Line");
if dstLine ~= -1
    delete_line(dstLine);
end

newLine = add_line(model, srcPorts.Outport(srcPortIndex), dstPorts.Inport(dstPortIndex), ...
    "autorouting", "on");
set_param(newLine, "Name", lineName);
end

function local_delete_line(model, src, dst)
try
    delete_line(model, src, dst);
catch
end
end

function local_delete_port_lines(blockPath)
try
    ports = get_param(blockPath, "PortHandles");
    handles = [ports.Inport(:); ports.Outport(:)];
    for i = 1:numel(handles)
        line = get_param(handles(i), "Line");
        if line ~= -1
            delete_line(line);
        end
    end
catch
end
end

function local_delete_dst_line(blockPath, portIndex)
try
    ports = get_param(blockPath, "PortHandles");
    line = get_param(ports.Inport(portIndex), "Line");
    if line ~= -1
        delete_line(line);
    end
catch
end
end
