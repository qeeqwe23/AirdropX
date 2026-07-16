function app = airdropx_gui(varargin)
%AIRDROPX_GUI Interactive tuning dashboard for AirdropX Simulink runs.
%
% Usage:
%   airdropx_gui
%   airdropx_gui("ProjectRoot", "D:\path\to\AirdropX")

opts = local_options(varargin{:});
projectRoot = string(opts.ProjectRoot);
if strlength(projectRoot) == 0
    thisFile = mfilename("fullpath");
    matlabDir = string(fileparts(thisFile));
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = string(fullfile(projectRoot, "matlab"));
end

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

cfg = airdropx_sim_params("ProjectRoot", projectRoot);
lastResult = [];
controls = struct();

fig = figure("Name", "AirdropX Tuning Console", ...
    "NumberTitle", "off", ...
    "MenuBar", "none", ...
    "ToolBar", "figure", ...
    "Color", [0.96 0.97 0.98], ...
    "Position", [80 80 1380 820]);

uipanel("Parent", fig, "Units", "normalized", ...
    "Position", [0.015 0.02 0.255 0.96], ...
    "BackgroundColor", [0.985 0.985 0.99], ...
    "BorderType", "line");

uicontrol("Parent", fig, "Style", "text", "Units", "normalized", ...
    "Position", [0.03 0.925 0.22 0.035], ...
    "String", "AirdropX Parameters", ...
    "FontWeight", "bold", "FontSize", 13, ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", [0.985 0.985 0.99]);

rowY = 0.875;
rowStep = 0.038;
addNumeric("Run time (s)", "stop_time_s", cfg.sim.stop_time_s);
addNumeric("Target alt (m)", "target_altitude_m", cfg.control.target_altitude_m);
addNumeric("Initial V (m/s)", "initial_airspeed_mps", cfg.initial.airspeed_mps);
addNumeric("Initial pitch (deg)", "initial_theta_deg", cfg.initial.theta_deg);
addNumeric("Initial elevator", "initial_elevator_delta", cfg.control.initial_elevator_delta);
addNumeric("Initial throttle", "initial_throttle_cmd", cfg.control.initial_throttle_cmd);
addNumeric("Wind speed (m/s)", "wind_speed_mps", cfg.environment.wind_speed_mps);
addNumeric("Wind dir (deg)", "wind_dir_from_deg", cfg.environment.wind_dir_from_deg);
addNumeric("Drop start (s)", "fixed_drop_start_s", cfg.fixed_drop.start_s);
addNumeric("Drop interval (s)", "fixed_drop_interval_s", cfg.fixed_drop.interval_s);
addNumeric("Kp", "Kp", cfg.control.pd_gains.Kp);
addNumeric("Kd", "Kd", cfg.control.pd_gains.Kd);
addNumeric("Elev limit", "u_limit", cfg.control.pd_gains.u_limit);
addNumeric("Elev rate limit", "u_rate_limit", cfg.control.pd_gains.u_rate_limit);
addNumeric("Throttle Kp", "throttle_kp", cfg.control.pd_gains.throttle_kp);
addNumeric("Throttle alt Kp", "throttle_alt_kp", cfg.control.pd_gains.throttle_alt_kp);
addNumeric("Pitch Kp", "pitch_kp", cfg.control.pd_gains.pitch_kp);
addNumeric("Pitch limit", "pitch_limit", cfg.control.pd_gains.pitch_limit);

uicontrol("Parent", fig, "Style", "text", "Units", "normalized", ...
    "Position", [0.035 rowY 0.105 0.028], "String", "Drop mode", ...
    "HorizontalAlignment", "left", "BackgroundColor", [0.985 0.985 0.99]);
modeDrop = uicontrol("Parent", fig, "Style", "popupmenu", "Units", "normalized", ...
    "Position", [0.145 rowY 0.105 0.03], ...
    "String", {"Fixed four-drop", "CARP/CEP"}, ...
    "Value", max(1, min(2, round(cfg.drop_mode))), ...
    "Callback", @(~, ~) liveApply());
rowY = rowY - rowStep;

runName = uicontrol("Parent", fig, "Style", "edit", "Units", "normalized", ...
    "Position", [0.035 rowY 0.215 0.032], ...
    "String", char("gui_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))), ...
    "BackgroundColor", "white", "HorizontalAlignment", "left");
rowY = rowY - 0.045;

runButton = uicontrol("Parent", fig, "Style", "pushbutton", "Units", "normalized", ...
    "Position", [0.035 rowY 0.215 0.04], "String", "Run Simulation", ...
    "FontWeight", "bold", "Callback", @(~, ~) runSimulation());
rowY = rowY - 0.05;

uicontrol("Parent", fig, "Style", "pushbutton", "Units", "normalized", ...
    "Position", [0.035 rowY 0.103 0.038], "String", "Save Parameters", ...
    "Callback", @(~, ~) saveParameters());
uicontrol("Parent", fig, "Style", "pushbutton", "Units", "normalized", ...
    "Position", [0.147 rowY 0.103 0.038], "String", "Save Results", ...
    "Callback", @(~, ~) saveResults());

statusText = uicontrol("Parent", fig, "Style", "text", "Units", "normalized", ...
    "Position", [0.03 0.035 0.225 0.045], ...
    "String", "Ready. Parameter edits update the workspace.", ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", [0.985 0.985 0.99], ...
    "ForegroundColor", [0.15 0.25 0.35]);

axAltitude = axes("Parent", fig, "Units", "normalized", "Position", [0.31 0.57 0.31 0.36]);
axSpeed = axes("Parent", fig, "Units", "normalized", "Position", [0.66 0.57 0.31 0.36]);
axControl = axes("Parent", fig, "Units", "normalized", "Position", [0.31 0.19 0.31 0.31]);
axDrop = axes("Parent", fig, "Units", "normalized", "Position", [0.66 0.19 0.31 0.31]);
initAxes();

dropTable = uitable("Parent", fig, "Units", "normalized", ...
    "Position", [0.31 0.02 0.43 0.13], "Data", cell(0, 0));
summaryBox = uicontrol("Parent", fig, "Style", "listbox", "Units", "normalized", ...
    "Position", [0.76 0.02 0.21 0.13], ...
    "String", {"No run yet.", "Edit parameters and run the simulation."}, ...
    "BackgroundColor", "white");

liveApply();

app = struct();
app.Figure = fig;
app.ProjectRoot = projectRoot;
app.GetOverrides = @collectOverrides;
app.Run = @runSimulation;

    function addNumeric(labelText, fieldName, value)
        uicontrol("Parent", fig, "Style", "text", "Units", "normalized", ...
            "Position", [0.035 rowY 0.105 0.028], ...
            "String", labelText, "HorizontalAlignment", "left", ...
            "BackgroundColor", [0.985 0.985 0.99]);
        controls.(fieldName) = uicontrol("Parent", fig, "Style", "edit", "Units", "normalized", ...
            "Position", [0.145 rowY 0.105 0.03], ...
            "String", sprintf("%.6g", double(value)), ...
            "BackgroundColor", "white", ...
            "HorizontalAlignment", "right", ...
            "Callback", @(~, ~) liveApply());
        rowY = rowY - rowStep;
    end

    function initAxes()
        title(axAltitude, "Altitude"); xlabel(axAltitude, "time (s)"); ylabel(axAltitude, "m"); grid(axAltitude, "on");
        title(axSpeed, "Speed / Vertical Speed"); xlabel(axSpeed, "time (s)"); ylabel(axSpeed, "m/s"); grid(axSpeed, "on");
        title(axControl, "Control"); xlabel(axControl, "time (s)"); ylabel(axControl, "norm"); grid(axControl, "on");
        title(axDrop, "Drops"); xlabel(axDrop, "time (s)"); ylabel(axDrop, "count"); grid(axDrop, "on");
    end

    function overrides = collectOverrides()
        names = string(fieldnames(controls));
        overrides = struct();
        for i = 1:numel(names)
            key = char(names(i));
            value = str2double(get(controls.(key), "String"));
            if ~isfinite(value)
                error("Invalid numeric value for %s.", key);
            end
            overrides.(key) = value;
        end
        overrides.drop_mode = get(modeDrop, "Value");
    end

    function liveApply()
        try
            overrides = collectOverrides();
            assignin("base", "airdropx_gui_overrides", overrides);
            assignin("base", "airdropx_stop_time_s", overrides.stop_time_s);
            assignin("base", "airdropx_drop_mode", overrides.drop_mode);
            assignin("base", "airdropx_target_altitude_m", overrides.target_altitude_m);
            assignin("base", "airdropx_initial_airspeed_mps", overrides.initial_airspeed_mps);
            assignin("base", "airdropx_initial_theta_deg", overrides.initial_theta_deg);
            assignin("base", "airdropx_initial_pitch_deg", overrides.initial_theta_deg);
            assignin("base", "airdropx_initial_elevator_delta", overrides.initial_elevator_delta);
            assignin("base", "airdropx_initial_throttle_cmd", overrides.initial_throttle_cmd);
            assignin("base", "airdropx_wind_speed_mps", overrides.wind_speed_mps);
            assignin("base", "airdropx_wind_dir_from_deg", overrides.wind_dir_from_deg);
            assignin("base", "airdropx_fixed_drop_start_s", overrides.fixed_drop_start_s);
            assignin("base", "airdropx_fixed_drop_interval_s", overrides.fixed_drop_interval_s);
            assignin("base", "airdropx_pd_Kp", overrides.Kp);
            assignin("base", "airdropx_pd_Kd", overrides.Kd);
            assignin("base", "airdropx_pd_u_limit", overrides.u_limit);
            assignin("base", "airdropx_pd_u_rate_limit", overrides.u_rate_limit);
            assignin("base", "airdropx_pd_throttle_kp", overrides.throttle_kp);
            assignin("base", "airdropx_pd_throttle_alt_kp", overrides.throttle_alt_kp);
            assignin("base", "airdropx_pd_pitch_kp", overrides.pitch_kp);
            assignin("base", "airdropx_pd_pitch_limit", overrides.pitch_limit);
            set(statusText, "String", "Parameters updated in workspace.");
        catch err
            set(statusText, "String", err.message);
        end
    end

    function runSimulation()
        try
            overrides = collectOverrides();
            set(runButton, "Enable", "off");
            set(statusText, "String", "Running Simulink simulation...");
            set(summaryBox, "String", {"Running...", "MATLAB is busy until Simulink finishes."});
            drawnow;
            result = airdropx_run_and_export( ...
                "ProjectRoot", projectRoot, ...
                "RunName", string(get(runName, "String")), ...
                "Overrides", overrides);
            lastResult = result;
            updateResultViews(result);
            set(statusText, "String", char("Run complete: " + result.output_dir));
        catch err
            set(statusText, "String", "Run failed.");
            set(summaryBox, "String", cellstr(splitlines(string(getReport(err, "extended", "hyperlinks", "off")))));
            errordlg(err.message, "AirdropX run failed");
        end
        set(runButton, "Enable", "on");
    end

    function updateResultViews(result)
        T = result.timeseries;
        axes(axAltitude); cla(axAltitude);
        plot(axAltitude, T.time_s, T.altitude_m, "LineWidth", 1.4); hold(axAltitude, "on");
        yline(axAltitude, result.cfg.control.target_altitude_m, "--"); hold(axAltitude, "off");
        title(axAltitude, "Altitude"); xlabel(axAltitude, "time (s)"); ylabel(axAltitude, "m"); grid(axAltitude, "on");

        axes(axSpeed); cla(axSpeed);
        plot(axSpeed, T.time_s, T.airspeed_mps, "LineWidth", 1.3); hold(axSpeed, "on");
        plot(axSpeed, T.time_s, T.vz_up_mps, "LineWidth", 1.3); hold(axSpeed, "off");
        title(axSpeed, "Speed / Vertical Speed"); xlabel(axSpeed, "time (s)"); ylabel(axSpeed, "m/s");
        legend(axSpeed, {"airspeed", "vz up"}, "Location", "best"); grid(axSpeed, "on");

        axes(axControl); cla(axControl);
        plot(axControl, T.time_s, T.elevator_cmd_norm, "LineWidth", 1.3); hold(axControl, "on");
        plot(axControl, T.time_s, T.throttle_norm, "LineWidth", 1.3);
        plot(axControl, T.time_s, T.u_out, "LineWidth", 1.1); hold(axControl, "off");
        title(axControl, "Control"); xlabel(axControl, "time (s)"); ylabel(axControl, "norm");
        legend(axControl, {"elevator", "throttle", "u out"}, "Location", "best"); grid(axControl, "on");

        axes(axDrop); cla(axDrop);
        stairs(axDrop, T.time_s, T.drop_count, "LineWidth", 1.5);
        title(axDrop, "Drops"); xlabel(axDrop, "time (s)"); ylabel(axDrop, "count"); ylim(axDrop, [-0.2 4.4]); grid(axDrop, "on");

        set(dropTable, "Data", table2cell(result.drop_table), ...
            "ColumnName", result.drop_table.Properties.VariableNames);

        r = result.report;
        set(summaryBox, "String", cellstr([
            "Output: " + result.output_dir
            "Drops: " + string(r.drop_count_final)
            "Drop times: " + mat2str(r.drop_times, 4)
            "Mean altitude error (m): " + sprintf("%.4f", r.h_err_mean)
            "RMS altitude error (m): " + sprintf("%.4f", r.h_err_rms)
            "Max altitude error (m): " + sprintf("%.4f", r.h_err_max)
            "Min altitude (m): " + sprintf("%.4f", r.min_altitude)
            "Max altitude (m): " + sprintf("%.4f", r.max_altitude)
            "Elevator saturation: " + sprintf("%.2f %%", 100.0 * r.elevator_sat_rate)
            ]));
    end

    function saveParameters()
        try
            overrides = collectOverrides();
            [file, path] = uiputfile("*.csv", "Save AirdropX parameters", ...
                fullfile(char(projectRoot), "matlab", "results", "airdropx_gui_params.csv"));
            if isequal(file, 0), return; end
            writetable(local_overrides_table(overrides), fullfile(path, file));
            set(statusText, "String", char("Parameters saved: " + string(fullfile(path, file))));
        catch err
            errordlg(err.message, "Save parameters failed");
        end
    end

    function saveResults()
        if isempty(lastResult)
            errordlg("Run the simulation before saving results.", "No results");
            return;
        end
        dest = uigetdir(char(projectRoot), "Choose a folder for result copy");
        if isequal(dest, 0), return; end
        target = fullfile(dest, "airdropx_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")));
        copyfile(char(lastResult.output_dir), char(target));
        set(statusText, "String", char("Results copied: " + string(target)));
    end
end

function T = local_overrides_table(overrides)
names = string(fieldnames(overrides));
values = zeros(numel(names), 1);
for i = 1:numel(names)
    values(i) = double(overrides.(char(names(i))));
end
T = table(names, values, "VariableNames", ["name", "value"]);
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
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
