function report = airdropx_release_check(varargin)
%AIRDROPX_RELEASE_CHECK Check that a downloaded MATLAB package is runnable.
%
% Usage:
%   report = airdropx_release_check
%   report = airdropx_release_check("RunSmoke", true)

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
matlabDir = string(fileparts(thisFile));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

checks = strings(0, 1);
ok = true;

[ok, checks] = local_check_file(ok, checks, fullfile(projectRoot, "matlab", "untitled1.slx"), "Simulink model");
[ok, checks] = local_check_file(ok, checks, fullfile(projectRoot, "aircraft", "MQ9_Reaper", "MQ9_Reaper.xml"), "aircraft model");
[ok, checks] = local_check_file(ok, checks, fullfile(projectRoot, "aircraft", "MQ9_Reaper", "reset_20m.xml"), "initial-condition template");
[ok, checks] = local_check_file(ok, checks, fullfile(projectRoot, "engine", "TPE331-10.xml"), "MQ-9 engine model");
[ok, checks] = local_check_file(ok, checks, fullfile(projectRoot, "engine", "direct.xml"), "JSBSim direct thruster model");
[ok, checks] = local_check_dir(ok, checks, fullfile(projectRoot, "third_party", "JSBSim"), "JSBSim source tree");
[ok, checks] = local_check_dir(ok, checks, fullfile(projectRoot, "third_party", "jsbsim-win64", "include"), "bundled JSBSim include directory");
[ok, checks] = local_check_dir(ok, checks, fullfile(projectRoot, "third_party", "jsbsim-win64", "lib"), "bundled JSBSim library directory");

mexFile = fullfile(projectRoot, "matlab", "sfunc_jsbsim", "sfun_airdropx_jsbsim." + mexext);
[ok, checks] = local_check_file(ok, checks, mexFile, "JSBSim S-Function MEX");

try
    cfg = airdropx_sim_params("ProjectRoot", projectRoot, "AssignBase", true);
    checks(end + 1) = "PASS: airdropx_sim_params initialized workspace variables.";
catch err
    ok = false;
    checks(end + 1) = "FAIL: airdropx_sim_params failed: " + string(err.message);
    cfg = [];
end

if ~isempty(cfg)
    generatedIc = string(cfg.icName) + ".xml";
    [ok, checks] = local_check_file(ok, checks, generatedIc, "generated runtime initial condition");
end

if opts.RunSmoke
    try
        smoke = airdropx_run_and_export("ProjectRoot", projectRoot, "RunName", "release_smoke");
        if smoke.report.drop_count_final == 4
            checks(end + 1) = "PASS: smoke simulation completed 4 drops.";
        else
            ok = false;
            checks(end + 1) = "FAIL: smoke simulation drop_count_final = " + string(smoke.report.drop_count_final);
        end
    catch err
        ok = false;
        checks(end + 1) = "FAIL: smoke simulation failed: " + string(err.message);
    end
end

for i = 1:numel(checks)
    fprintf("%s\n", checks(i));
end

if ok
    fprintf("AirdropX release check PASSED.\n");
else
    fprintf("AirdropX release check FAILED.\n");
end

report = struct();
report.ok = ok;
report.projectRoot = projectRoot;
report.checks = checks;

if ~ok
    error("AirdropX release check failed. See messages above.");
end
end

function opts = local_options(varargin)
opts.RunSmoke = false;

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

function [ok, checks] = local_check_file(ok, checks, path, label)
if isfile(path)
    checks(end + 1) = "PASS: " + string(label) + " found.";
else
    ok = false;
    checks(end + 1) = "FAIL: " + string(label) + " missing: " + string(path);
end
end

function [ok, checks] = local_check_dir(ok, checks, path, label)
if isfolder(path)
    checks(end + 1) = "PASS: " + string(label) + " found.";
else
    ok = false;
    checks(end + 1) = "FAIL: " + string(label) + " missing: " + string(path);
end
end
