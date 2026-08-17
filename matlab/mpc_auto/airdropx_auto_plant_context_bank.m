function result = airdropx_auto_plant_context_bank(varargin)
%AIRDROPX_AUTO_PLANT_CONTEXT_BANK Persistent cross-mission Plant/trim context bank.
%
% v30 uses this bank only to choose a good Plant/trim SEED for a new mission.
% The selected seed is never trusted blindly: the existing v29 equilibrium
% probe still decides whether each cfg Plant can be reused or must be rebuilt.
%
% Actions:
%   register - add/update one mission master Plant file
%   nearest  - find the closest reusable Plant master for a requested context
%   report   - return the current index table
%
% Example:
%   s = airdropx_auto_plant_context_bank( ...
%       "Action","nearest", ...
%       "Root","matlab/results/mpc_auto_global_plant_bank", ...
%       "TargetAltitudeM",80,"TargetAirspeedMps",47.5, ...
%       "ReferenceMassKg",3423,"CargoMassKg",300);

opts = local_options(varargin{:});
projectRoot = local_project_root(opts.ProjectRoot);
root = local_resolve_path(projectRoot, opts.Root);
if ~isfolder(root), mkdir(root); end
indexFile = fullfile(root, "plant_context_index.csv");
action = lower(string(opts.Action));

switch action
    case "register"
        masterMat = local_resolve_path(projectRoot, opts.MasterMat);
        outputRoot = local_resolve_path(projectRoot, opts.OutputRoot);
        if strlength(string(opts.MasterMat)) == 0 || ~isfile(masterMat)
            error("AirdropX:PlantContextBank:MissingMaster", ...
                "MasterMat does not exist: %s", masterMat);
        end
        row = table( ...
            string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")), ...
            double(opts.TargetAltitudeM), double(opts.TargetAirspeedMps), ...
            double(opts.ReferenceMassKg), double(opts.CargoMassKg), ...
            round(double(opts.TotalDropCount)), ...
            string(outputRoot), string(masterMat), ...
            logical(opts.AllVerified), logical(opts.MissionPass), ...
            logical(opts.PlantValid), string(opts.Source), ...
            'VariableNames', {'timestamp','target_altitude_m','target_airspeed_mps', ...
            'reference_mass_kg','cargo_mass_kg','total_drop_count','output_root', ...
            'master_mat','all_verified','mission_pass','plant_valid','source'});
        T = local_read_index(indexFile);
        if isempty(T)
            T = row;
        else
            same = abs(double(T.target_altitude_m)-double(opts.TargetAltitudeM)) < 1e-8 & ...
                abs(double(T.target_airspeed_mps)-double(opts.TargetAirspeedMps)) < 1e-8 & ...
                abs(double(T.reference_mass_kg)-double(opts.ReferenceMassKg)) < 1e-8 & ...
                abs(double(T.cargo_mass_kg)-double(opts.CargoMassKg)) < 1e-8 & ...
                round(double(T.total_drop_count)) == round(double(opts.TotalDropCount)) & ...
                string(T.output_root) == string(outputRoot);
            if any(same)
                T(find(same,1,"last"),:) = row;
                if nnz(same) > 1
                    first = find(same,1,"last");
                    remove = find(same);
                    remove(remove == first) = [];
                    T(remove,:) = [];
                end
            else
                T = [T; row]; %#ok<AGROW>
            end
        end
        writetable(T,indexFile);
        result = struct('action',"register",'index_file',string(indexFile), ...
            'registered',true,'row',row,'table',T);

    case "nearest"
        T = local_read_index(indexFile);
        if isempty(T)
            result = local_empty_nearest(indexFile,T);
            return;
        end
        keep = logical(T.plant_valid);
        for i = 1:height(T)
            p = local_resolve_path(projectRoot, string(T.master_mat(i)));
            keep(i) = keep(i) && isfile(p);
            if keep(i), T.master_mat(i) = string(p); end
        end
        T = T(keep,:);
        if isempty(T)
            result = local_empty_nearest(indexFile,T);
            return;
        end
        dV = abs(double(T.target_airspeed_mps)-double(opts.TargetAirspeedMps)) / max(double(opts.SpeedScaleMps),eps);
        dH = abs(double(T.target_altitude_m)-double(opts.TargetAltitudeM)) / max(double(opts.AltitudeScaleM),eps);
        dM = abs(double(T.reference_mass_kg)-double(opts.ReferenceMassKg)) / max(double(opts.MassScaleKg),eps);
        dC = abs(double(T.cargo_mass_kg)-double(opts.CargoMassKg)) / max(double(opts.CargoScaleKg),eps);
        dN = abs(double(T.total_drop_count)-double(opts.TotalDropCount));
        score = dV + double(opts.AltitudeWeight)*dH + ...
            double(opts.MassWeight)*dM + double(opts.CargoWeight)*dC + ...
            double(opts.DropCountWeight)*dN;
        % Prefer a complete-mission PASS when two Plant seeds are otherwise
        % very similar, but never reject a Plant-valid seed solely because its
        % final controller mission did not pass.
        score = score + double(opts.NonMissionPassPenalty) * (~logical(T.mission_pass));
        [bestScore,idx] = min(score);
        result = struct();
        result.action = "nearest";
        result.index_file = string(indexFile);
        result.found = true;
        result.distance = bestScore;
        result.master_mat = string(T.master_mat(idx));
        result.output_root = string(T.output_root(idx));
        result.row = T(idx,:);
        result.table = T;

    case "report"
        T = local_read_index(indexFile);
        result = struct('action',"report",'index_file',string(indexFile), ...
            'found',~isempty(T),'table',T);

    otherwise
        error("AirdropX:PlantContextBank:BadAction", "Unknown Action: %s", opts.Action);
end
end

function result = local_empty_nearest(indexFile,T)
result = struct('action',"nearest",'index_file',string(indexFile), ...
    'found',false,'distance',Inf,'master_mat',"",'output_root',"", ...
    'row',table(),'table',T);
end

function T = local_read_index(indexFile)
if ~isfile(indexFile)
    T = table();
    return;
end
try
    T = readtable(indexFile,'TextType','string');
catch
    T = readtable(indexFile);
end
required = {'timestamp','target_altitude_m','target_airspeed_mps','reference_mass_kg', ...
    'cargo_mass_kg','total_drop_count','output_root','master_mat','all_verified', ...
    'mission_pass','plant_valid','source'};
if ~all(ismember(required,T.Properties.VariableNames))
    error("AirdropX:PlantContextBank:BadIndex", ...
        "Plant context index has an incompatible schema: %s", indexFile);
end
end

function opts = local_options(varargin)
opts = struct();
opts.Action = "report";
opts.ProjectRoot = "";
opts.Root = "matlab/results/mpc_auto_global_plant_bank";
opts.TargetAltitudeM = 200.0;
opts.TargetAirspeedMps = 50.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.TotalDropCount = 4;
opts.OutputRoot = "";
opts.MasterMat = "";
opts.AllVerified = false;
opts.MissionPass = false;
opts.PlantValid = false;
opts.Source = "v30";
% Distance scales. Airspeed dominates Plant reuse; altitude gets deliberately
% zero weight in v31 because 20-200 m target altitude is a mission reference,
% not a Plant identity. Airspeed/mass/payload dominate Plant reuse.
opts.SpeedScaleMps = 5.0;
opts.AltitudeScaleM = 180.0;
opts.MassScaleKg = 300.0;
opts.CargoScaleKg = 100.0;
opts.AltitudeWeight = 0.0;
opts.MassWeight = 0.75;
opts.CargoWeight = 0.50;
opts.DropCountWeight = 2.0;
opts.NonMissionPassPenalty = 0.05;
if mod(numel(varargin),2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name) = varargin{i+1};
end
end

function root = local_project_root(projectRoot)
root = string(projectRoot);
if strlength(root) == 0, root = string(pwd); end
root = string(char(root));
end

function p = local_resolve_path(projectRoot,p)
p = string(p);
if strlength(p) == 0, return; end
c = char(p);
isAbs = startsWith(c,'/') || startsWith(c,'\\') || ...
    ~isempty(regexp(c,'^[A-Za-z]:[\\/]','once'));
if ~isAbs, p = string(fullfile(projectRoot,p)); end
end
