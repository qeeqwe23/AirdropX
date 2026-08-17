function report = airdropx_auto_learning_bank_report(varargin)
%AIRDROPX_AUTO_LEARNING_BANK_REPORT Summarize the persistent v29 LearningBank.
%
% Example:
%   R = airdropx_auto_learning_bank_report( ...
%       "LearningBankRoot","matlab/results/mpc_auto_global_learning_bank");
%
% This is a read-only helper. It does not modify checkpoints/controllers.

p = inputParser;
addParameter(p,"LearningBankRoot","matlab/results/mpc_auto_global_learning_bank");
parse(p,varargin{:});
root = string(p.Results.LearningBankRoot);
if ~isfolder(root)
    error("AirdropX:UnifiedLearner:LearningBankMissing", ...
        "LearningBank directory does not exist: %s",root);
end

verifiedFile = fullfile(root,"verified_controllers.csv");
evalFile = fullfile(root,"evaluations.csv");
V = table(); E = table();
if isfile(verifiedFile)
    V = readtable(verifiedFile,'VariableNamingRule','preserve','TextType','string');
end
if isfile(evalFile)
    E = readtable(evalFile,'VariableNamingRule','preserve','TextType','string');
end

fprintf("\n[V29 LearningBank] %s\n",root);
fprintf("  verified controller records : %d\n",height(V));
fprintf("  full-certification records  : %d\n",height(E));

verifiedContexts = 0; missions = 0; formalEvals = 0;
if ~isempty(V) && ismember("context_signature",string(V.Properties.VariableNames))
    verifiedContexts = numel(unique(string(V.context_signature)));
end
if ~isempty(V) && ismember("mission_signature",string(V.Properties.VariableNames))
    missions = numel(unique(string(V.mission_signature)));
elseif ~isempty(E) && ismember("mission_signature",string(E.Properties.VariableNames))
    missions = numel(unique(string(E.mission_signature)));
end
if ~isempty(E) && ismember("formal_pass",string(E.Properties.VariableNames))
    formalEvals = nnz(local_bool(E.formal_pass));
end
fprintf("  distinct verified contexts  : %d\n",verifiedContexts);
fprintf("  distinct mission signatures : %d\n",missions);
fprintf("  formal-pass eval records     : %d\n",formalEvals);

best = table();
if ~isempty(E) && all(ismember(["context_signature","gate_ratio"],string(E.Properties.VariableNames)))
    g = local_num(E.gate_ratio);
    valid = isfinite(g);
    T = E(valid,:);
    if ~isempty(T)
        g = local_num(T.gate_ratio);
        [~,ord] = sort(g,'ascend');
        T = T(ord,:);
        [~,ia] = unique(string(T.context_signature),'stable');
        best = T(ia,:);
        keep = intersect(["mission_signature","context_signature","config_id", ...
            "target_altitude_m","target_airspeed_mps","estimated_mass_kg","cargo_mass_kg", ...
            "gate_ratio","formal_pass","Np","Nc","Wh","Wvz","Wq","RateScale", ...
            "Authority","HeightToVzGain","HeightIntegralGain","HeightVzLimit"], ...
            string(best.Properties.VariableNames),'stable');
        best = best(:,cellstr(keep));
        fprintf("\n  Best learned evaluation per context:\n");
        disp(best);
    end
end

report = struct();
report.learning_bank_root = root;
report.verified_records = V;
report.evaluation_records = E;
report.best_per_context = best;
report.verified_context_count = verifiedContexts;
report.mission_count = missions;
report.formal_evaluation_count = formalEvals;
end

function v = local_num(x)
if isnumeric(x) || islogical(x)
    v = double(x);
else
    v = str2double(string(x));
end
end

function v = local_bool(x)
if islogical(x)
    v = x;
elseif isnumeric(x)
    v = x~=0;
else
    s = lower(strtrim(string(x)));
    v = s=="true" | s=="1" | s=="yes";
end
end
