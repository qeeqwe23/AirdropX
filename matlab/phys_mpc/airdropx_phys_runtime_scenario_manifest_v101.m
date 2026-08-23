function T=airdropx_phys_runtime_scenario_manifest_v101(outputPath)
%AIRDROPX_PHYS_RUNTIME_SCENARIO_MANIFEST_V101 Six full-range dynamic-feasible reference missions.
Scenario=["altitude_down_v45";"altitude_up_v65";"speed_up_h20";"speed_down_h200";"coupled_low_to_high";"coupled_high_to_low"];
Duration_s=150*ones(6,1); PreviewMode=repmat("known_reference",6,1); ReferenceMode=repmat("dynamic_feasible",6,1); T=table(Scenario,Duration_s,PreviewMode,ReferenceMode);
if nargin>0 && strlength(string(outputPath))>0, writetable(T,string(outputPath)); end
end
