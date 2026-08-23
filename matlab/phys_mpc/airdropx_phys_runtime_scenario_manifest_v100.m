function T=airdropx_phys_runtime_scenario_manifest_v100(outputPath)
%AIRDROPX_PHYS_RUNTIME_SCENARIO_MANIFEST_V100 Six uninterrupted moving-command missions.
Scenario=["altitude_down_v45";"altitude_up_v65";"speed_up_h20";"speed_down_h200";"coupled_low_to_high";"coupled_high_to_low"];
Duration_s=150*ones(6,1); PreviewMode=repmat("known_reference",6,1); T=table(Scenario,Duration_s,PreviewMode);
if nargin>0 && strlength(string(outputPath))>0, writetable(T,string(outputPath)); end
end
