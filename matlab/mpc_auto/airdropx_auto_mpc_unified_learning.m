function result = airdropx_auto_mpc_unified_learning(varargin)
%AIRDROPX_AUTO_MPC_UNIFIED_LEARNING Generic v29 context-aware self-learning entry.
%
% This wrapper keeps the proven v16-v28 engine/checkpoint format while making
% the v29 UnifiedLearning path explicit.  The same function is intended for
% later target-altitude, target-airspeed and payload-mass missions.
%
% Example:
%   r = airdropx_auto_mpc_unified_learning( ...
%       "IdentifiedMat","matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat", ...
%       "OutputRoot","matlab/results/mpc_auto_H200_V50_Cargo300_v29", ...
%       "LearningBankRoot","matlab/results/mpc_auto_global_learning_bank", ...
%       "TargetAltitudeM",200,"TargetAirspeedMps",50,"CargoMassKg",300, ...
%       "ConfigIds",(0:4).',"UseParallel",true,"ParallelWorkers",5);
%
% For a NEW mission, change OutputRoot but KEEP LearningBankRoot.  Verified
% controllers and v29 evaluations in the shared bank become transfer data.

args = [{"UnifiedLearning",true}, varargin];
result = airdropx_auto_mpc_200m_all_configs(args{:});
end
