function result=airdropx_urmpc_v21_corrected_flight_ablation(varargin)
%AIRDROPX_URMPC_V21_CORRECTED_FLIGHT_ABLATION
% V2.1C causal flight experiment: corrected A/B only, NO tube tightening.
%
% This runner never overwrites the v2.0 flight bank. It first requires the
% independent v2.1B offline candidate to have passed:
%   15/15 projected blocked validation
%   55/55 corrected vertex preflight
%   12/12 corrected cfg-transition certification
% Then it runs the existing fixed-stability real JSBSim mission using the
% corrected candidate bank. All MPC costs, horizons, estimator policy,
% physical MV limits and soft rate-trust constraints remain those stored in
% the candidate bank. No tube feedback or constraint tightening is added.

o=local_options(varargin{:});root=local_root(o.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,o.CandidateBankMat);certRoot=local_resolve(root,o.CertRoot);outRoot=local_resolve(root,o.OutputRoot);
if ~isfile(bank),error('AirdropX:URMPC:MissingV21Candidate','Missing v2.1B corrected candidate bank: %s',bank);end
if ~isfolder(certRoot),error('AirdropX:URMPC:MissingV21CertificateRoot','Missing v2.1B certificate directory: %s',certRoot);end

B=load(bank,'ur_mpc','ur_models','ur_meta');
if ~isfield(B,'ur_mpc')||~isfield(B,'ur_models')||~isfield(B,'ur_meta'),error('AirdropX:URMPC:BadV21Candidate','Candidate bank missing ur_mpc/ur_models/ur_meta.');end
if ~isfield(B.ur_meta,'v21_deploy_ready')||~logical(B.ur_meta.v21_deploy_ready),error('AirdropX:URMPC:V21CandidateNotReady','Candidate bank ur_meta.v21_deploy_ready is not true.');end
if ~contains(lower(string(B.ur_meta.version)),'v2_1b_corrected_candidate'),error('AirdropX:URMPC:WrongV21Candidate','Unexpected candidate version: %s',string(B.ur_meta.version));end

summaryPath=fullfile(certRoot,'urmpc_v21b_summary.csv');
projPath=fullfile(certRoot,'urmpc_v21_projected_validation.csv');
vertexPath=fullfile(certRoot,'urmpc_v21_corrected_vertex_preflight.csv');
transPath=fullfile(certRoot,'urmpc_v21_corrected_linear_drop_cert.csv');
wPath=fullfile(certRoot,'urmpc_v21_corrected_w_envelope.csv');
req={summaryPath,projPath,vertexPath,transPath,wPath};for i=1:numel(req),if ~isfile(req{i}),error('AirdropX:URMPC:MissingV21Certificate','Missing v2.1B artifact: %s',req{i});end,end
S=readtable(summaryPath);P=readtable(projPath);V=readtable(vertexPath);T=readtable(transPath);W=readtable(wPath);
if height(S)~=1||~logical(S.deploy_ready(1))||~all(string(P.status)=="PASS")||~all(logical(V.pass))||~all(logical(T.pass))
    error('AirdropX:URMPC:V21OfflineGateFailed','v2.1B offline certificates are not all PASS. Refusing nonlinear flight.');
end

% Audit p95 residual envelope only. This is intentionally NOT used as a
% deterministic hard tube set in v2.1C.
wnames={'w_norm_h_p95_abs','w_norm_va_p95_abs','w_norm_pitch_p95_abs','w_norm_vz_p95_abs','w_norm_q_p95_abs'};
wmax=NaN(1,5);for j=1:5,if ismember(wnames{j},W.Properties.VariableNames),wmax(j)=max(double(W.(wnames{j})),[],'omitnan');end,end
fprintf('\n[UR-MPC v2.1C CORRECTED-MODEL FLIGHT ABLATION]\n');
fprintf('  v2.1B gates: projected %d/%d; vertex %d/%d; transition %d/%d; deploy_ready=1\n',sum(string(P.status)=="PASS"),height(P),sum(V.pass),height(V),sum(T.pass),height(T));
fprintf('  corrected residual p95 max (normalized): H=%.6g Va=%.6g pitch=%.6g vz=%.6g q=%.6g\n',wmax(1),wmax(2),wmax(3),wmax(4),wmax(5));
fprintf('  experiment: corrected A/B ONLY; no tube tightening; no recovery/TECS/H-PI.\n');
fprintf('  candidate bank: %s\n',bank);

if isfolder(outRoot)&&logical(o.CleanOutputRoot),rmdir(outRoot,'s');end
if ~isfolder(outRoot),mkdir(outRoot);end
result=airdropx_urmpc_fixed_stability_scan('ProjectRoot',root,'BankMat',bank,'OutputRoot',outRoot, ...
    'ShortFileGenRoot',o.ShortFileGenRoot,'JobStorageRoot',o.JobStorageRoot,'RequireDDriveTemp',o.RequireDDriveTemp, ...
    'AircraftName',o.AircraftName,'SpeedsMps',o.SpeedsMps,'TargetAltitudeM',o.TargetAltitudeM,'StopTimeS',o.StopTimeS, ...
    'DropStartS',o.DropStartS,'DropIntervalS',o.DropIntervalS,'AfterDropTime',o.AfterDropTime,'ParallelWorkers',o.ParallelWorkers);

% Save an explicit experiment manifest beside the normal fixed-stability
% outputs so v2.1C cannot be confused with v2.0.x runs.
M=table(string(B.ur_meta.version),string(bank),true,true,true,true,wmax(1),wmax(2),wmax(3),wmax(4),wmax(5), ...
    'VariableNames',{'candidate_version','candidate_bank','projected_validation_pass','vertex_preflight_pass','linear_transition_pass','tube_enabled', ...
    'w_norm_h_p95_max','w_norm_va_p95_max','w_norm_pitch_p95_max','w_norm_vz_p95_max','w_norm_q_p95_max'});
% tube_enabled is intentionally false; keep explicit logical value here.
M.tube_enabled(:)=false;
writetable(M,fullfile(outRoot,'urmpc_v21c_experiment_manifest.csv'));
result.v21c_manifest=M;result.candidate_bank=string(bank);result.tube_enabled=false;
end

function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";
o.CandidateBankMat="matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate/airdropx_urmpc_v21_corrected_candidate.mat";
o.CertRoot="matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate";
o.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_urmpc_v21c_corrected_only";
o.ShortFileGenRoot="D:\\AXC\\urmpc_v21c";o.JobStorageRoot="D:\\MATLAB_TEMP\\AirdropX_urmpc_v21c\\jobs";o.RequireDDriveTemp=true;
o.AircraftName="MQ9_Reaper";o.SpeedsMps=50;o.TargetAltitudeM=200;o.StopTimeS=255;o.DropStartS=50;o.DropIntervalS=50;o.AfterDropTime=10;o.ParallelWorkers=1;o.CleanOutputRoot=true;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
