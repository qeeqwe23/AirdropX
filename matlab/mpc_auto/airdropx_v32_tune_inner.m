function result=airdropx_v32_tune_inner(varargin)
%AIRDROPX_V32_TUNE_INNER Learn ONE inner-MPC hyperparameter set for all speed/cfg nodes.
opts=local_options(varargin{:});root=string(opts.OutputRoot);if ~isfolder(root),mkdir(root);end
P=double(opts.Profile);if isempty(P),P=[0 50 0;20 45 -0.8;50 55 0.8;80 50 -2.0;115 50 2.0;150 50 0];end
resumeFile=fullfile(root,'inner_resume_checkpoint.mat');searchFile=fullfile(root,'inner_search_records.csv');
records=table();
bestGate=Inf;bestParams=struct();resumeAttempt=0;
if isfile(resumeFile)
    try,S=load(resumeFile);if isfield(S,'resume'),resumeAttempt=double(S.resume.attempt_count);bestGate=double(S.resume.best_gate);bestParams=S.resume.best_params;end,catch,end
end
resumeMode=isfinite(bestGate) && ~isempty(fieldnames(bestParams));
if resumeMode
    fprintf('[V32.1-INNER] resume memory found: previous best gate=%.3f attempt=%d. Running local resume candidates before new BO.\n',bestGate,resumeAttempt);
    C=local_resume_candidates(bestParams);
else
    C=local_seed_candidates();
end
for i=1:height(C)
    p=local_row(C(i,:));if resumeMode,src="resume_local";else,src="seed";end
    [g,rows]=local_eval(p,opts,P,src+"_"+i);r0=local_record(p,g,src);if isempty(records),records=r0;else,records=[records;r0];end %#ok<AGROW>
    if g<bestGate,bestGate=g;bestParams=p;end
    fprintf('[V32-INNER] seed %d/%d gate=%.3f\n',i,height(C),g);
end
local_append_records(searchFile,records);
resume=struct('attempt_count',resumeAttempt+1,'best_gate',bestGate,'best_params',bestParams,'formal_pass',false,'updated_at',datetime('now')); %#ok<NASGU>
save(resumeFile,'resume','-v7');
if bestGate>1
    vars=[optimizableVariable('Np',[15 36],'Type','integer'),optimizableVariable('Nc',[3 7],'Type','integer'),...
        optimizableVariable('Wva',[4 18],'Transform','log'),optimizableVariable('Wvz',[20 120],'Transform','log'),...
        optimizableVariable('Wq',[0.4 8],'Transform','log'),optimizableVariable('WrateElev',[0.25 6],'Transform','log'),optimizableVariable('WrateThrottle',[0.25 5],'Transform','log')];
    obj=@(X)local_bayes_objective(X,opts,P);
    fprintf('[V32-INNER] deterministic seeds did not pass; starting bounded global BO (%d evaluations, max 3 workers).\n',opts.BayesEvaluations);
    bo=bayesopt(obj,vars,'MaxObjectiveEvaluations',opts.BayesEvaluations,'IsObjectiveDeterministic',true,'UseParallel',logical(opts.UseParallel),'Verbose',1,'AcquisitionFunctionName','expected-improvement-plus');
    X=bestPoint(bo);p=local_row(X);g=bo.MinObjective;
    save(fullfile(root,'inner_bayesopt.mat'),'bo','-v7.3');
    if g<bestGate,bestGate=g;bestParams=p;end
    resume=struct('attempt_count',resumeAttempt+1,'best_gate',bestGate,'best_params',bestParams,'formal_pass',false,'updated_at',datetime('now')); %#ok<NASGU>
    save(resumeFile,'resume','-v7');
end
bestBank=fullfile(root,'best_inner_bank.mat');local_build(bestParams,opts,bestBank);
[formalGate,formalRows]=local_validate_bank(bestBank,opts,P,'formal');
formalPass=formalGate<=1;
writetable(formalRows,fullfile(root,'inner_formal_validation.csv'));
writetable(local_record(bestParams,formalGate,"formal"),fullfile(root,'best_inner_candidate.csv'));
result=struct('pass',formalPass,'gate_ratio',formalGate,'params',bestParams,'bank_mat',string(bestBank),'validation',formalRows);
resume=struct('attempt_count',resumeAttempt+1,'best_gate',formalGate,'best_params',bestParams,'formal_pass',formalPass,'updated_at',datetime('now')); %#ok<NASGU>
save(resumeFile,'resume','-v7');
if ~formalPass,error('AirdropX:V32:InnerBlocked','Clean inner MPC did not pass bounded learning. Best gate %.3f; v32 resume checkpoint saved.',formalGate);end
fprintf('[V32-INNER] FORMAL PASS gate=%.3f. Va/vz inner layer certified before height learning.\n',formalGate);
end

function y=local_bayes_objective(X,o,P)
p=local_row(X);[y,~]=local_eval(p,o,P,"bo");if ~isfinite(y),y=50;end
end
function [g,rows]=local_eval(p,o,P,label)
local_worker_isolation(o.ShortFileGenRoot);
id=string(label)+"_"+string(char(java.util.UUID.randomUUID()));dir=fullfile(o.OutputRoot,'evaluations',char(id));if ~isfolder(dir),mkdir(dir);end
bank=fullfile(dir,'bank.mat');local_build(p,o,bank);
[g,rows]=local_validate_bank(bank,o,P,char(id));
writetable(rows,fullfile(dir,'cfg_summary.csv'));writetable(local_record(p,g,string(label)),fullfile(dir,'candidate.csv'));
end
function [g,rows]=local_validate_bank(bank,o,P,label)
rows=table();g=0;
for cfg=double(o.ConfigIds(:)).'
    out=fullfile(o.OutputRoot,'runtime',sprintf('%s_cfg%d',label,cfg));
    try
        r=airdropx_v32_inner_validation('ProjectRoot',o.ProjectRoot,'BankMat',bank,'OutputRoot',out,'CaseId',sprintf('v32_inner_cfg%d',cfg),...
            'ConfigId',cfg,'Profile',P,'InitialAltitudeM',o.ReferenceAltitudeM,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'HiddenElevatorTrim',o.HiddenElevatorTrim);
        row=r.summary;row.config_id=repmat(cfg,height(row),1);row.infrastructure_fail=false(height(row),1);if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,double(r.gate_ratio)); %#ok<AGROW>
    catch ME
        % One clean retry for transient Simulink worker errors. A repeated failure
        % is kept out of the learned optimum by a large deterministic penalty.
        try
            pause(0.2);r=airdropx_v32_inner_validation('ProjectRoot',o.ProjectRoot,'BankMat',bank,'OutputRoot',char(string(out)+"_retry"),'CaseId',sprintf('v32_inner_cfg%d_retry',cfg),...
                'ConfigId',cfg,'Profile',P,'InitialAltitudeM',o.ReferenceAltitudeM,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'HiddenElevatorTrim',o.HiddenElevatorTrim);
            row=r.summary;row.config_id=repmat(cfg,height(row),1);row.infrastructure_fail=false(height(row),1);if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,double(r.gate_ratio)); %#ok<AGROW>
        catch ME2
            row=table(false,50,Inf,Inf,Inf,Inf,Inf,Inf,-Inf,string(ME2.message),cfg,true,'VariableNames',...
                {'pass','gate_ratio','max_tail_V_rms_mps','max_tail_vz_rms_mps','max_abs_q_dps','max_pitch_dev_deg','max_V_settle_s','max_vz_settle_s','min_altitude_m','status','config_id','infrastructure_fail'});
            if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,50); %#ok<AGROW>
        end
    end
end
end
function local_build(p,o,file)
airdropx_v32_build_bank('IdentifiedMats',o.IdentifiedMats,'SpeedNodesMps',o.SpeedNodesMps,'OutputMat',file,...
    'Np',p.Np,'Nc',p.Nc,'Wva',p.Wva,'Wvz',p.Wvz,'Wq',p.Wq,'Wpitch',o.Wpitch,'WrateElev',p.WrateElev,'WrateThrottle',p.WrateThrottle,...
    'WmvElev',o.WmvElev,'WmvThrottle',o.WmvThrottle,'ElevatorDeviationLimit',o.ElevatorDeviationLimit,'ThrottleDeviationLimit',o.ThrottleDeviationLimit,...
    'ElevatorDeviationRateLimit',o.ElevatorDeviationRateLimit,'ThrottleDeviationRateLimit',o.ThrottleDeviationRateLimit);
end

function C=local_resume_candidates(p)
% Cross-start memory: refine around the previous v32 best instead of replaying
% the original deterministic seeds.
M=[1 1 1 1 1 1 1; ...
   1 1 1 1.25 0.85 0.80 1.00; ...
   1 1 1 1.50 0.70 0.65 1.00; ...
   1 1 0.85 1.20 1.15 1.15 0.85; ...
   1 1 1.15 0.90 0.85 0.85 1.15; ...
   1 1 1.00 1.35 1.00 0.55 0.75];
Np=[p.Np;p.Np;max(15,p.Np-2);min(36,p.Np+2);p.Np;p.Np];
Nc=[p.Nc;p.Nc;p.Nc;p.Nc;max(3,p.Nc-1);min(7,p.Nc+1)];
C=table(Np,Nc,...
 max(4,min(18,p.Wva*M(:,3))),max(20,min(120,p.Wvz*M(:,4))),max(0.4,min(8,p.Wq*M(:,5))),...
 max(0.25,min(6,p.WrateElev*M(:,6))),max(0.25,min(5,p.WrateThrottle*M(:,7))),...
 'VariableNames',{'Np','Nc','Wva','Wvz','Wq','WrateElev','WrateThrottle'});
end

function C=local_seed_candidates()
C=table([24;30;20;28;18;34],[5;5;4;6;4;6],[8;8;10;12;6;10],[55;85;45;70;75;110],[2;1.3;2.5;0.8;3;0.6],[1.0;0.6;1.6;0.45;0.8;0.3],[1.2;0.8;1.5;0.7;2.0;0.5],...
 'VariableNames',{'Np','Nc','Wva','Wvz','Wq','WrateElev','WrateThrottle'});
end
function p=local_row(X)
p=struct();names={'Np','Nc','Wva','Wvz','Wq','WrateElev','WrateThrottle'};for i=1:numel(names),p.(names{i})=double(X.(names{i})(1));end;p.Np=round(p.Np);p.Nc=round(p.Nc);
end
function T=local_record(p,g,src)
T=table(string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')),string(src),p.Np,p.Nc,p.Wva,p.Wvz,p.Wq,p.WrateElev,p.WrateThrottle,g,g<=1,...
 'VariableNames',{'timestamp','source','Np','Nc','Wva','Wvz','Wq','WrateElev','WrateThrottle','gate_ratio','pass'});
end
function local_append_records(file,T)
if isempty(T),return;end
try
    if isfile(file)
        writetable(T,file,'WriteMode','append','WriteVariableNames',false);
    else
        writetable(T,file);
    end
catch
    % Fallback for older table writers: preserve the newest run even if append fails.
    writetable(T,file);
end
end

function local_worker_isolation(root)
try
    task=getCurrentTask();if isempty(task),wid=0;else,wid=task.ID;end
    r=fullfile(char(root),sprintf('w%d',wid));c=fullfile(r,'c');g=fullfile(r,'g');if ~isfolder(c),mkdir(c);end;if ~isfolder(g),mkdir(g);end
    Simulink.fileGenControl('set','CacheFolder',c,'CodeGenFolder',g,'createDir',true);
catch
end
end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.OutputRoot="";opts.IdentifiedMats=strings(0,1);opts.SpeedNodesMps=[45;50;55];opts.ConfigIds=(0:4).';opts.Profile=[];opts.ReferenceAltitudeM=200;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;
opts.BayesEvaluations=24;opts.UseParallel=true;opts.ShortFileGenRoot="D:\\AXC\\v32_inner";opts.Wpitch=0.08;opts.WmvElev=0.08;opts.WmvThrottle=0.08;opts.ElevatorDeviationLimit=0.08;opts.ThrottleDeviationLimit=0.12;opts.ElevatorDeviationRateLimit=0.012;opts.ThrottleDeviationRateLimit=0.020;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
