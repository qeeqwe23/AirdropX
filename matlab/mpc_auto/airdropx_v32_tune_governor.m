function result=airdropx_v32_tune_governor(varargin)
%AIRDROPX_V32_TUNE_GOVERNOR Learn one downstream-aware height governor for every cfg.
opts=local_options(varargin{:});root=string(opts.OutputRoot);if ~isfolder(root),mkdir(root);end
P=double(opts.Profile);if isempty(P),P=[0 200 50;30 190 50;80 205 50;140 180 50;210 200 50];end
resumeFile=fullfile(root,'governor_resume_checkpoint.mat');searchFile=fullfile(root,'governor_search_records.csv');
records=table();
bestGate=Inf;best=struct();resumeAttempt=0;
if isfile(resumeFile)
    try,S=load(resumeFile);if isfield(S,'resume'),resumeAttempt=double(S.resume.attempt_count);bestGate=double(S.resume.best_gate);best=S.resume.best_params;end,catch,end
end
resumeMode=isfinite(bestGate) && ~isempty(fieldnames(best));
if resumeMode
    fprintf('[V32.1-GOV] resume memory found: previous best gate=%.3f attempt=%d. Running local resume candidates before new BO.\n',bestGate,resumeAttempt);
    C=local_resume_candidates(best);
else
    C=local_seeds();
end
for i=1:height(C)
    p=local_row(C(i,:));if resumeMode,src="resume_local";else,src="seed";end
    g=local_eval(p,opts,P,src+"_"+i);r0=local_record(p,g,src);if isempty(records),records=r0;else,records=[records;r0];end %#ok<AGROW>
    if g<bestGate,bestGate=g;best=p;end
    fprintf('[V32-GOV] seed %d/%d gate=%.3f\n',i,height(C),g);
end
local_append_records(searchFile,records);
resume=struct('attempt_count',resumeAttempt+1,'best_gate',bestGate,'best_params',best,'formal_pass',false,'updated_at',datetime('now')); %#ok<NASGU>
save(resumeFile,'resume','-v7');
if bestGate>1
    vars=[optimizableVariable('Kh',[0.03 0.35],'Transform','log'),optimizableVariable('Ki',[0.0003 0.03],'Transform','log'),...
        optimizableVariable('Kaw',[0.03 2.0],'Transform','log'),optimizableVariable('VzMax',[0.8 2.5]),...
        optimizableVariable('VzSlew',[0.2 1.8]),optimizableVariable('BiasMax',[0.4 3.0])];
    bo=bayesopt(@(X)local_bayes(X,opts,P),vars,'MaxObjectiveEvaluations',opts.BayesEvaluations,'IsObjectiveDeterministic',true,'UseParallel',logical(opts.UseParallel),'Verbose',1,'AcquisitionFunctionName','expected-improvement-plus');
    p=local_row(bestPoint(bo));g=bo.MinObjective;save(fullfile(root,'governor_bayesopt.mat'),'bo','-v7.3');if g<bestGate,bestGate=g;best=p;end
    resume=struct('attempt_count',resumeAttempt+1,'best_gate',bestGate,'best_params',best,'formal_pass',false,'updated_at',datetime('now')); %#ok<NASGU>
    save(resumeFile,'resume','-v7');
end
[formalGate,rows]=local_validate(best,opts,P,'formal');writetable(rows,fullfile(root,'governor_formal_validation.csv'));writetable(local_record(best,formalGate,"formal"),fullfile(root,'best_governor.csv'));
result=struct('pass',formalGate<=1,'gate_ratio',formalGate,'params',best,'validation',rows);
resume=struct('attempt_count',resumeAttempt+1,'best_gate',formalGate,'best_params',best,'formal_pass',formalGate<=1,'updated_at',datetime('now')); %#ok<NASGU>
save(resumeFile,'resume','-v7');
if formalGate>1,error('AirdropX:V32:GovernorBlocked','Height governor did not pass bounded clean learning. Best gate %.3f; v32 resume checkpoint saved.',formalGate);end
fprintf('[V32-GOV] FORMAL PASS gate=%.3f. Height loop certified after inner Va/vz layer.\n',formalGate);
end
function y=local_bayes(X,o,P),p=local_row(X);y=local_eval(p,o,P,"bo");if ~isfinite(y),y=50;end,end
function g=local_eval(p,o,P,label)
local_worker_isolation(o.ShortFileGenRoot);id=string(label)+"_"+string(char(java.util.UUID.randomUUID()));[g,rows]=local_validate(p,o,P,char(id));d=fullfile(o.OutputRoot,'evaluations',char(id));if ~isfolder(d),mkdir(d);end;writetable(rows,fullfile(d,'cfg_summary.csv'));writetable(local_record(p,g,string(label)),fullfile(d,'candidate.csv'));
end
function [g,rows]=local_validate(p,o,P,label)
g=0;rows=table();for cfg=double(o.ConfigIds(:)).'
 out=fullfile(o.OutputRoot,'runtime',sprintf('%s_cfg%d',label,cfg));
 try
  r=airdropx_v32_governor_validation('ProjectRoot',o.ProjectRoot,'BankMat',o.BankMat,'OutputRoot',out,'ConfigId',cfg,'Profile',P,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'HiddenElevatorTrim',o.HiddenElevatorTrim,...
   'HeightKh',p.Kh,'HeightKi',p.Ki,'HeightKaw',p.Kaw,'HeightVzMaxMps',p.VzMax,'HeightVzSlewMps2',p.VzSlew,'HeightBiasMaxMps',p.BiasMax);
  row=r.summary;row.config_id=repmat(cfg,height(row),1);row.infrastructure_fail=false(height(row),1);if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,double(r.gate_ratio)); %#ok<AGROW>
 catch ME
  try
   pause(0.2);r=airdropx_v32_governor_validation('ProjectRoot',o.ProjectRoot,'BankMat',o.BankMat,'OutputRoot',char(string(out)+"_retry"),'ConfigId',cfg,'Profile',P,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'HiddenElevatorTrim',o.HiddenElevatorTrim,...
    'HeightKh',p.Kh,'HeightKi',p.Ki,'HeightKaw',p.Kaw,'HeightVzMaxMps',p.VzMax,'HeightVzSlewMps2',p.VzSlew,'HeightBiasMaxMps',p.BiasMax);
   row=r.summary;row.config_id=repmat(cfg,height(row),1);row.infrastructure_fail=false(height(row),1);if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,double(r.gate_ratio)); %#ok<AGROW>
  catch ME2
   row=table(false,50,Inf,Inf,Inf,Inf,Inf,Inf,-Inf,string(ME2.message),cfg,true,'VariableNames',{'pass','gate_ratio','max_tail_H_rms_m','max_tail_V_rms_mps','max_H_settle_s','max_abs_vz_mps','max_abs_q_dps','max_pitch_dev_deg','min_altitude_m','status','config_id','infrastructure_fail'});if isempty(rows),rows=row;else,rows=[rows;row];end;g=max(g,50); %#ok<AGROW>
  end
 end
end
end

function C=local_resume_candidates(p)
% Cross-start memory: continue near the best v32 governor from the prior run.
F=[1 1 1 1 1 1;0.85 1 1.25 1 0.85 1;1.15 1 1.25 1 1.15 1;1 0.75 1.5 1.10 1 1;1 1.25 1.5 0.90 1 1.15;1 1 0.75 1.15 1.20 0.90];
C=table(max(0.03,min(0.35,p.Kh*F(:,1))),max(0.0003,min(0.03,p.Ki*F(:,2))),max(0.03,min(2.0,p.Kaw*F(:,3))),...
 max(0.8,min(2.5,p.VzMax*F(:,4))),max(0.2,min(1.8,p.VzSlew*F(:,5))),max(0.4,min(3.0,p.BiasMax*F(:,6))),...
 'VariableNames',{'Kh','Ki','Kaw','VzMax','VzSlew','BiasMax'});
end

function C=local_seeds()
C=table([0.08;0.12;0.16;0.20;0.10;0.25;0.14;0.18],[0.002;0.004;0.006;0.008;0.010;0.003;0.012;0.005],[0.20;0.35;0.50;0.70;1.00;0.15;0.80;1.20],[1.2;1.5;1.8;2.0;1.4;2.3;2.5;1.8],[0.45;0.60;0.75;0.90;0.60;1.10;1.20;0.80],[1.0;1.5;1.8;2.0;1.2;2.5;2.2;1.8],...
 'VariableNames',{'Kh','Ki','Kaw','VzMax','VzSlew','BiasMax'});
end
function p=local_row(X),p=struct();for n={'Kh','Ki','Kaw','VzMax','VzSlew','BiasMax'},p.(n{1})=double(X.(n{1})(1));end,end
function T=local_record(p,g,src),T=table(string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')),string(src),p.Kh,p.Ki,p.Kaw,p.VzMax,p.VzSlew,p.BiasMax,g,g<=1,'VariableNames',{'timestamp','source','Kh','Ki','Kaw','VzMax','VzSlew','BiasMax','gate_ratio','pass'});end
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
try,task=getCurrentTask();if isempty(task),wid=0;else,wid=task.ID;end;r=fullfile(char(root),sprintf('w%d',wid));c=fullfile(r,'c');g=fullfile(r,'g');if ~isfolder(c),mkdir(c);end;if ~isfolder(g),mkdir(g);end;Simulink.fileGenControl('set','CacheFolder',c,'CodeGenFolder',g,'createDir',true);catch,end
end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="";opts.OutputRoot="";opts.ConfigIds=(0:4).';opts.Profile=[];opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;opts.BayesEvaluations=18;opts.UseParallel=true;opts.ShortFileGenRoot="D:\\AXC\\v32_gov";
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
