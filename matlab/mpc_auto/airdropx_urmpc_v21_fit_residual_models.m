function result=airdropx_urmpc_v21_fit_residual_models(varargin)
%AIRDROPX_URMPC_V21_FIT_RESIDUAL_MODELS Fit local delta-A/delta-B from calibration data.
%
% Stage-A offline learner. It never overwrites the flight bank. The same
% normalized no-intercept ridge model and ONE globally selected lambda are
% used for all speed x cfg vertices. The nominal equilibrium is preserved.
%
% residual(k) ~= dA*(x-xnom) + dB*(u-unom) + w(k)
%
% Only trusted near-trim samples are accepted. A deploy-ready flag is true
% only if EVERY requested vertex has enough data and its blocked validation
% residual is not worsened by the learned correction.

opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));
bank=local_resolve(root,opts.BankMat);calRoot=local_resolve(root,opts.CalibrationRoot);
manifestPath=fullfile(calRoot,'urmpc_v21_calibration_manifest.csv');if ~isfile(manifestPath),error('AirdropX:URMPC:MissingCalibrationManifest','Missing %s',manifestPath);end
B=load(bank,'ur_models','ur_meta','ur_mpc');M=readtable(manifestPath,'TextType','string');
Sx=diag(local_state_scales(B.ur_meta));Su=diag(local_mv_scales(B.ur_mpc));
lambdas=double(opts.LambdaGrid(:));nV=height(M);D=cell(nV,1);usable=false(nV,1);
for i=1:nV
    try,D{i}=local_vertex_data(M(i,:),B,Sx,Su,opts);usable(i)=D{i}.n>=opts.MinTrustedSamples;catch ME,D{i}=struct('n',0,'error',string(ME.identifier)+" | "+string(ME.message));end
end
cvRows=zeros(numel(lambdas),4);for li=1:numel(lambdas)
    ratios=[];for i=1:nV,if ~usable(i),continue;end;ratios(end+1)=local_cv_ratio(D{i},lambdas(li),opts.TrainFraction);end %#ok<AGROW>
    if isempty(ratios),med=Inf;p90=Inf;worst=Inf;else,med=median(ratios,'omitnan');p90=local_pctl(ratios,90);worst=max(ratios,[],'omitnan');end
    cvRows(li,:)=[lambdas(li),med,p90,worst];
end
% The deployment contract is worst-vertex, not median-vertex: a single
% global correction is deployable only when EVERY usable vertex is not
% worsened. Therefore lambda selection must use the same minimax criterion.
% Selecting by median can choose an aggressive lambda that improves most
% vertices while catastrophically overfitting a few (the v2.1A bug).
allVerticesUsable=all(usable) && nV>0;
feasible=repmat(allVerticesUsable,size(cvRows,1),1) & (cvRows(:,4)<=double(opts.MaxValidationRatio));
if any(feasible)
    cand=find(feasible);
    % Robust choice: maximize the worst-vertex validation margin. Tie-break
    % with p90, then median, then prefer stronger regularization.
    key=[cvRows(cand,4),cvRows(cand,3),cvRows(cand,2),-log10(max(cvRows(cand,1),realmin))];
    [~,ord]=sortrows(key,[1 2 3 4]);best=cand(ord(1));
    selectionPolicy="minimax_feasible_worst_vertex";
else
    % No deploy-feasible lambda exists (or some requested vertex lacks
    % trusted data). Still choose the minimax diagnostic candidate, but the
    % final deploy_ready gate below remains false.
    key=[cvRows(:,4),cvRows(:,3),cvRows(:,2),-log10(max(cvRows(:,1),realmin))];
    [~,ord]=sortrows(key,[1 2 3 4]);best=ord(1);
    selectionPolicy="minimax_diagnostic_no_feasible_lambda";
end
lambda=cvRows(best,1);
CV=array2table([cvRows,double(feasible)],'VariableNames',{'lambda','median_validation_ratio','p90_validation_ratio','worst_validation_ratio','deploy_feasible'});

models=repmat(struct('speed_mps',NaN,'cfg_id',NaN,'deltaA',zeros(5),'deltaB',zeros(5,2),'w_normalized_abs_p95',NaN(1,5),'lambda',lambda,'usable',false),nV,1);
reportRows=cell(nV,1);allReady=true;
for i=1:nV
    speed=double(M.speed_mps(i));cfg=double(M.cfg_id(i));n=0;base=NaN;corr=NaN;ratio=Inf;dAn=NaN;dBn=NaN;wp=NaN(1,5);fitStatus="INSUFFICIENT_DATA";
    if usable(i)
        X=D{i};n=X.n;[Theta,base,corr,ratio,wp]=local_fit_and_validate(X,lambda,opts.TrainFraction);
        Txa=Theta(:,1:5);Txb=Theta(:,6:7);dA=Sx*Txa/Sx;dB=Sx*Txb/Su;dAn=norm(dA,'fro');dBn=norm(dB,'fro');
        models(i)=struct('speed_mps',speed,'cfg_id',cfg,'deltaA',dA,'deltaB',dB,'w_normalized_abs_p95',wp,'lambda',lambda,'usable',ratio<=opts.MaxValidationRatio);
        if ratio<=opts.MaxValidationRatio,fitStatus="PASS";else,fitStatus="VALIDATION_WORSE";allReady=false;end
    else
        allReady=false;
    end
    reportRows{i}=table(speed,cfg,n,base,corr,ratio,dAn,dBn,wp(1),wp(2),wp(3),wp(4),wp(5),fitStatus, ...
        'VariableNames',{'speed_mps','cfg_id','trusted_samples','validation_rms_before','validation_rms_after','validation_ratio','deltaA_fro','deltaB_fro', ...
        'w_norm_h_p95_abs','w_norm_va_p95_abs','w_norm_pitch_p95_abs','w_norm_vz_p95_abs','w_norm_q_p95_abs','status'});
end
Fit=vertcat(reportRows{:});
allReady=allReady && height(Fit)==height(M) && all(Fit.status=="PASS");
outMat=fullfile(calRoot,'urmpc_v21_residual_models_candidate.mat');outFit=fullfile(calRoot,'urmpc_v21_residual_fit_report.csv');outCv=fullfile(calRoot,'urmpc_v21_lambda_cv.csv');
residual_models=models;selected_lambda=lambda;deploy_ready=logical(allReady);selection_policy=selectionPolicy; %#ok<NASGU>
selected_worst_validation_ratio=cvRows(best,4);selected_p90_validation_ratio=cvRows(best,3);selected_median_validation_ratio=cvRows(best,2); %#ok<NASGU>
save(outMat,'residual_models','selected_lambda','deploy_ready','selection_policy','selected_worst_validation_ratio','selected_p90_validation_ratio','selected_median_validation_ratio','-v7.3');writetable(Fit,outFit);writetable(CV,outCv);
result=struct('models',models,'fit_report',Fit,'lambda_cv',CV,'selected_lambda',lambda,'selection_policy',selectionPolicy, ...
    'selected_worst_validation_ratio',selected_worst_validation_ratio,'deploy_ready',allReady,'candidate_mat',string(outMat),'fit_csv',string(outFit),'cv_csv',string(outCv));
fprintf('[UR-MPC v2.1A.1 FIT] lambda=%.6g policy=%s worst=%.6f deploy_ready=%d pass=%d/%d\n',lambda,char(selectionPolicy),selected_worst_validation_ratio,allReady,sum(Fit.status=="PASS"),height(Fit));
end

function X=local_vertex_data(row,B,Sx,Su,o)
out=char(row.output_dir);rp=fullfile(out,'urmpc_one_step_residual.csv');tp=fullfile(out,'urmpc_controller_trace.csv');
if ~isfile(rp)||~isfile(tp),error('AirdropX:URMPC:MissingCalibrationCSV','Missing residual/trace under %s',out);end
R=readtable(rp);T=readtable(tp);[tf,loc]=ismembertol(double(R.prediction_from_time_s),double(T.time_s),1e-7,'DataScale',1);idx=find(tf);loc=loc(tf);R=R(idx,:);T=T(loc,:);
cfg=double(row.cfg_id);m=double(R.valid_prediction)>0.5 & round(double(R.cfg_from))==cfg & double(R.prediction_from_time_s)>=double(row.fit_start_s) & double(R.prediction_from_time_s)<=double(row.fit_end_s);
R=R(m,:);T=T(m,:);if isempty(R),X=struct('n',0);return;end
DX=[double(T.actual_h_m)-double(T.requested_h_m),double(T.actual_v_mps)-double(T.nominal_va_mps),double(T.pitch_deg)-double(T.nominal_pitch_deg),double(T.actual_vz_mps),double(T.q_est_dps)].';
DU=[double(T.physical_elevator_cmd)-double(T.nominal_physical_elevator),double(T.physical_throttle_cmd)-double(T.nominal_throttle)].';
Y=[double(R.res_h_m),double(R.res_va_mps),double(R.res_pitch_deg),double(R.res_vz_mps),double(R.res_q_dps)].';
Z=[Sx\DX;Su\DU];Yn=Sx\Y;stateNorm=sqrt(sum((Sx\DX).^2,1));good=all(isfinite([Z;Yn]),1)&isfinite(stateNorm)&stateNorm<=o.TrustStateNormMax;
Z=Z(:,good);Yn=Yn(:,good);Yraw=Y(:,good);X=struct('Z',Z,'Yn',Yn,'Yraw',Yraw,'n',size(Z,2));
end
function ratio=local_cv_ratio(X,lambda,f)
n=X.n;ntr=max(10,min(n-10,floor(double(f)*n)));Theta=local_ridge(X.Z(:,1:ntr),X.Yn(:,1:ntr),lambda);Y=X.Yn(:,ntr+1:end);E=Y-Theta*X.Z(:,ntr+1:end);b=sqrt(mean(sum(Y.^2,1),'omitnan'));a=sqrt(mean(sum(E.^2,1),'omitnan'));ratio=a/max(b,eps);
end
function [Theta,before,after,ratio,wp]=local_fit_and_validate(X,lambda,f)
n=X.n;ntr=max(10,min(n-10,floor(double(f)*n)));Theta=local_ridge(X.Z(:,1:ntr),X.Yn(:,1:ntr),lambda);Yv=X.Yn(:,ntr+1:end);Zv=X.Z(:,ntr+1:end);Ev=Yv-Theta*Zv;before=sqrt(mean(sum(Yv.^2,1),'omitnan'));after=sqrt(mean(sum(Ev.^2,1),'omitnan'));ratio=after/max(before,eps);
% Fit final correction on all trusted data, then estimate the remaining W on
% all trusted samples. This W is diagnostic only at Stage-A.
Theta=local_ridge(X.Z,X.Yn,lambda);En=X.Yn-Theta*X.Z;wp=zeros(1,5);for j=1:5,wp(j)=local_pctl(abs(En(j,:)),95);end
end
function T=local_ridge(Z,Y,lambda),T=(Y*Z')/(Z*Z'+double(lambda)*eye(size(Z,1)));end
function sc=local_state_scales(meta),sc=[3;1.5;6;.5;.5];try,x=double(meta.output_scales(:));if numel(x)==5&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function sc=local_mv_scales(C),sc=[.1;.1];try,x=arrayfun(@(m)double(m.ScaleFactor),C.MV(:));x=x(:);if numel(x)==2&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function q=local_pctl(x,p),x=sort(double(x(isfinite(x))));if isempty(x),q=NaN;return;end;if numel(x)==1,q=x;return;end;z=1+(numel(x)-1)*p/100;i=floor(z);f=z-i;if i>=numel(x),q=x(end);else,q=x(i)*(1-f)+x(i+1)*f;end,end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";o.CalibrationRoot="matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration";
o.TrustStateNormMax=1.0;o.MinTrustedSamples=120;o.TrainFraction=0.65;o.LambdaGrid=10.^(-6:1:2);o.MaxValidationRatio=1.0;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
