function result=airdropx_urmpc_v21_corrected_certify(varargin)
%AIRDROPX_URMPC_V21_CORRECTED_CERTIFY Build/certify a corrected LPV candidate bank.
%
% Stage v2.1B is OFFLINE ONLY. It never overwrites the current flight bank.
% The v2.1A.1 residual correction is first projected onto known longitudinal
% physics structure, then revalidated on the original blocked calibration
% data. Only after that are corrected A/B models certified through the same
% 55-point adaptive-MPC preflight and 12 cfg-transition tests as UR-MPC v2.0.
%
% Known structure preserved exactly:
%   h(k+1) = h(k) + Ts*vz(k)
%   altitude error is not allowed to become a learned predictor of the
%   [Va pitch vz q] dynamics merely through calibration correlation.
%
% The corrected residual p95 envelope W(rho,cfg) is audit output only in
% v2.1B. No tube tightening is applied yet.

opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));
sourceBank=local_resolve(root,opts.SourceBankMat);corrMat=local_resolve(root,opts.ResidualCorrectionMat);calRoot=local_resolve(root,opts.CalibrationRoot);
outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
if ~isfile(sourceBank),error('AirdropX:URMPC:MissingSourceBank','Missing source bank: %s',sourceBank);end
if ~isfile(corrMat),error('AirdropX:URMPC:MissingResidualCorrection','Missing residual correction candidate: %s',corrMat);end
manifestPath=fullfile(calRoot,'urmpc_v21_calibration_manifest.csv');
if ~isfile(manifestPath),error('AirdropX:URMPC:MissingCalibrationManifest','Missing calibration manifest: %s',manifestPath);end

B=load(sourceBank);required={'ur_mpc','ur_models','ur_meta'};for i=1:numel(required),if ~isfield(B,required{i}),error('AirdropX:URMPC:BadSourceBank','Source bank missing %s.',required{i});end,end
C=load(corrMat);if ~isfield(C,'residual_models')||~isfield(C,'deploy_ready'),error('AirdropX:URMPC:BadResidualCorrection','Correction MAT missing residual_models/deploy_ready.');end
if logical(opts.RequireStageADeployReady)&&~logical(C.deploy_ready),error('AirdropX:URMPC:StageANotDeployReady','v2.1A candidate is not deploy_ready.');end

ur_mpc=B.ur_mpc;baseModels=B.ur_models;baseMeta=B.ur_meta;models=C.residual_models(:);
Ts=double(baseMeta.Ts);nCfg=size(baseModels,2);speedNodes=double([baseModels(:,1).speed_mps]).';
if numel(speedNodes)~=3||any(abs(sort(speedNodes(:))-[45;50;55])>1e-6),error('AirdropX:URMPC:UnexpectedSourceGrid','Expected 45/50/55 source model nodes.');end
if numel(models)~=numel(speedNodes)*nCfg,error('AirdropX:URMPC:CorrectionCountMismatch','Expected %d correction vertices, got %d.',numel(speedNodes)*nCfg,numel(models));end

% Project learned matrices onto hard physics structure and revalidate using
% exactly the original blocked time split. This makes the Stage-A guarantee
% applicable to what Stage-B will actually deploy.
M=readtable(manifestPath,'TextType','string');Sx=diag(local_state_scales(baseMeta));Su=diag(local_mv_scales(ur_mpc));
projReport=cell(height(M),1);corrMap=repmat(struct('speed_mps',NaN,'cfg_id',NaN,'deltaA',zeros(5),'deltaB',zeros(5,2), ...
    'validation_ratio',Inf,'w_norm_abs_p95',NaN(1,5),'w_phys_abs_p95',NaN(1,5)),height(M),1);
allProjectedReady=true;
for i=1:height(M)
    sp=double(M.speed_mps(i));cfg=double(M.cfg_id(i));cm=local_find_correction(models,sp,cfg);
    dA=double(cm.deltaA);dB=double(cm.deltaB);
    if logical(opts.PreserveAltitudeStructure)
        dA(1,:)=0;dA(:,1)=0;dB(1,:)=0;
    end
    X=local_vertex_data(M(i,:),baseModels,Sx,Su,opts);
    [before,after,ratio,wpNorm,wpPhys]=local_revalidate(X,dA,dB,Sx,opts.TrainFraction);
    status="PASS";if X.n<double(opts.MinTrustedSamples)||~isfinite(ratio)||ratio>double(opts.MaxValidationRatio),status="VALIDATION_WORSE";allProjectedReady=false;end
    corrMap(i)=struct('speed_mps',sp,'cfg_id',cfg,'deltaA',dA,'deltaB',dB,'validation_ratio',ratio,'w_norm_abs_p95',wpNorm,'w_phys_abs_p95',wpPhys);
    projReport{i}=table(sp,cfg,X.n,before,after,ratio,norm(dA,'fro'),norm(dB,'fro'), ...
        wpNorm(1),wpNorm(2),wpNorm(3),wpNorm(4),wpNorm(5),wpPhys(1),wpPhys(2),wpPhys(3),wpPhys(4),wpPhys(5),status, ...
        'VariableNames',{'speed_mps','cfg_id','trusted_samples','validation_rms_before','validation_rms_after','validation_ratio','deltaA_projected_fro','deltaB_projected_fro', ...
        'w_norm_h_p95_abs','w_norm_va_p95_abs','w_norm_pitch_p95_abs','w_norm_vz_p95_abs','w_norm_q_p95_abs', ...
        'w_h_p95_abs_m','w_va_p95_abs_mps','w_pitch_p95_abs_deg','w_vz_p95_abs_mps','w_q_p95_abs_dps','status'});
end
Projected=vertcat(projReport{:});writetable(Projected,fullfile(outRoot,'urmpc_v21_projected_validation.csv'));

% Apply correction only at the stored 45/50/55 x cfg vertices. Runtime and
% dense certification obtain intermediate speeds by the same interpolation
% law used by the existing adaptive MPC.
correctedModels=baseModels;vertexRows={};kk=0;
for ni=1:size(baseModels,1)
    for c=1:nCfg
        kk=kk+1;base=baseModels(ni,c);cm=local_find_correction(corrMap,double(base.speed_mps),double(base.cfg_id));
        Ac=double(base.A)+double(cm.deltaA);Bc=double(base.B)+double(cm.deltaB);
        % Assert exact known altitude kinematics after correction.
        targetRow=zeros(1,5);targetRow(1)=1;targetRow(4)=Ts;
        if logical(opts.PreserveAltitudeStructure) && (max(abs(Ac(1,:)-targetRow))>1e-12 || max(abs(Bc(1,:)))>1e-12 || max(abs(Ac(2:5,1)))>1e-12)
            error('AirdropX:URMPC:AltitudeStructureViolation','Projected correction violated altitude structure at V=%.1f cfg%d.',base.speed_mps,base.cfg_id);
        end
        P=ss(Ac,[Bc Bc],eye(5),zeros(5,4),Ts);P=setmpcsignals(P,'MV',[1 2],'UD',[3 4]);
        try,P.StateName=base.plant.StateName;P.OutputName=base.plant.OutputName;P.InputName=base.plant.InputName;catch,end
        correctedModels(ni,c).A=Ac;correctedModels(ni,c).B=Bc;correctedModels(ni,c).plant=P;
        rho0=max(abs(eig(double(base.A(2:5,2:5)))));rho1=max(abs(eig(Ac(2:5,2:5))));
        vertexRows{kk}=table(double(base.speed_mps),double(base.cfg_id),rho0,rho1,rho1-rho0,norm(cm.deltaA,'fro'),norm(cm.deltaB,'fro'),rank(ctrb(Ac,Bc)), ...
            'VariableNames',{'speed_mps','cfg_id','base_spectral_radius','corrected_spectral_radius','spectral_radius_change','deltaA_fro','deltaB_fro','controllability_rank'});
    end
end
VertexModelReport=vertcat(vertexRows{:});writetable(VertexModelReport,fullfile(outRoot,'urmpc_v21_corrected_vertex_models.csv'));

% Dense 45:1:55 model audit and conservative W envelope. W uses the maximum
% p95 of the two bracketing calibration vertices per cfg; exact node speeds
% use that node's local p95. No inflation is silently invented here.
vgrid=(min(speedNodes):double(opts.PreflightSpeedStepMps):max(speedNodes)).';if abs(vgrid(end)-max(speedNodes))>1e-9,vgrid(end+1)=max(speedNodes);end
gridRows={};gg=0;wRows={};ww=0;
for ii=1:numel(vgrid)
    for c=1:nCfg
        gg=gg+1;mb=local_interp_vertex(baseModels,vgrid(ii),c,Ts);mc=local_interp_vertex(correctedModels,vgrid(ii),c,Ts);
        gridRows{gg}=table(vgrid(ii),c-1,max(abs(eig(mb.A(2:5,2:5)))),max(abs(eig(mc.A(2:5,2:5)))), ...
            norm(mc.A-mb.A,'fro'),norm(mc.B-mb.B,'fro'),rank(ctrb(mc.A,mc.B)), ...
            'VariableNames',{'speed_mps','cfg_id','base_spectral_radius','corrected_spectral_radius','deltaA_interp_fro','deltaB_interp_fro','controllability_rank'});
        [wn,wp]=local_conservative_w(corrMap,vgrid(ii),c-1);
        ww=ww+1;wRows{ww}=table(vgrid(ii),c-1,wn(1),wn(2),wn(3),wn(4),wn(5),wp(1),wp(2),wp(3),wp(4),wp(5), ...
            'VariableNames',{'speed_mps','cfg_id','w_norm_h_p95_abs','w_norm_va_p95_abs','w_norm_pitch_p95_abs','w_norm_vz_p95_abs','w_norm_q_p95_abs', ...
            'w_h_p95_abs_m','w_va_p95_abs_mps','w_pitch_p95_abs_deg','w_vz_p95_abs_mps','w_q_p95_abs_dps'});
    end
end
GridModelReport=vertcat(gridRows{:});WEnvelope=vertcat(wRows{:});writetable(GridModelReport,fullfile(outRoot,'urmpc_v21_corrected_grid_report.csv'));writetable(WEnvelope,fullfile(outRoot,'urmpc_v21_corrected_w_envelope.csv'));

% Reuse the exact v2.0 controller object; only adaptive Plant matrices are
% changed. Constraints, estimator policy, costs and horizons remain fixed.
o=local_cert_options(baseMeta,ur_mpc,opts);
preflight=local_vertex_preflight(ur_mpc,correctedModels,o);writetable(preflight,fullfile(outRoot,'urmpc_v21_corrected_vertex_preflight.csv'));
transition=local_transition_cert(ur_mpc,correctedModels,o);writetable(transition,fullfile(outRoot,'urmpc_v21_corrected_linear_drop_cert.csv'));

projectedPass=all(Projected.status=="PASS");prePass=all(preflight.pass);transPass=all(transition.pass);
deployReady=projectedPass&&prePass&&transPass&&all(VertexModelReport.controllability_rank==5)&&all(GridModelReport.controllability_rank==5);

% Save a separate candidate bank using the normal runtime variable names.
% The existing v2.0 flight bank path is never modified by this function.
ur_models=correctedModels;ur_meta=baseMeta;ur_meta.version='urmpc_v2_1b_corrected_candidate';
ur_meta.architecture=char(string(baseMeta.architecture)+"; vertex-conditioned residual deltaA/deltaB correction; known altitude structure projected; W audit only (no tube tightening yet)");
ur_meta.v21_source_bank=string(sourceBank);ur_meta.v21_correction_mat=string(corrMat);ur_meta.v21_calibration_root=string(calRoot);
ur_meta.v21_selected_lambda=local_field_default(C,'selected_lambda',NaN);ur_meta.v21_selection_policy=char(string(local_field_default(C,'selection_policy',"unknown")));
ur_meta.v21_projected_validation_pass=projectedPass;ur_meta.vertex_preflight_pass=prePass;ur_meta.linear_drop_cert_pass=transPass;ur_meta.v21_deploy_ready=deployReady;ur_meta.created_at=datetime('now');
outBank=local_resolve(root,opts.OutputBankMat);d=fileparts(outBank);if ~isfolder(d),mkdir(d);end
if isfield(B,'v32_nodes'),v32_nodes=B.v32_nodes;else,v32_nodes=[];end %#ok<NASGU>
if isfield(B,'speed_nodes'),speed_nodes=B.speed_nodes;else,speed_nodes=speedNodes;end %#ok<NASGU>
save(outBank,'ur_mpc','ur_models','ur_meta','v32_nodes','speed_nodes','-v7.3');

Summary=table(projectedPass,sum(Projected.status=="PASS"),height(Projected),prePass,sum(preflight.pass),height(preflight),transPass,sum(transition.pass),height(transition),deployReady, ...
    'VariableNames',{'projected_validation_pass','projected_validation_pass_count','projected_validation_total','vertex_preflight_pass','vertex_preflight_pass_count','vertex_preflight_total', ...
    'linear_transition_pass','linear_transition_pass_count','linear_transition_total','deploy_ready'});
writetable(Summary,fullfile(outRoot,'urmpc_v21b_summary.csv'));

fprintf('\n[UR-MPC v2.1B CORRECTED CERT]\n');
fprintf('  projected calibration validation: %d/%d PASS\n',sum(Projected.status=="PASS"),height(Projected));
fprintf('  corrected vertex preflight:       %d/%d PASS\n',sum(preflight.pass),height(preflight));
fprintf('  corrected cfg transitions:        %d/%d PASS\n',sum(transition.pass),height(transition));
fprintf('  deploy_ready=%d\n',deployReady);fprintf('  candidate bank: %s\n',outBank);
result=struct('projected_validation',Projected,'vertex_model_report',VertexModelReport,'grid_model_report',GridModelReport,'w_envelope',WEnvelope, ...
    'vertex_preflight',preflight,'linear_drop_cert',transition,'deploy_ready',deployReady,'candidate_bank',string(outBank),'summary',Summary);
end

function X=local_vertex_data(row,baseModels,Sx,Su,o)
out=char(row.output_dir);rp=fullfile(out,'urmpc_one_step_residual.csv');tp=fullfile(out,'urmpc_controller_trace.csv');
if ~isfile(rp)||~isfile(tp),error('AirdropX:URMPC:MissingCalibrationCSV','Missing residual/trace under %s',out);end
R=readtable(rp);T=readtable(tp);[tf,loc]=ismembertol(double(R.prediction_from_time_s),double(T.time_s),1e-7,'DataScale',1);R=R(tf,:);T=T(loc(tf),:);
cfg=double(row.cfg_id);m=double(R.valid_prediction)>0.5 & round(double(R.cfg_from))==cfg & double(R.prediction_from_time_s)>=double(row.fit_start_s) & double(R.prediction_from_time_s)<=double(row.fit_end_s);
R=R(m,:);T=T(m,:);if isempty(R),X=struct('n',0);return;end
DX=[double(T.actual_h_m)-double(T.requested_h_m),double(T.actual_v_mps)-double(T.nominal_va_mps),double(T.pitch_deg)-double(T.nominal_pitch_deg),double(T.actual_vz_mps),double(T.q_est_dps)].';
DU=[double(T.physical_elevator_cmd)-double(T.nominal_physical_elevator),double(T.physical_throttle_cmd)-double(T.nominal_throttle)].';
Y=[double(R.res_h_m),double(R.res_va_mps),double(R.res_pitch_deg),double(R.res_vz_mps),double(R.res_q_dps)].';
stateNorm=sqrt(sum((Sx\DX).^2,1));good=all(isfinite([DX;DU;Y]),1)&isfinite(stateNorm)&stateNorm<=double(o.TrustStateNormMax);
X=struct('DX',DX(:,good),'DU',DU(:,good),'Y',Y(:,good),'Yn',Sx\Y(:,good),'n',sum(good));
end
function [before,after,ratio,wpNorm,wpPhys]=local_revalidate(X,dA,dB,Sx,f)
if X.n<20,before=NaN;after=NaN;ratio=Inf;wpNorm=NaN(1,5);wpPhys=NaN(1,5);return;end
n=X.n;ntr=max(10,min(n-10,floor(double(f)*n)));idx=ntr+1:n;
Y=X.Y(:,idx);Yn=Sx\Y;pred=dA*X.DX(:,idx)+dB*X.DU(:,idx);En=Sx\(Y-pred);
before=sqrt(mean(sum(Yn.^2,1),'omitnan'));after=sqrt(mean(sum(En.^2,1),'omitnan'));ratio=after/max(before,eps);
Eall=X.Y-dA*X.DX-dB*X.DU;EnAll=Sx\Eall;wpNorm=zeros(1,5);wpPhys=zeros(1,5);for j=1:5,wpNorm(j)=local_pctl(abs(EnAll(j,:)),95);wpPhys(j)=local_pctl(abs(Eall(j,:)),95);end
end
function cm=local_find_correction(models,sp,cfg)
idx=find(arrayfun(@(m)abs(double(m.speed_mps)-double(sp))<1e-6 && double(m.cfg_id)==double(cfg),models),1);
if isempty(idx),error('AirdropX:URMPC:MissingResidualVertex','No residual correction at V=%.1f cfg%d.',sp,cfg);end;cm=models(idx);
end
function [wn,wp]=local_conservative_w(map,v,cfg)
idx=find(arrayfun(@(m)double(m.cfg_id)==double(cfg),map));if isempty(idx),error('Missing W cfg%d',cfg);end
sp=arrayfun(@(m)double(m.speed_mps),map(idx));[sp,ord]=sort(sp);idx=idx(ord);
if v<=sp(1)+1e-9,use=idx(1);elseif v>=sp(end)-1e-9,use=idx(end);else,i1=find(sp>=v,1,'first');i0=i1-1;if abs(v-sp(i0))<1e-9,use=idx(i0);elseif abs(v-sp(i1))<1e-9,use=idx(i1);else,use=[idx(i0) idx(i1)];end,end
if numel(use)==1,wn=map(use).w_norm_abs_p95;wp=map(use).w_phys_abs_p95;else,wn=max(vertcat(map(use).w_norm_abs_p95),[],1);wp=max(vertcat(map(use).w_phys_abs_p95),[],1);end
end
function m=local_interp_vertex(M,v,cfg,Ts)
speeds=double([M(:,1).speed_mps]).';[speeds,ord]=sort(speeds);M=M(ord,:);
if v<=speeds(1),i0=1;i1=1;w=0;elseif v>=speeds(end),i0=numel(speeds);i1=i0;w=0;else,i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));end
m0=M(i0,cfg);m1=M(i1,cfg);if i0==i1,A=m0.A;B=m0.B;x=m0.x_nominal;u=m0.u_nominal;h=m0.hidden_elevator_offset;else,A=(1-w)*m0.A+w*m1.A;B=(1-w)*m0.B+w*m1.B;x=(1-w)*m0.x_nominal+w*m1.x_nominal;u=(1-w)*m0.u_nominal+w*m1.u_nominal;h=(1-w)*m0.hidden_elevator_offset+w*m1.hidden_elevator_offset;end
P=ss(A,[B B],eye(5),zeros(5,4),Ts);P=setmpcsignals(P,'MV',[1 2],'UD',[3 4]);m=struct('speed_mps',double(v),'cfg_id',cfg-1,'A',A,'B',B,'plant',P,'x_nominal',x,'u_nominal',u,'hidden_elevator_offset',h);
end
function T=local_vertex_preflight(C,M,o)
sn=double([M(:,1).speed_mps]).';vgrid=(min(sn):double(o.PreflightSpeedStepMps):max(sn)).';if abs(vgrid(end)-max(sn))>1e-9,vgrid(end+1)=max(sn);end
rows={};k=0;for ii=1:numel(vgrid),for c=1:size(M,2),k=k+1;m=local_interp_vertex(M,vgrid(ii),c,double(o.Ts));Nom=local_nominal(m,double(o.ReferenceAltitudeM));st=local_state_at_nominal(C,Nom,m.u_nominal);y=Nom.Y(:).';r=y;opt=local_options_for_hidden(m.hidden_elevator_offset,o);ok=false;its=NaN;qp="";de=NaN;dt=NaN;signOK=false;ctr=rank(ctrb(m.A,m.B));
    try,[u,info]=mpcmoveAdaptive(C,st,m.plant,Nom,y,r,[],opt);its=double(info.Iterations);qp=string(info.QPCode);de=u(1)-m.u_nominal(1);dt=u(2)-m.u_nominal(2);nominalOK=its>0&&all(isfinite(u))&&abs(de)<0.03&&abs(dt)<0.03;st2=local_state_at_nominal(C,Nom,m.u_nominal);y2=y;y2(1)=y2(1)-1;y2(4)=-0.2;[u2,info2]=mpcmoveAdaptive(C,st2,m.plant,Nom,y2,r,[],opt);signOK=double(info2.Iterations)>0&&isfinite(u2(1))&&(u2(1)<=m.u_nominal(1)+0.01);ok=nominalOK&&signOK&&(ctr==5);catch ME,qp="EX:"+string(ME.identifier);end
    rows(k,:)={m.speed_mps,m.cfg_id,ok,its,qp,de,dt,signOK,ctr};end,end
T=cell2table(rows,'VariableNames',{'speed_mps','cfg_id','pass','iterations','qp_code','nominal_dElev','nominal_dThrottle','nose_up_sign_pass','controllability_rank'});
end
function T=local_transition_cert(C,M,o)
rows={};k=0;Ts=double(o.Ts);N=max(1,ceil(double(o.LinearTransitionSimS)/Ts));
for i=1:size(M,1),for c=1:size(M,2)-1,k=k+1;old=M(i,c);nw=M(i,c+1);x=old.x_nominal(:);x(1)=double(o.ReferenceAltitudeM);u=old.u_nominal(:);Nom=local_nominal(nw,double(o.ReferenceAltitudeM));st=mpcstate(C);st.LastMove=u;try,if numel(st.Plant)==5,st.Plant=x(:);end,catch,end;r=[double(o.ReferenceAltitudeM),nw.x_nominal(2),nw.x_nominal(3),0,0];opt=local_options_for_hidden(nw.hidden_elevator_offset,o);finite=true;maxH=0;maxV=0;maxVz=0;maxQ=0;qpFails=0;maxStepE=0;maxStepT=0;rateExceedCount=0;maxSlack=0;lastU=u(:);
    for n=1:N,y=x(:).';try,[uCmd,info]=mpcmoveAdaptive(C,st,nw.plant,Nom,y,r,[],opt);if double(info.Iterations)<=0,qpFails=qpFails+1;end;if isfield(info,'Slack')&&isfinite(double(info.Slack)),maxSlack=max(maxSlack,double(info.Slack));end,catch,finite=false;break;end;if any(~isfinite(uCmd)),finite=false;break;end;u=double(uCmd(:));step=abs(u-lastU);maxStepE=max(maxStepE,step(1));maxStepT=max(maxStepT,step(2));if numel(o.DerivedRateLimits)>=2&&any(step>double(o.DerivedRateLimits(:))+1e-9),rateExceedCount=rateExceedCount+1;end;lastU=u;dx=x-Nom.X(:);du=u-nw.u_nominal(:);x=Nom.X(:)+nw.A*dx+nw.B*du;maxH=max(maxH,abs(x(1)-o.ReferenceAltitudeM));maxV=max(maxV,abs(x(2)-nw.x_nominal(2)));maxVz=max(maxVz,abs(x(4)));maxQ=max(maxQ,abs(x(5)));if any(~isfinite(x)),finite=false;break;end,end
    final=[abs(x(1)-o.ReferenceAltitudeM),abs(x(2)-nw.x_nominal(2)),abs(x(4)),abs(x(5))];pass=finite&&qpFails==0&&final(1)<=1.0&&final(2)<=0.5&&final(3)<=0.20&&final(4)<=0.20;rows(k,:)={nw.speed_mps,c-1,c,pass,qpFails,maxH,maxV,maxVz,maxQ,final(1),final(2),final(3),final(4),maxStepE,maxStepT,rateExceedCount,maxSlack};end,end
T=cell2table(rows,'VariableNames',{'speed_mps','cfg_from','cfg_to','pass','qp_fail_count','max_h_error_m','max_va_error_mps','max_vz_mps','max_q_dps','final_h_error_m','final_va_error_mps','final_vz_mps','final_q_dps','max_elevator_step','max_throttle_step','rate_soft_exceed_count','max_slack'});
end
function Nom=local_nominal(m,h),Nom=struct('X',[double(h);m.x_nominal(2:5)],'U',[m.u_nominal(:);0;0],'Y',[double(h);m.x_nominal(2:5)],'DX',zeros(5,1));end
function st=local_state_at_nominal(C,Nom,uNom),st=mpcstate(C);try,if numel(st.Plant)==numel(Nom.X),st.Plant=Nom.X(:);end,catch,end;st.LastMove=double(uNom(:));end
function opt=local_options_for_hidden(hidden,o),[eMin,eMax]=local_elevator_physical_bounds(hidden,o);opt=mpcmoveopt;opt.MVMin=[eMin,double(o.ThrottleMin)];opt.MVMax=[eMax,double(o.ThrottleMax)];end
function [lo,hi]=local_elevator_physical_bounds(hidden,o),d=abs(double(o.ExternalElevatorDeltaLimit));lo=max(double(o.PhysicalElevatorMin),double(hidden)-d);hi=min(double(o.PhysicalElevatorMax),double(hidden)+d);if ~(isfinite(lo)&&isfinite(hi)&&lo<hi),error('Bad elevator envelope.');end,end
function o=local_cert_options(meta,C,user)
o=user;o.Ts=double(meta.Ts);o.ReferenceAltitudeM=double(C.Model.Nominal.X(1));o.ExternalElevatorDeltaLimit=double(meta.elevator_external_delta_limit);o.PhysicalElevatorMin=double(meta.physical_elevator_limit(1));o.PhysicalElevatorMax=double(meta.physical_elevator_limit(2));o.ThrottleMin=double(meta.throttle_limit(1));o.ThrottleMax=double(meta.throttle_limit(2));o.DerivedRateLimits=double(meta.model_validity_rate_limits_per_sample(:));
end
function sc=local_state_scales(meta),sc=[3;1.5;6;.5;.5];try,x=double(meta.output_scales(:));if numel(x)==5&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function sc=local_mv_scales(C),sc=[.1;.1];try,x=arrayfun(@(m)double(m.ScaleFactor),C.MV(:));x=x(:);if numel(x)==2&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function q=local_pctl(x,p),x=sort(double(x(isfinite(x))));if isempty(x),q=NaN;return;end;if numel(x)==1,q=x;return;end;z=1+(numel(x)-1)*p/100;i=floor(z);f=z-i;if i>=numel(x),q=x(end);else,q=x(i)*(1-f)+x(i+1)*f;end,end
function v=local_field_default(S,name,d),if isfield(S,name),v=S.(name);else,v=d;end,end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.SourceBankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";
o.ResidualCorrectionMat="matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration/urmpc_v21_residual_models_candidate.mat";
o.CalibrationRoot="matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration";
o.OutputRoot="matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate";
o.OutputBankMat="matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate/airdropx_urmpc_v21_corrected_candidate.mat";
o.RequireStageADeployReady=true;o.PreserveAltitudeStructure=true;o.TrustStateNormMax=1.0;o.MinTrustedSamples=120;o.TrainFraction=0.65;o.MaxValidationRatio=1.0;
o.PreflightSpeedStepMps=1.0;o.LinearTransitionSimS=30;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=char(string(varargin{i}));if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
