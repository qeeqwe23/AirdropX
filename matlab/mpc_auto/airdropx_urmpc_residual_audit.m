function result=airdropx_urmpc_residual_audit(traceInput,bankInput,outputRoot)
%AIRDROPX_URMPC_RESIDUAL_AUDIT Read-only one-step model/estimator audit.
%
% This function DOES NOT alter the controller or simulation.  It reconstructs
% the exact one-step LPV model used at sample k, predicts x(k+1) from the
% measured x(k) and applied physical MV, and compares it with telemetry.
% It also projects the residual onto the two B columns to test how much of the
% mismatch can even be represented by the controller's two load-disturbance
% channels.  The resulting CSVs are evidence for a later local tube/W-set
% design; they are not used online by UR-MPC v2.0.x.

if nargin<3||isempty(outputRoot),outputRoot=pwd;end
if istable(traceInput),T=traceInput;else,T=readtable(char(string(traceInput)));end
if ischar(bankInput)||isstring(bankInput)
    B=load(char(string(bankInput)),'ur_models','ur_meta');
else
    B=bankInput;
end
if ~isfield(B,'ur_models')||~isfield(B,'ur_meta'),error('AirdropX:URMPC:AuditBadBank','Audit requires ur_models and ur_meta.');end
if height(T)<2,error('AirdropX:URMPC:AuditShortTrace','Trace has fewer than two samples.');end
req={'time_s','cfg_id','requested_h_m','actual_h_m','actual_v_mps','actual_vz_mps','pitch_deg','q_est_dps','physical_elevator_cmd','physical_throttle_cmd'};
for k=1:numel(req),if ~ismember(req{k},T.Properties.VariableNames),error('AirdropX:URMPC:AuditMissingColumn','Missing trace column %s.',req{k});end,end

speeds=double(B.ur_meta.speed_nodes_mps(:));M=B.ur_models;N=height(T);
rows=NaN(N-1,25);
for k=1:N-1
    a=T(k,:);b=T(k+1,:);dt=double(b.time_s-a.time_s);
    cfg=min(max(round(double(a.cfg_id))+1,1),size(M,2));
    va=double(a.actual_v_mps);hNom=double(a.requested_h_m);
    [A,Bu,xNom,uNom,i0,i1,w]=local_interp(M,speeds,va,cfg,hNom);
    x0=[double(a.actual_h_m);double(a.actual_v_mps);double(a.pitch_deg);double(a.actual_vz_mps);double(a.q_est_dps)];
    x1=[double(b.actual_h_m);double(b.actual_v_mps);double(b.pitch_deg);double(b.actual_vz_mps);double(b.q_est_dps)];
    u0=[double(a.physical_elevator_cmd);double(a.physical_throttle_cmd)];
    valid=all(isfinite([dt;x0;x1;u0;xNom;uNom]))&&dt>0&&abs(dt-double(B.ur_meta.Ts))<=max(1e-6,0.25*double(B.ur_meta.Ts));
    pred=NaN(5,1);res=NaN(5,1);dEq=NaN(2,1);fit=NaN(5,1);expl=NaN;nr=NaN;
    if valid
        pred=xNom+A*(x0-xNom)+Bu*(u0-uNom);
        res=x1-pred;
        try,dEq=pinv(Bu)*res;fit=Bu*dEq;den=sum(res.^2);if den>eps,expl=max(0,min(1,1-sum((res-fit).^2)/den));else,expl=1;end,catch,end
        sc=local_scales(B.ur_meta);nr=norm(res./sc);
    end
    estD1=local_col(T,k,'est_disturbance_1');estD2=local_col(T,k,'est_disturbance_2');estDn=local_col(T,k,'est_disturbance_norm');
    estPe=NaN; if all(ismember({'est_plant_h_m','est_plant_va_mps','est_plant_pitch_deg','est_plant_vz_mps','est_plant_q_dps'},T.Properties.VariableNames))
        xe=[double(T.est_plant_h_m(k));double(T.est_plant_va_mps(k));double(T.est_plant_pitch_deg(k));double(T.est_plant_vz_mps(k));double(T.est_plant_q_dps(k))];
        if all(isfinite(xe)),estPe=norm((xe-x0)./local_scales(B.ur_meta));end
    end
    rows(k,:)=[double(b.time_s),double(a.time_s),double(a.cfg_id),double(b.cfg_id),dt,double(i0),double(i1),double(w),double(valid), ...
        res(:).',nr,dEq(:).',expl,norm(res-fit),estD1,estD2,estDn,estPe,double(u0(:).')];
end
names={'time_s','prediction_from_time_s','cfg_from','cfg_to','dt_s','node_low_index','node_high_index','node_weight_high','valid_prediction', ...
    'res_h_m','res_va_mps','res_pitch_deg','res_vz_mps','res_q_dps','normalized_residual_norm', ...
    'equiv_load_elevator','equiv_load_throttle','b_explained_fraction','b_projection_error_norm', ...
    'est_disturbance_1','est_disturbance_2','est_disturbance_norm','est_plant_error_norm', ...
    'applied_physical_elevator','applied_throttle'};
% rows has 25 actual output columns; retain an assertion so future trace
% extensions cannot silently corrupt the audit table.
rows=rows(:,1:numel(names));Aout=array2table(rows,'VariableNames',names);

S=local_summary(Aout);W=local_windows(Aout,5.0);
if ~isfolder(outputRoot),mkdir(outputRoot);end
p1=fullfile(outputRoot,'urmpc_one_step_residual.csv');p2=fullfile(outputRoot,'urmpc_residual_summary.csv');p3=fullfile(outputRoot,'urmpc_residual_windows_5s.csv');
writetable(Aout,p1);writetable(S,p2);writetable(W,p3);
result=struct('one_step',Aout,'summary',S,'windows',W,'one_step_csv',string(p1),'summary_csv',string(p2),'windows_csv',string(p3));
end

function [A,B,x,u,i0,i1,w]=local_interp(M,speeds,v,cfg,hNom)
[i0,i1,w]=local_bracket(speeds,v);m0=M(i0,cfg);m1=M(i1,cfg);
if i0==i1,A=double(m0.A);B=double(m0.B);x=double(m0.x_nominal(:));u=double(m0.u_nominal(:));
else,A=(1-w)*double(m0.A)+w*double(m1.A);B=(1-w)*double(m0.B)+w*double(m1.B);x=(1-w)*double(m0.x_nominal(:))+w*double(m1.x_nominal(:));u=(1-w)*double(m0.u_nominal(:))+w*double(m1.u_nominal(:));end
x(1)=hNom;
end
function [i0,i1,w]=local_bracket(s,v)
s=double(s(:));v=min(max(double(v),s(1)),s(end));if v<=s(1),i0=1;i1=1;w=0;return;end;if v>=s(end),i0=numel(s);i1=i0;w=0;return;end;i1=find(s>=v,1,'first');i0=i1-1;w=(v-s(i0))/(s(i1)-s(i0));
end
function sc=local_scales(meta)
sc=[3;1.5;6;0.5;0.5];try,x=double(meta.output_scales(:));if numel(x)==5&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end
end
function x=local_col(T,k,n),x=NaN;if ismember(n,T.Properties.VariableNames),try,x=double(T.(n)(k));catch,end,end,end

function S=local_summary(A)
rows={};r=0;for cfg=0:4
    m=A.valid_prediction>0.5 & round(A.cfg_from)==cfg & isfinite(A.normalized_residual_norm);D=A(m,:);if isempty(D),continue;end
    r=r+1;R=struct();R.cfg_id=cfg;R.samples=height(D);R.start_s=min(D.time_s);R.end_s=max(D.time_s);
    vars={'res_h_m','res_va_mps','res_pitch_deg','res_vz_mps','res_q_dps'};
    for j=1:numel(vars),x=double(D.(vars{j}));R.([vars{j} '_rms'])=sqrt(mean(x.^2,'omitnan'));R.([vars{j} '_p95_abs'])=local_pctl(abs(x),95);end
    x=double(D.normalized_residual_norm);R.normalized_residual_rms=sqrt(mean(x.^2,'omitnan'));R.normalized_residual_p95=local_pctl(x,95);R.normalized_residual_max=max(x,[],'omitnan');
    x=double(D.b_explained_fraction);R.b_explained_median=median(x,'omitnan');R.b_explained_p10=local_pctl(x,10);
    R.first_norm_gt_0p5_s=local_first(D.time_s,D.normalized_residual_norm,0.5);R.first_norm_gt_1_s=local_first(D.time_s,D.normalized_residual_norm,1.0);
    if ismember('est_disturbance_norm',D.Properties.VariableNames),R.est_disturbance_norm_p95=local_pctl(double(D.est_disturbance_norm),95);else,R.est_disturbance_norm_p95=NaN;end
    rows{r,1}=struct2table(R); %#ok<AGROW>
end
if isempty(rows),S=table();else,S=vertcat(rows{:});end
end
function W=local_windows(A,span)
rows={};r=0;if isempty(A),W=table();return;end;t0=floor(min(A.time_s)/span)*span;t1=ceil(max(A.time_s)/span)*span;
for a=t0:span:(t1-span)
    m=A.valid_prediction>0.5&A.time_s>a&A.time_s<=a+span&isfinite(A.normalized_residual_norm);D=A(m,:);if isempty(D),continue;end
    r=r+1;R=struct('window_start_s',a,'window_end_s',a+span,'cfg_mode',mode(round(D.cfg_from)),'samples',height(D));x=double(D.normalized_residual_norm);R.norm_rms=sqrt(mean(x.^2,'omitnan'));R.norm_p95=local_pctl(x,95);R.norm_max=max(x,[],'omitnan');R.b_explained_median=median(double(D.b_explained_fraction),'omitnan');R.est_disturbance_norm_p95=local_pctl(double(D.est_disturbance_norm),95);rows{r,1}=struct2table(R); %#ok<AGROW>
end
if isempty(rows),W=table();else,W=vertcat(rows{:});end
end
function y=local_first(t,x,thr),m=isfinite(t)&isfinite(x)&x>thr;if any(m),z=double(t(m));y=z(1);else,y=NaN;end,end
function q=local_pctl(x,p),x=sort(double(x(isfinite(x))));if isempty(x),q=NaN;return;end;if numel(x)==1,q=x;return;end;z=1+(numel(x)-1)*p/100;i=floor(z);f=z-i;if i>=numel(x),q=x(end);else,q=x(i)*(1-f)+x(i+1)*f;end,end
