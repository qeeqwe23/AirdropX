function result=airdropx_urmpc_v21d_tube_feasibility(varargin)
%AIRDROPX_URMPC_V21D_TUBE_FEASIBILITY Empirical tube-MPC feasibility audit.
%
% This function DOES NOT alter or deploy the flight controller. It uses only
% the independent v2.1D holdout experiment to build a componentwise empirical
% residual-support box, then audits a globally normalized scheduled LQR
% ancillary feedback and conservative RPI/tightening feasibility.
%
% The empirical max box is a hard support only for the observed holdout
% samples. It is NOT claimed to be a distribution-free deterministic bound
% for all future flights. That limitation is written into every summary.

opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));
bank=local_resolve(root,opts.CandidateBankMat);holdRoot=local_resolve(root,opts.HoldoutRoot);outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
manifestPath=fullfile(holdRoot,'urmpc_v21d_holdout_manifest.csv');if ~isfile(manifestPath),error('AirdropX:URMPC:MissingHoldoutManifest','Missing holdout manifest: %s',manifestPath);end
if ~isfile(bank),error('AirdropX:URMPC:MissingCandidateBank','Missing candidate bank: %s',bank);end
B=load(bank,'ur_models','ur_meta','ur_mpc');if ~isfield(B,'ur_models')||~isfield(B,'ur_meta')||~isfield(B,'ur_mpc'),error('AirdropX:URMPC:BadCandidateBank','Candidate bank missing ur_models/ur_meta/ur_mpc.');end
if logical(opts.RequireDeployReady) && (~isfield(B.ur_meta,'v21_deploy_ready')||~logical(B.ur_meta.v21_deploy_ready)),error('AirdropX:URMPC:CandidateNotDeployReady','v2.1B candidate not deploy_ready.');end
M=readtable(manifestPath,'TextType','string');if isempty(M),error('AirdropX:URMPC:EmptyHoldout','Holdout manifest is empty.');end
Sx=diag(local_state_scales(B.ur_meta));Su=diag(local_mv_scales(B.ur_mpc));
Support=local_holdout_support(M,B.ur_meta,Sx,opts);writetable(Support,fullfile(outRoot,'urmpc_v21d_holdout_residual_support.csv'));
holdoutPass=height(Support)==height(M)&&all(Support.status=="PASS")&&all(M.status=="PASS");

% Evaluate one GLOBAL ancillary input-cost scale. Q=I in normalized state
% coordinates for every scheduled point; R=rScale*I for every point. This
% preserves a unified controller-design rule and avoids cfg-specific gains.
rScales=double(opts.AncillaryRScaleGrid(:));searchRows=cell(numel(rScales),1);cache=cell(numel(rScales),1);
for ri=1:numel(rScales)
    E=local_evaluate_rscale(B,Support,Sx,Su,rScales(ri),opts);cache{ri}=E;
    searchRows{ri}=table(rScales(ri),E.local_pass_count,E.local_total,E.speed_common_pass_count,E.speed_common_total,E.worst_closed_loop_rho, ...
        E.worst_local_input_tightening_ratio,E.worst_local_trust_corner_norm,E.worst_common_input_tightening_ratio,E.worst_common_trust_corner_norm,E.worst_rate_trust_ratio,E.feasible,E.score, ...
        'VariableNames',{'r_scale','local_pass_count','local_total','speed_common_pass_count','speed_common_total','worst_closed_loop_rho', ...
        'worst_local_input_tightening_ratio','worst_local_trust_corner_norm','worst_common_input_tightening_ratio','worst_common_trust_corner_norm','worst_rate_trust_ratio','feasible','minimax_score'});
end
Search=vertcat(searchRows{:});
feas=find(Search.feasible);if ~isempty(feas),[~,j]=min(Search.minimax_score(feas));best=feas(j);selectionPolicy="minimax_feasible_global_rscale";else,[~,best]=min(Search.minimax_score);selectionPolicy="minimax_diagnostic_no_feasible_rscale";end
Selected=cache{best};selectedR=Search.r_scale(best);writetable(Search,fullfile(outRoot,'urmpc_v21d_ancillary_search.csv'));
writetable(Selected.local_grid,fullfile(outRoot,'urmpc_v21d_local_rpi.csv'));writetable(Selected.speed_common,fullfile(outRoot,'urmpc_v21d_speed_common_rpi.csv'));
if ~isempty(Selected.global_switch),writetable(Selected.global_switch,fullfile(outRoot,'urmpc_v21d_global_switch_box.csv'));end

empiricalTubeFeasible=holdoutPass&&logical(Search.feasible(best));
Summary=table(holdoutPass,sum(Support.status=="PASS"),height(Support),selectedR,string(selectionPolicy),Search.local_pass_count(best),Search.local_total(best), ...
    Search.speed_common_pass_count(best),Search.speed_common_total(best),Search.worst_closed_loop_rho(best),Search.worst_local_input_tightening_ratio(best), ...
    Search.worst_local_trust_corner_norm(best),Search.worst_common_input_tightening_ratio(best),Search.worst_common_trust_corner_norm(best),Search.worst_rate_trust_ratio(best), ...
    empiricalTubeFeasible,false,string("observed_independent_holdout_empirical_max_only"), ...
    'VariableNames',{'holdout_pass','holdout_pass_count','holdout_total','selected_r_scale','selection_policy','local_rpi_pass_count','local_rpi_total', ...
    'speed_common_rpi_pass_count','speed_common_rpi_total','worst_closed_loop_rho','worst_local_input_tightening_ratio','worst_local_trust_corner_norm', ...
    'worst_common_input_tightening_ratio','worst_common_trust_corner_norm','worst_rate_trust_ratio','empirical_tube_feasible','deterministic_future_guarantee','support_scope'});
writetable(Summary,fullfile(outRoot,'urmpc_v21d_summary.csv'));

fprintf('\n[UR-MPC v2.1D TUBE FEASIBILITY]\n');
fprintf('  independent holdout: %d/%d PASS\n',sum(Support.status=="PASS"),height(Support));
fprintf('  selected global R scale: %.6g (%s)\n',selectedR,char(selectionPolicy));
fprintf('  local RPI: %d/%d PASS\n',Search.local_pass_count(best),Search.local_total(best));
fprintf('  speed-common RPI: %d/%d PASS\n',Search.speed_common_pass_count(best),Search.speed_common_total(best));
fprintf('  empirical_tube_feasible=%d\n',empiricalTubeFeasible);
fprintf('  NOTE: support is empirical max on independent holdout only; no future deterministic guarantee is claimed.\n');
result=struct('support',Support,'ancillary_search',Search,'selected_r_scale',selectedR,'selection_policy',selectionPolicy, ...
    'local_rpi',Selected.local_grid,'speed_common_rpi',Selected.speed_common,'summary',Summary,'empirical_tube_feasible',empiricalTubeFeasible);
end

function Support=local_holdout_support(M,meta,Sx,o)
rows=cell(height(M),1);sc=diag(Sx).';
for i=1:height(M)
    sp=double(M.speed_mps(i));cfg=double(M.cfg_id(i));out=char(M.output_dir(i));rp=fullfile(out,'urmpc_one_step_residual.csv');tp=fullfile(out,'urmpc_controller_trace.csv');
    n=0;status="MISSING_DATA";vals=NaN(1,5);p95=vals;p99=vals;mx=vals;nvals=vals;np95=vals;np99=vals;nmx=vals;normMax=NaN;normP99=NaN;
    if isfile(rp)&&isfile(tp)
        R=readtable(rp);T=readtable(tp);[tf,loc]=ismembertol(double(R.prediction_from_time_s),double(T.time_s),1e-7,'DataScale',1);R=R(tf,:);T=T(loc(tf),:);
        t0=double(M.eval_start_s(i));t1=double(M.eval_end_s(i));good=double(R.valid_prediction)>0.5 & round(double(R.cfg_from))==cfg & double(R.prediction_from_time_s)>=t0 & double(R.prediction_from_time_s)<=t1;
        R=R(good,:);T=T(good,:);
        if ~isempty(R)
            D=[double(T.actual_h_m)-double(T.requested_h_m),double(T.actual_v_mps)-double(T.nominal_va_mps),double(T.pitch_deg)-double(T.nominal_pitch_deg),double(T.actual_vz_mps),double(T.q_est_dps)];
            stateNorm=sqrt(sum((D./sc).^2,2));Y=[double(R.res_h_m),double(R.res_va_mps),double(R.res_pitch_deg),double(R.res_vz_mps),double(R.res_q_dps)];
            keep=isfinite(stateNorm)&stateNorm<=double(o.TrustStateNormMax)&all(isfinite(Y),2);Y=Y(keep,:);Yn=(Sx\Y.').';n=size(Y,1);
            if n>0
                for j=1:5,vals(j)=sqrt(mean(Y(:,j).^2,'omitnan'));p95(j)=local_pctl(abs(Y(:,j)),95);p99(j)=local_pctl(abs(Y(:,j)),99);mx(j)=max(abs(Y(:,j)),[],'omitnan'); ...
                        nvals(j)=sqrt(mean(Yn(:,j).^2,'omitnan'));np95(j)=local_pctl(abs(Yn(:,j)),95);np99(j)=local_pctl(abs(Yn(:,j)),99);nmx(j)=max(abs(Yn(:,j)),[],'omitnan');end
                nn=sqrt(sum(Yn.^2,2));normMax=max(nn,[],'omitnan');normP99=local_pctl(nn,99);
                if n>=double(o.MinTrustedSamples),status="PASS";else,status="INSUFFICIENT_TRUSTED_DATA";end
            end
        end
    end
    rows{i}=table(sp,cfg,n,vals(1),vals(2),vals(3),vals(4),vals(5),p95(1),p95(2),p95(3),p95(4),p95(5),p99(1),p99(2),p99(3),p99(4),p99(5),mx(1),mx(2),mx(3),mx(4),mx(5), ...
        np95(1),np95(2),np95(3),np95(4),np95(5),np99(1),np99(2),np99(3),np99(4),np99(5),nmx(1),nmx(2),nmx(3),nmx(4),nmx(5),normP99,normMax,status, ...
        'VariableNames',{'speed_mps','cfg_id','trusted_samples','rms_h_m','rms_va_mps','rms_pitch_deg','rms_vz_mps','rms_q_dps', ...
        'p95_h_m','p95_va_mps','p95_pitch_deg','p95_vz_mps','p95_q_dps','p99_h_m','p99_va_mps','p99_pitch_deg','p99_vz_mps','p99_q_dps', ...
        'max_h_m','max_va_mps','max_pitch_deg','max_vz_mps','max_q_dps','norm_p95_h','norm_p95_va','norm_p95_pitch','norm_p95_vz','norm_p95_q', ...
        'norm_p99_h','norm_p99_va','norm_p99_pitch','norm_p99_vz','norm_p99_q','norm_max_h','norm_max_va','norm_max_pitch','norm_max_vz','norm_max_q', ...
        'normalized_residual_norm_p99','normalized_residual_norm_max','status'});
end
Support=vertcat(rows{:});
end

function E=local_evaluate_rscale(B,Support,Sx,Su,rScale,o)
sn=double([B.ur_models(:,1).speed_mps]).';vgrid=(min(sn):double(o.SpeedGridStepMps):max(sn)).';if abs(vgrid(end)-max(sn))>1e-9,vgrid(end+1)=max(sn);end
rows={};k=0;localPass=0;worstRho=0;worstInput=0;worstTrust=0;worstRate=0;store=cell(size(B.ur_models,2),1);
for c=1:size(B.ur_models,2),store{c}=struct('Acl',{},'w',{},'K',{},'head',{},'speed',{});end
for ii=1:numel(vgrid)
    for c=1:size(B.ur_models,2)
        k=k+1;m=local_interp_vertex(B.ur_models,vgrid(ii),c,double(B.ur_meta.Ts));w=local_interp_support(Support,vgrid(ii),c-1);
        An=Sx\(double(m.A)*Sx);Bn=Sx\(double(m.B)*Su);ok=true;msg="";rho=Inf;z=NaN(5,1);tail=NaN;terms=0;uRad=NaN(2,1);inputRatio=Inf;trustCorner=Inf;rateRatio=Inf;K=NaN(2,5);Acl=NaN(5);
        try
            [K,~,p]=dlqr(An,Bn,eye(5),double(rScale)*eye(2));Acl=An-Bn*K;rho=max(abs(p));
            [z,rpiOK,terms,tail]=local_rpi_radius(Acl,w,o);ok=ok&&rpiOK&&rho<1-double(o.StabilityMarginEps);
            [head,~,~]=local_mv_headroom(m,B.ur_meta);uNorm=abs(K)*z;uRad=diag(Su).*uNorm;inputRatio=max(uRad./max(head,eps));trustCorner=norm(z,2);
            rateNorm=abs(K*(Acl-eye(5)))*z+abs(K)*w;ratePhys=diag(Su).*rateNorm;rateLim=double(B.ur_meta.model_validity_rate_limits_per_sample(:));rateRatio=max(ratePhys./max(rateLim,eps));
            ok=ok&&inputRatio<1&&trustCorner<1;
        catch ME
            ok=false;msg=string(ME.identifier)+" | "+string(ME.message);
        end
        if ok,localPass=localPass+1;end;worstRho=max(worstRho,rho);worstInput=max(worstInput,inputRatio);worstTrust=max(worstTrust,trustCorner);worstRate=max(worstRate,rateRatio);
        rows{k}=table(vgrid(ii),c-1,rScale,rho,ok,terms,tail,z(1),z(2),z(3),z(4),z(5),uRad(1),uRad(2),inputRatio,trustCorner,rateRatio,msg, ...
            'VariableNames',{'speed_mps','cfg_id','r_scale','closed_loop_rho','pass','rpi_terms','rpi_tail_bound_norm','z_norm_h','z_norm_va','z_norm_pitch','z_norm_vz','z_norm_q', ...
            'tube_elevator_radius','tube_throttle_radius','input_tightening_ratio','trust_corner_norm','rate_trust_ratio','message'});
        store{c}(end+1)=struct('Acl',Acl,'w',w,'K',K,'head',local_mv_headroom(m,B.ur_meta),'speed',vgrid(ii)); %#ok<AGROW>
    end
end
Local=vertcat(rows{:});
commonRows=cell(size(B.ur_models,2),1);commonPass=0;worstCommonInput=0;worstCommonTrust=0;
for c=1:size(B.ur_models,2)
    [r,conv,it]=local_common_box(store{c},o);inp=Inf;trust=Inf;pass=false;
    if conv
        trust=norm(r,2);inp=0;for j=1:numel(store{c}),u=diag(Su).*(abs(store{c}(j).K)*r);inp=max(inp,max(u./max(store{c}(j).head,eps)));end
        pass=trust<1&&inp<1;if pass,commonPass=commonPass+1;end
    end
    worstCommonInput=max(worstCommonInput,inp);worstCommonTrust=max(worstCommonTrust,trust);
    commonRows{c}=table(c-1,conv,it,r(1),r(2),r(3),r(4),r(5),trust,inp,pass, ...
        'VariableNames',{'cfg_id','converged','iterations','z_norm_h','z_norm_va','z_norm_pitch','z_norm_vz','z_norm_q','trust_corner_norm','input_tightening_ratio','pass'});
end
Common=vertcat(commonRows{:});
% Informational global arbitrary-switch box; NOT a deployment gate because
% cfg drops are known mode transitions with changing nominal equilibria.
allStore=[store{:}];[gr,gconv,git]=local_common_box(allStore,o);Global=table(gconv,git,gr(1),gr(2),gr(3),gr(4),gr(5),norm(gr,2), ...
    'VariableNames',{'converged','iterations','z_norm_h','z_norm_va','z_norm_pitch','z_norm_vz','z_norm_q','trust_corner_norm'});
feasible=(localPass==height(Local))&&(commonPass==height(Common));score=max([worstRho,worstInput,worstTrust,worstCommonInput,worstCommonTrust]);if ~isfinite(score),score=Inf;end
E=struct('local_grid',Local,'speed_common',Common,'global_switch',Global,'local_pass_count',localPass,'local_total',height(Local),'speed_common_pass_count',commonPass, ...
    'speed_common_total',height(Common),'worst_closed_loop_rho',worstRho,'worst_local_input_tightening_ratio',worstInput,'worst_local_trust_corner_norm',worstTrust, ...
    'worst_common_input_tightening_ratio',worstCommonInput,'worst_common_trust_corner_norm',worstCommonTrust,'worst_rate_trust_ratio',worstRate,'feasible',feasible,'score',score);
end

function [r,ok,terms,tail]=local_rpi_radius(A,w,o)
n=size(A,1);r=NaN(n,1);ok=false;terms=0;tail=Inf;if any(~isfinite(A(:)))||any(~isfinite(w(:)))||any(w<0),return;end
rho=max(abs(eig(A)));if ~(isfinite(rho)&&rho<1),return;end
wInf=norm(w,inf);if wInf==0,r=zeros(n,1);ok=true;tail=0;return;end
Am=eye(n);mFound=0;gamma=Inf;
for m=1:double(o.MaxTailBlock)
    Am=Am*A;g=norm(Am,inf);if isfinite(g)&&g<double(o.TailBlockNormMax),mFound=m;gamma=g;break;end
end
if mFound==0,return;end
cblock=0;Ar=eye(n);for j=0:mFound-1,cblock=cblock+norm(Ar,inf);Ar=Ar*A;end
if gamma==0,Q=1;else,target=double(o.RPITailTolerance)*(1-gamma)/(wInf*cblock);if target>=1,Q=0;else,Q=max(0,ceil(log(max(target,realmin))/log(gamma)));end,end
N=Q*mFound;if N>double(o.MaxPowerTerms),return;end
rad=zeros(n,1);Ap=eye(n);for i=1:N,rad=rad+abs(Ap)*w;Ap=Ap*A;end
if gamma==0,tail=0;else,tail=wInf*cblock*(gamma^Q)/(1-gamma);end
r=rad+tail*ones(n,1);terms=N;ok=all(isfinite(r));
end
function [r,conv,it]=local_common_box(S,o)
r=zeros(5,1);conv=false;it=0;if isempty(S),r(:)=NaN;return;end
for k=1:double(o.CommonBoxMaxIterations)
    rn=zeros(5,1);for j=1:numel(S),if any(~isfinite(S(j).Acl(:)))||any(~isfinite(S(j).w(:))),r(:)=NaN;return;end;rn=max(rn,abs(S(j).Acl)*r+S(j).w);end
    if any(~isfinite(rn))||norm(rn,inf)>double(o.CommonBoxAbortRadius),r=rn;it=k;return;end
    if norm(rn-r,inf)<=double(o.CommonBoxTolerance)*(1+norm(rn,inf)),r=rn;conv=true;it=k;return;end
    r=rn;
end
it=double(o.CommonBoxMaxIterations);
end
function w=local_interp_support(S,v,cfg)
idx=find(round(double(S.cfg_id))==round(double(cfg)));if isempty(idx),error('Missing holdout support cfg%d.',cfg);end
sp=double(S.speed_mps(idx));[sp,ord]=sort(sp);idx=idx(ord);cols={'norm_max_h','norm_max_va','norm_max_pitch','norm_max_vz','norm_max_q'};
if v<=sp(1)+1e-9,use=idx(1);elseif v>=sp(end)-1e-9,use=idx(end);else,i1=find(sp>=v,1,'first');i0=i1-1;if abs(v-sp(i0))<1e-9,use=idx(i0);elseif abs(v-sp(i1))<1e-9,use=idx(i1);else,use=[idx(i0) idx(i1)];end,end
W=zeros(numel(use),5);for q=1:numel(use),for j=1:5,W(q,j)=double(S.(cols{j})(use(q)));end,end;w=max(W,[],1).';
end
function m=local_interp_vertex(M,v,cfg,Ts)
speeds=double([M(:,1).speed_mps]).';[speeds,ord]=sort(speeds);M=M(ord,:);
if v<=speeds(1),i0=1;i1=1;ww=0;elseif v>=speeds(end),i0=numel(speeds);i1=i0;ww=0;else,i1=find(speeds>=v,1,'first');i0=i1-1;ww=(v-speeds(i0))/(speeds(i1)-speeds(i0));end
m0=M(i0,cfg);m1=M(i1,cfg);if i0==i1,A=m0.A;BB=m0.B;x=m0.x_nominal;u=m0.u_nominal;h=m0.hidden_elevator_offset;else,A=(1-ww)*m0.A+ww*m1.A;BB=(1-ww)*m0.B+ww*m1.B;x=(1-ww)*m0.x_nominal+ww*m1.x_nominal;u=(1-ww)*m0.u_nominal+ww*m1.u_nominal;h=(1-ww)*m0.hidden_elevator_offset+ww*m1.hidden_elevator_offset;end
m=struct('speed_mps',double(v),'cfg_id',cfg-1,'A',double(A),'B',double(BB),'x_nominal',double(x(:)),'u_nominal',double(u(:)),'hidden_elevator_offset',double(h),'Ts',Ts);
end
function [head,lo,hi]=local_mv_headroom(m,meta)
d=abs(double(meta.elevator_external_delta_limit));pLim=double(meta.physical_elevator_limit(:));tLim=double(meta.throttle_limit(:));
lo=[max(pLim(1),double(m.hidden_elevator_offset)-d);tLim(1)];hi=[min(pLim(2),double(m.hidden_elevator_offset)+d);tLim(2)];u=double(m.u_nominal(:));head=min(u-lo,hi-u);head=max(head,0);
end
function sc=local_state_scales(meta),sc=[3;1.5;6;.5;.5];try,x=double(meta.output_scales(:));if numel(x)==5&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function sc=local_mv_scales(C),sc=[.1;.1];try,x=arrayfun(@(m)double(m.ScaleFactor),C.MV(:));x=x(:);if numel(x)==2&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end,end
function q=local_pctl(x,p),x=sort(double(x(isfinite(x))));if isempty(x),q=NaN;return;end;if numel(x)==1,q=x;return;end;z=1+(numel(x)-1)*p/100;i=floor(z);f=z-i;if i>=numel(x),q=x(end);else,q=x(i)*(1-f)+x(i+1)*f;end,end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.CandidateBankMat="matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate/airdropx_urmpc_v21_corrected_candidate.mat";
o.HoldoutRoot="matlab/results/mpc_physics_v1/urmpc_v21d_holdout_validation";o.OutputRoot="matlab/results/mpc_physics_v1/urmpc_v21d_tube_feasibility";o.RequireDeployReady=true;
o.TrustStateNormMax=1.0;o.MinTrustedSamples=120;o.SpeedGridStepMps=1.0;o.AncillaryRScaleGrid=[0.25 0.5 1 2 4 8 16 32];
o.StabilityMarginEps=1e-8;o.RPITailTolerance=1e-8;o.MaxTailBlock=500;o.TailBlockNormMax=0.95;o.MaxPowerTerms=50000;
o.CommonBoxTolerance=1e-10;o.CommonBoxMaxIterations=20000;o.CommonBoxAbortRadius=1e6;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=char(string(varargin{i}));if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
