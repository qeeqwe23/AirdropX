function result = airdropx_urmpc_build(varargin)
%AIRDROPX_URMPC_BUILD Build one unified adaptive/robust Physics MPC.
%
% UR-MPC v2.0 design principles:
%   1) ONE MPC object for the whole 45..55 m/s, cfg0..cfg4 envelope.
%   2) The prediction model is updated online from the verified physics bank
%      (LPV/adaptive MPC); controller weights/horizons/constraints do not
%      change with cfg or speed.
%   3) Altitude is an MPC state/output. There is no external H->vz PI loop.
%   4) Two load-disturbance channels enter through the same B columns as
%      elevator/throttle.  Their estimator persistence is explicit and
%      auditable (v2.0.5 defaults to static-white for a causal ablation).
%   5) Manipulated-variable limits are ABSOLUTE physical actuator limits,
%      not the old +/-0.10 / +/-0.18 local-deviation box.
%   6) Tuning scales use a Bryson-style engineering normalization derived
%      from the formal performance gates. No cfg-specific tuning is used.
%   7) Absolute MV bounds are physical, while MV increment limits are SOFT
%      model-validity limits inherited from the certified local physics bank.
%      This prevents unvalidated one-sample bang-bang moves without removing
%      physical recovery authority over multiple samples.
%
% Requires the already-certified v1.6 physics bank; no trim/ID/MEX rebuild.

opts = local_options(varargin{:});
root = local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));
addpath(fullfile(root,'matlab','mpc'));
addpath(fullfile(root,'matlab','mpc_auto'));
outRoot=local_resolve(root,opts.OutputRoot); if ~isfolder(outRoot),mkdir(outRoot);end

src = local_resolve(root,opts.SourceBankMat);
if ~isfile(src), error('AirdropX:URMPC:MissingPhysicsBank','Missing certified physics bank: %s',src); end
S = load(src,'v32_nodes','speed_nodes','physics_mpc_meta');
if ~isfield(S,'v32_nodes') || isempty(S.v32_nodes)
    error('AirdropX:URMPC:BadPhysicsBank','Certified bank does not contain v32_nodes.');
end
nodes = S.v32_nodes(:);
speeds = double([nodes.speed_mps]).';
[speeds,ord] = sort(speeds); nodes = nodes(ord);
if numel(nodes) < 3 || any(abs(speeds(:)-[45;50;55])>1e-6)
    error('AirdropX:URMPC:EnvelopeMismatch','Expected certified speed vertices [45 50 55] m/s.');
end
nCfg = 5; Ts = double(opts.Ts);

% ----- Extract and augment all 15 certified local physics models.
ur_models = repmat(struct('speed_mps',NaN,'cfg_id',NaN,'A',[],'B',[], ...
    'plant',[],'x_nominal',[],'u_nominal',[],'hidden_elevator_offset',NaN),numel(nodes),nCfg);
reportRows = [];
for ni=1:numel(nodes)
    if numel(nodes(ni).plant_bank)<nCfg || numel(nodes(ni).trim_bank)<nCfg
        error('AirdropX:URMPC:IncompletePhysicsBank','Speed %.1f does not have 5 plant/trim models.',speeds(ni));
    end
    hidden = double(nodes(ni).mpc_meta.hidden_elevator_offset);
    for cfg=1:nCfg
        P4 = nodes(ni).plant_bank{cfg};
        if isempty(P4), error('AirdropX:URMPC:MissingVertex','Missing V=%.1f cfg%d model.',speeds(ni),cfg-1); end
        [A4,B4,C4,D4] = ssdata(P4); %#ok<ASGLU>
        if ~isequal(size(A4),[4 4]) || ~isequal(size(B4),[4 2])
            error('AirdropX:URMPC:BadVertexDimension','Expected 4x4 A and 4x2 B at V=%.1f cfg%d.',speeds(ni),cfg-1);
        end
        % Absolute altitude state h, followed by the certified longitudinal
        % state [Va pitch vz q]. h[k+1]=h[k]+Ts*vz[k].
        A5 = zeros(5,5); B5 = zeros(5,2);
        A5(1,1)=1; A5(1,4)=Ts;
        A5(2:5,2:5)=double(A4);
        B5(2:5,:)=double(B4);

        % Add two unmeasured LOAD-disturbance inputs through B itself.
        % This is the standard offset-free load-disturbance construction:
        % x+ = A x + B u + B d.
        P5 = ss(A5,[B5 B5],eye(5),zeros(5,4),Ts);
        P5 = setmpcsignals(P5,'MV',[1 2],'UD',[3 4]);
        P5.StateName={'h_m','Va_mps','pitch_deg','vz_mps','q_dps'};
        P5.OutputName=P5.StateName;
        P5.InputName={'elevator_physical','throttle','load_elevator','load_throttle'};

        tr = nodes(ni).trim_bank(cfg);
        xnom = [double(opts.ReferenceAltitudeM); double(tr.airspeed_mps); double(tr.pitch_deg); ...
            local_field(tr,'vz_up_mps',0); local_field(tr,'q_dps',0)];
        elev = double(nodes(ni).mpc_meta.physical_elevator_nominals(cfg));
        thr = double(nodes(ni).mpc_meta.throttle_nominals(cfg));
        unom = [elev;thr];
        if any(~isfinite([xnom(:);unom(:);hidden]))
            error('AirdropX:URMPC:NonfiniteVertex','Nonfinite nominal at V=%.1f cfg%d.',speeds(ni),cfg-1);
        end
        ur_models(ni,cfg)=struct('speed_mps',speeds(ni),'cfg_id',cfg-1,'A',A5,'B',B5, ...
            'plant',P5,'x_nominal',xnom,'u_nominal',unom,'hidden_elevator_offset',hidden);

        lam=eig(A4); rho=max(abs(lam)); tau=local_short_tau(lam,Ts);
        [eMin,eMax]=local_elevator_physical_bounds(hidden,opts);
        reportRows=[reportRows; speeds(ni),cfg-1,rho,tau,elev,thr,hidden,eMin,eMax]; %#ok<AGROW>
    end
end
R = array2table(reportRows,'VariableNames',{'speed_mps','cfg_id','spectral_radius','short_mode_tau_s', ...
    'elevator_nominal','throttle_nominal','hidden_elevator_offset','elevator_physical_min','elevator_physical_max'});

% ----- Derive ONE trim-manifold TRUST envelope for diagnostics/scaling.
% IMPORTANT: this is NOT an actuator hard limit.  The previous v2.0 build
% incorrectly intersected the physical MV limits with this local-model trust
% envelope.  The V50 nonlinear run proved that the controller then exhausted
% the artificial trim envelope (elevator/throttle) while substantial real
% actuator authority was still available.  Keep this envelope as a model-
% validity diagnostic and for numerical scaling only; hard MV constraints are
% the actual software/plant reachable limits below.
[eJump,tJump]=local_max_adjacent_trim_jump(ur_models);
idE=local_meta_option(S,'ElevatorAmplitude',0.012);idT=local_meta_option(S,'ThrottleAmplitude',0.025);
eMargin=max(eJump,abs(idE));tMargin=max(tJump,abs(idT));
[eRateLimit,tRateLimit,rateLimitSource]=local_model_validity_rate_limits(nodes,idE,idT,opts);
opts.DerivedRateLimits=[eRateLimit;tRateLimit];
eDesignMin=max(double(opts.PhysicalElevatorMin),min(R.elevator_nominal)-eMargin);
eDesignMax=min(double(opts.PhysicalElevatorMax),max(R.elevator_nominal)+eMargin);
tDesignMin=max(double(opts.ThrottleMin),min(R.throttle_nominal)-tMargin);
tDesignMax=min(double(opts.ThrottleMax),max(R.throttle_nominal)+tMargin);
if ~(eDesignMin<eDesignMax&&tDesignMin<tDesignMax),error('AirdropX:URMPC:BadDerivedMVEnvelope','Derived unified MV envelope is invalid.');end
opts.DerivedMVBounds=[eDesignMin eDesignMax;tDesignMin tDesignMax];

% ----- Derive one prediction horizon from the whole verified envelope.
if isempty(opts.PredictionHorizon)
    tau = R.short_mode_tau_s(isfinite(R.short_mode_tau_s)&R.short_mode_tau_s>0);
    if isempty(tau), tauDesign=1.5; else
        tau=sort(tau); tauDesign=tau(max(1,ceil(0.90*numel(tau))));
    end
    predTime=min(max(4.0,3.0*tauDesign),8.0);
    Np=max(30,ceil(predTime/Ts));
else
    Np=max(10,round(double(opts.PredictionHorizon)));
    tauDesign=NaN; predTime=Np*Ts;
end
if isempty(opts.ControlHorizon)
    Nc=local_block_horizon(Np);
else
    Nc=double(opts.ControlHorizon(:).');
    if sum(Nc)~=Np,error('AirdropX:URMPC:BadControlHorizon','Blocked ControlHorizon must sum to PredictionHorizon.');end
end

% ----- Representative model only defines controller dimensions. Runtime
% model/nominal are replaced every sample by mpcmoveAdaptive.
[~,midN]=min(abs(speeds-50)); midCfg=3; % cfg2
base = ur_models(midN,midCfg);
ur_mpc = mpc(base.plant,Ts,Np,Nc);
ur_mpc.Model.Nominal.X=base.x_nominal;
ur_mpc.Model.Nominal.Y=base.x_nominal;
ur_mpc.Model.Nominal.U=[base.u_nominal;0;0];
ur_mpc.Model.Nominal.DX=zeros(5,1);

% ----- Estimator disturbance structure (v2.0.5 ablation).
% Keep the SAME two unmeasured load inputs B*d in the adaptive plant, but
% remove persistence from the disturbance estimator for this causal test.
% v2.0.4 proved that hidden output-disturbance integrators were not the main
% source of the cfg2 chronic divergence.  Its remaining two integrated UD
% states then tracked the equivalent load residual almost one-for-one while
% the MPC commanded the opposite MV correction.  Before choosing an
% arbitrary leaky pole, run one zero-persistence ablation using the MPC
% Toolbox-supported unity-gain static input-disturbance model.
%
% IMPORTANT: this is NOT a final estimator tuning.  It is a single-variable
% experiment: B*d geometry, plant models, costs, constraints, rate trust and
% output-disturbance policy are unchanged; only disturbance persistence is
% removed.
inputDistPolicy=lower(string(opts.InputDisturbancePolicy));
switch inputDistPolicy
    case "static_white"
        % Two UD outputs, direct unity gain, zero internal states.
        setindist(ur_mpc,'model',tf(eye(2)));
    case "integrators"
        % v2.0.4 baseline retained for reproducibility.
        setindist(ur_mpc,'integrators');
    otherwise
        error('AirdropX:URMPC:BadInputDisturbancePolicy', ...
            'InputDisturbancePolicy must be static_white or integrators; got %s.',inputDistPolicy);
end
zeroOutDist=tf(zeros(5,1));
setoutdist(ur_mpc,'model',zeroOutDist);
[inDist,inDistChannels]=getindist(ur_mpc);
[outDist,outDistChannels]=getoutdist(ur_mpc);
inDistSS=ss(inDist);outDistSS=ss(outDist);
nInputDistStates=size(inDistSS.A,1);
nOutputDistStates=size(outDistSS.A,1);
if inputDistPolicy=="static_white"
    if nInputDistStates~=0
        error('AirdropX:URMPC:UnexpectedInputDisturbanceModel', ...
            'Static-white ablation requires zero input-disturbance states; got %d.',nInputDistStates);
    end
elseif nInputDistStates~=2 || numel(inDistChannels)~=2
    error('AirdropX:URMPC:UnexpectedInputDisturbanceModel', ...
        'Integrator baseline requires exactly two input-disturbance states; got %d states, channels=%s.', ...
        nInputDistStates,mat2str(inDistChannels));
end
if nOutputDistStates~=0 || ~isempty(outDistChannels)
    error('AirdropX:URMPC:UnexpectedOutputDisturbanceModel', ...
        'Output disturbance model must be disabled; got %d states, channels=%s.', ...
        nOutputDistStates,mat2str(outDistChannels));
end

% Absolute PHYSICAL actuator envelope.  Exact elevator reachability is
% tightened online from the once-latched hidden offset and external-delta
% channel limit.  Do not hard-clip to the trim-manifold trust envelope: doing
% so removes recovery authority exactly when nonlinear/model-mismatch loads
% grow after a payload release.
ur_mpc.MV(1).Min=double(opts.PhysicalElevatorMin);
ur_mpc.MV(1).Max=double(opts.PhysicalElevatorMax);
ur_mpc.MV(2).Min=double(opts.ThrottleMin);
ur_mpc.MV(2).Max=double(opts.ThrottleMax);
% Model-validity increment envelope.  V2.0.1 proved that physical absolute
% bounds alone allow the optimizer to use one-sample moves far outside the
% perturbation size/rate used to certify the local A/B models (at cfg1, the
% nonlinear run repeatedly jumped throttle by O(0.3..0.5) per 0.1 s).  Keep
% the physical absolute bounds HARD, but make the certified increment limits
% SOFT so the optimizer may exceed them only when the tracking problem truly
% justifies paying slack.  This follows the MPC constraint separation:
% physical range = actuator safety; increment range = prediction-model trust.
if logical(opts.ApplyModelValidityRateLimits)
    ur_mpc.MV(1).RateMin=-eRateLimit; ur_mpc.MV(1).RateMax=eRateLimit;
    ur_mpc.MV(2).RateMin=-tRateLimit; ur_mpc.MV(2).RateMax=tRateLimit;
    rateECR=double(opts.ModelValidityRateECR);
    ur_mpc.MV(1).RateMinECR=rateECR; ur_mpc.MV(1).RateMaxECR=rateECR;
    ur_mpc.MV(2).RateMinECR=rateECR; ur_mpc.MV(2).RateMaxECR=rateECR;
end

% Bryson-style normalization: scale factors are 2x the formal RMS gates for
% h/Va/vz/q, while pitch gets a 6-deg natural-trim deviation allowance.
sc=double(opts.OutputScales(:).');
for j=1:5, ur_mpc.OV(j).ScaleFactor=sc(j); end
ur_mpc.Weights.OutputVariables=double(opts.OutputWeights(:).');
ur_mpc.Weights.ManipulatedVariables=double(opts.MVWeights(:).');
ur_mpc.Weights.ManipulatedVariablesRate=double(opts.MVRateWeights(:).');
ur_mpc.MV(1).ScaleFactor=max(0.05,0.5*(eDesignMax-eDesignMin));
ur_mpc.MV(2).ScaleFactor=max(0.05,0.5*(tDesignMax-tDesignMin));
ur_mpc.Weights.ECR=double(opts.ECRWeight);

% Broad SOFT flight-envelope bounds only. Tracking is driven by the cost;
% these constraints exist to keep the QP numerically/safely meaningful.
ovMin=[0,30,-25,-10,-20]; ovMax=[10000,70,25,10,20];
for j=1:5
    ur_mpc.OV(j).Min=ovMin(j); ur_mpc.OV(j).Max=ovMax(j);
    ur_mpc.OV(j).MinECR=1; ur_mpc.OV(j).MaxECR=1;
end

% Estimator policy is explicit above: exactly two integrated input-load
% disturbance states and zero output-disturbance states.  Do not allow the
% toolbox defaults to silently add measured-output integrators.

% ----- Persist design inputs BEFORE certification.  A failed certificate must
% still leave enough audit data to diagnose the design without retuning it.
runtimePolicy="physical absolute MV hard bounds; trim envelope diagnostic/scaling; certified model-validity MV-rate soft bounds";
design=table(Ts,Np,predTime,string(mat2str(Nc)),tauDesign,eDesignMin,eDesignMax,tDesignMin,tDesignMax,eMargin,tMargin,eJump,tJump,idE,idT, ...
    double(opts.PhysicalElevatorMin),double(opts.PhysicalElevatorMax),double(opts.ThrottleMin),double(opts.ThrottleMax), ...
    eRateLimit,tRateLimit,double(opts.ModelValidityRateECR),string(rateLimitSource),runtimePolicy, ...
    'VariableNames',{'Ts_s','prediction_horizon','prediction_time_s','control_horizon','tau90_s','elevator_design_min','elevator_design_max','throttle_design_min','throttle_design_max','elevator_margin','throttle_margin','max_adjacent_elevator_jump','max_adjacent_throttle_jump','source_id_elevator_amplitude','source_id_throttle_amplitude', ...
    'controller_elevator_hard_min','controller_elevator_hard_max','controller_throttle_hard_min','controller_throttle_hard_max', ...
    'elevator_model_validity_rate_per_sample','throttle_model_validity_rate_per_sample','model_validity_rate_ecr','model_validity_rate_source','runtime_mv_policy'});
writetable(R,fullfile(outRoot,'urmpc_model_envelope_report.csv'));
writetable(design,fullfile(outRoot,'urmpc_design_parameters.csv'));
estimatorDesign=table(nInputDistStates,nOutputDistStates,string(mat2str(inDistChannels)),string(mat2str(outDistChannels)), ...
    string(inputDistPolicy),"disabled_zero_static", ...
    'VariableNames',{'input_disturbance_states','output_disturbance_states','input_integrator_channels','output_integrator_channels', ...
    'input_disturbance_policy','output_disturbance_policy'});
writetable(estimatorDesign,fullfile(outRoot,'urmpc_estimator_design.csv'));

% ----- Preflight every vertex through THE SAME adaptive MPC object.
preflight = local_vertex_preflight(ur_mpc,ur_models,opts);
writetable(preflight,fullfile(outRoot,'urmpc_vertex_preflight.csv'));
if any(~preflight.pass)
    error('AirdropX:URMPC:VertexPreflightFailed','Unified controller failed %d/%d vertex preflight cases.',sum(~preflight.pass),height(preflight));
end

% ----- Linear drop-transition certification: old cfg state/input -> new cfg
% model/nominal, one controller, physical constraints, no recovery layer.
transition = local_transition_cert(ur_mpc,ur_models,opts);
writetable(transition,fullfile(outRoot,'urmpc_linear_drop_cert.csv'));
if any(~transition.pass)
    error('AirdropX:URMPC:LinearTransitionCertFailed','Unified controller failed %d/%d linear cfg-transition certificate cases.',sum(~transition.pass),height(transition));
end

ur_meta=struct();
ur_meta.version='urmpc_v2_0_5_input_estimator_ablation';
ur_meta.architecture=sprintf('ONE adaptive MPC; LPV A/B interpolation; altitude-in-MPC; two B-load UD channels with input-disturbance policy=%s; NO output-disturbance integrators; physical MV hard limits; certified soft MV-rate trust limits; trim MV envelope diagnostic only; no recovery controller',char(inputDistPolicy));
ur_meta.source_bank=string(src); ur_meta.speed_nodes_mps=speeds; ur_meta.n_cfg=nCfg;
ur_meta.Ts=Ts; ur_meta.Np=Np; ur_meta.ControlHorizon=Nc; ur_meta.prediction_time_s=predTime; ur_meta.short_tau_design_s=tauDesign;
ur_meta.output_scales=double(opts.OutputScales(:)); ur_meta.output_weights=double(opts.OutputWeights(:));
ur_meta.mv_weights=double(opts.MVWeights(:)); ur_meta.mv_rate_weights=double(opts.MVRateWeights(:));
ur_meta.hidden_offsets_by_speed=arrayfun(@(n)double(n.mpc_meta.hidden_elevator_offset),nodes(:));
ur_meta.reference_mass_kg=double(opts.ReferenceMassKg); ur_meta.cargo_mass_kg=double(opts.CargoMassKg);
ur_meta.elevator_external_delta_limit=double(opts.ExternalElevatorDeltaLimit);
ur_meta.physical_elevator_limit=[double(opts.PhysicalElevatorMin) double(opts.PhysicalElevatorMax)];
ur_meta.throttle_limit=[double(opts.ThrottleMin) double(opts.ThrottleMax)];
ur_meta.design_mv_bounds=[eDesignMin eDesignMax;tDesignMin tDesignMax]; % trust envelope, NOT runtime hard bounds
ur_meta.design_mv_margin=[eMargin;tMargin];
ur_meta.runtime_mv_policy='physical_absolute_hard_plus_model_validity_rate_soft';
ur_meta.design_mv_bounds_are_hard=false;
ur_meta.max_adjacent_trim_jump=[eJump;tJump];
ur_meta.source_id_excitation=[idE;idT];
ur_meta.input_disturbance_state_count=nInputDistStates;
ur_meta.input_disturbance_policy=char(inputDistPolicy);
ur_meta.output_disturbance_state_count=nOutputDistStates;
ur_meta.output_disturbance_policy='disabled_zero_static';
ur_meta.model_validity_rate_constraints_enabled=logical(opts.ApplyModelValidityRateLimits);
ur_meta.model_validity_rate_limits_per_sample=[eRateLimit;tRateLimit];
ur_meta.model_validity_rate_ecr=double(opts.ModelValidityRateECR);
ur_meta.model_validity_rate_source=char(rateLimitSource);
ur_meta.hard_rate_constraints_enabled=false;
ur_meta.vertex_preflight_pass=all(preflight.pass); ur_meta.linear_drop_cert_pass=all(transition.pass);
ur_meta.created_at=datetime('now');

% Keep v32_nodes/speed_nodes in the new bank only for certified trim metadata
% and initial-condition setup. Runtime never uses the old controller objects.
v32_nodes=nodes; speed_nodes=speeds; %#ok<NASGU>
outBank=local_resolve(root,opts.OutputBankMat); d=fileparts(outBank);if ~isfolder(d),mkdir(d);end
save(outBank,'ur_mpc','ur_models','ur_meta','v32_nodes','speed_nodes','-v7.3');

local_write_manifest(outRoot,outBank,ur_meta,preflight,transition);
result=struct('bank_mat',string(outBank),'controller',ur_mpc,'models',ur_models,'meta',ur_meta, ...
    'model_report',R,'vertex_preflight',preflight,'linear_drop_cert',transition);
fprintf('\n[UR-MPC v2.0] BUILD COMPLETE\n');
fprintf('  One MPC object: YES\n  Np=%d (%.2f s), ControlHorizon=%s\n',Np,predTime,mat2str(Nc));
fprintf('  Vertex preflight: %d/%d PASS\n',sum(preflight.pass),height(preflight));
fprintf('  Linear cfg transitions: %d/%d PASS\n',sum(transition.pass),height(transition));
fprintf('  Bank: %s\n',outBank);
end

function T=local_vertex_preflight(C,M,o)
% Dense scheduling-grid preflight: this checks the interpolation law itself,
% not only the three stored speed vertices.
sn=double([M(:,1).speed_mps]).';vgrid=(min(sn):double(o.PreflightSpeedStepMps):max(sn)).';
if abs(vgrid(end)-max(sn))>1e-9,vgrid(end+1)=max(sn);end
rows={}; k=0;
for ii=1:numel(vgrid)
    for c=1:size(M,2)
        k=k+1; m=local_interp_vertex(M,vgrid(ii),c,double(o.Ts));
        Nom=local_nominal(m,double(o.ReferenceAltitudeM));
        % mpcstate(C) is initialized at C.Model.Nominal (the V50/cfg2
        % anchor).  Each preflight row is an independent operating point, so
        % initialize the absolute plant-state estimate and previous MV at THIS
        % row's moving nominal.  Otherwise the first Kalman update sees a
        % fictitious operating-point error and produces a nonzero nominal move.
        st=local_state_at_nominal(C,Nom,m.u_nominal);
        y=Nom.Y(:).'; r=y; opt=local_options_for_hidden(m.hidden_elevator_offset,o);
        ok=false;its=NaN;qp="";de=NaN;dt=NaN;signOK=false;ctr=rank(ctrb(m.A,m.B));
        try
            [u,info]=mpcmoveAdaptive(C,st,m.plant,Nom,y,r,[],opt);
            its=double(info.Iterations); qp=string(info.QPCode); de=u(1)-m.u_nominal(1);dt=u(2)-m.u_nominal(2);
            nominalOK=its>0&&all(isfinite(u))&&abs(de)<0.03&&abs(dt)<0.03;
            % Physical sign sanity: low + descending must not produce a
            % materially more nose-down elevator than trim (negative is
            % nose-up in this MQ9 convention).
            st2=local_state_at_nominal(C,Nom,m.u_nominal);y2=y;y2(1)=y2(1)-1;y2(4)=-0.2;
            [u2,info2]=mpcmoveAdaptive(C,st2,m.plant,Nom,y2,r,[],opt);
            signOK=double(info2.Iterations)>0&&isfinite(u2(1))&&(u2(1)<=m.u_nominal(1)+0.01);
            ok=nominalOK&&signOK&&(ctr==5);
        catch ME
            qp="EX:"+string(ME.identifier);
        end
        rows(k,:)={m.speed_mps,m.cfg_id,ok,its,qp,de,dt,signOK,ctr}; %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',{'speed_mps','cfg_id','pass','iterations','qp_code','nominal_dElev','nominal_dThrottle','nose_up_sign_pass','controllability_rank'});
end

function m=local_interp_vertex(M,v,cfg,Ts)
speeds=double([M(:,1).speed_mps]).';[speeds,ord]=sort(speeds);M=M(ord,:);
if v<=speeds(1),i0=1;i1=1;w=0;elseif v>=speeds(end),i0=numel(speeds);i1=i0;w=0;else,i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));end
m0=M(i0,cfg);m1=M(i1,cfg);if i0==i1,A=m0.A;B=m0.B;x=m0.x_nominal;u=m0.u_nominal;h=m0.hidden_elevator_offset;else,A=(1-w)*m0.A+w*m1.A;B=(1-w)*m0.B+w*m1.B;x=(1-w)*m0.x_nominal+w*m1.x_nominal;u=(1-w)*m0.u_nominal+w*m1.u_nominal;h=(1-w)*m0.hidden_elevator_offset+w*m1.hidden_elevator_offset;end
P=ss(A,[B B],eye(5),zeros(5,4),Ts);P=setmpcsignals(P,'MV',[1 2],'UD',[3 4]);m=struct('speed_mps',double(v),'cfg_id',cfg-1,'A',A,'B',B,'plant',P,'x_nominal',x,'u_nominal',u,'hidden_elevator_offset',h);
end
function T=local_transition_cert(C,M,o)
rows={};k=0;Ts=double(o.Ts);N=max(1,ceil(double(o.LinearTransitionSimS)/Ts));
for i=1:size(M,1)
    for c=1:size(M,2)-1
        k=k+1; old=M(i,c); nw=M(i,c+1);
        x=old.x_nominal(:); x(1)=double(o.ReferenceAltitudeM); u=old.u_nominal(:);
        Nom=local_nominal(nw,double(o.ReferenceAltitudeM));
        st=mpcstate(C); st.LastMove=u;
        % mpcstate.Plant is in ABSOLUTE engineering units (it is not a
        % deviation state).  At the instant of a cfg switch the physical
        % aircraft is still at the old-cfg absolute state x.
        try,if numel(st.Plant)==5,st.Plant=x(:);end,catch,end
        r=[double(o.ReferenceAltitudeM),nw.x_nominal(2),nw.x_nominal(3),0,0];
        opt=local_options_for_hidden(nw.hidden_elevator_offset,o);
        finite=true;maxH=0;maxV=0;maxVz=0;maxQ=0;qpFails=0;
        maxStepE=0;maxStepT=0;rateExceedCount=0;maxSlack=0;lastU=u(:);
        for n=1:N
            y=x(:).';
            try
                [uCmd,info]=mpcmoveAdaptive(C,st,nw.plant,Nom,y,r,[],opt);
                if double(info.Iterations)<=0,qpFails=qpFails+1;end
                if isfield(info,'Slack') && isfinite(double(info.Slack)),maxSlack=max(maxSlack,double(info.Slack));end
            catch
                finite=false;break;
            end
            if any(~isfinite(uCmd)),finite=false;break;end
            u=double(uCmd(:));
            step=abs(u-lastU);maxStepE=max(maxStepE,step(1));maxStepT=max(maxStepT,step(2));
            if isfield(o,'DerivedRateLimits') && numel(o.DerivedRateLimits)>=2 && any(step>double(o.DerivedRateLimits(:))+1e-9),rateExceedCount=rateExceedCount+1;end
            lastU=u;
            % Exact affine use of the certified NEW local model.
            dx=x-Nom.X(:); du=u-nw.u_nominal(:);
            x=Nom.X(:)+nw.A*dx+nw.B*du;
            maxH=max(maxH,abs(x(1)-o.ReferenceAltitudeM));
            maxV=max(maxV,abs(x(2)-nw.x_nominal(2)));
            maxVz=max(maxVz,abs(x(4)));maxQ=max(maxQ,abs(x(5)));
            if any(~isfinite(x)),finite=false;break;end
        end
        final=[abs(x(1)-o.ReferenceAltitudeM),abs(x(2)-nw.x_nominal(2)),abs(x(4)),abs(x(5))];
        pass=finite&&qpFails==0&&final(1)<=1.0&&final(2)<=0.5&&final(3)<=0.20&&final(4)<=0.20;
        rows(k,:)={nw.speed_mps,c-1,c,pass,qpFails,maxH,maxV,maxVz,maxQ,final(1),final(2),final(3),final(4),maxStepE,maxStepT,rateExceedCount,maxSlack}; %#ok<AGROW>
    end
end
T=cell2table(rows,'VariableNames',{'speed_mps','cfg_from','cfg_to','pass','qp_fail_count','max_h_error_m','max_va_error_mps','max_vz_mps','max_q_dps', ...
    'final_h_error_m','final_va_error_mps','final_vz_mps','final_q_dps','max_elevator_step','max_throttle_step','rate_soft_exceed_count','max_slack'});
end

function Nom=local_nominal(m,h)
Nom=struct('X',[double(h);m.x_nominal(2:5)],'U',[m.u_nominal(:);0;0], ...
    'Y',[double(h);m.x_nominal(2:5)],'DX',zeros(5,1));
end
function st=local_state_at_nominal(C,Nom,uNom)
st=mpcstate(C);
% Plant/LastMove are absolute engineering values.  Disturbance/noise states
% remain at their zero defaults so the nominal point contains no artificial
% estimator bias.
try,if numel(st.Plant)==numel(Nom.X),st.Plant=Nom.X(:);end,catch,end
st.LastMove=double(uNom(:));
end
function opt=local_options_for_hidden(hidden,o)
% Certification must use the same hard-authority policy as runtime.  The
% trim-derived envelope is deliberately NOT intersected here.
[eMin,eMax]=local_elevator_physical_bounds(hidden,o);
tMin=double(o.ThrottleMin);tMax=double(o.ThrottleMax);
opt=mpcmoveopt;opt.MVMin=[eMin,tMin];opt.MVMax=[eMax,tMax];
end
function [lo,hi]=local_elevator_physical_bounds(hidden,o)
d=abs(double(o.ExternalElevatorDeltaLimit));
lo=max(double(o.PhysicalElevatorMin),double(hidden)-d);
hi=min(double(o.PhysicalElevatorMax),double(hidden)+d);
if ~(isfinite(lo)&&isfinite(hi)&&lo<hi),error('AirdropX:URMPC:BadElevatorEnvelope','Invalid physical elevator range.');end
end
function [eJump,tJump]=local_max_adjacent_trim_jump(M)
e=[];t=[];
for i=1:size(M,1)
    for c=1:size(M,2)-1,e(end+1)=abs(M(i,c+1).u_nominal(1)-M(i,c).u_nominal(1));t(end+1)=abs(M(i,c+1).u_nominal(2)-M(i,c).u_nominal(2));end %#ok<AGROW>
end
for i=1:size(M,1)-1
    for c=1:size(M,2),e(end+1)=abs(M(i+1,c).u_nominal(1)-M(i,c).u_nominal(1));t(end+1)=abs(M(i+1,c).u_nominal(2)-M(i,c).u_nominal(2));end %#ok<AGROW>
end
eJump=max(e,[],'omitnan');tJump=max(t,[],'omitnan');if ~isfinite(eJump),eJump=0;end;if ~isfinite(tJump),tJump=0;end
end
function [eRate,tRate,source]=local_model_validity_rate_limits(nodes,idE,idT,o)
% Use the rate envelope that was already certified when the source local
% physics models were identified/built.  If a future source bank omits that
% metadata, fall back to the ID excitation magnitude rather than inventing a
% new cfg-specific number.  The minimum positive value across speed nodes is
% used so ONE controller respects the validity of every scheduled vertex.
e=[];t=[];
for i=1:numel(nodes)
    try
        m=nodes(i).mpc_meta;
        if isfield(m,'elevator_deviation_rate_limit'),v=abs(double(m.elevator_deviation_rate_limit));if isscalar(v)&&isfinite(v)&&v>0,e(end+1)=v;end,end %#ok<AGROW>
        if isfield(m,'throttle_deviation_rate_limit'),v=abs(double(m.throttle_deviation_rate_limit));if isscalar(v)&&isfinite(v)&&v>0,t(end+1)=v;end,end %#ok<AGROW>
    catch
    end
end
if logical(o.ApplyLegacyArtificialRateLimits)
    eRate=abs(double(o.LegacyElevatorRatePerSample));tRate=abs(double(o.LegacyThrottleRatePerSample));source="legacy_explicit_override";
elseif ~isempty(e)&&~isempty(t)
    eRate=min(e);tRate=min(t);source="source_bank_mpc_meta";
else
    eRate=abs(double(idE));tRate=abs(double(idT));source="source_id_excitation_fallback";
end
if ~(isfinite(eRate)&&eRate>0&&isfinite(tRate)&&tRate>0),error('AirdropX:URMPC:BadModelValidityRate','Cannot derive finite positive unified MV-rate trust limits.');end
end
function v=local_meta_option(S,name,fallback)
v=double(fallback);try,if isfield(S,'physics_mpc_meta')&&isfield(S.physics_mpc_meta,'options')&&isfield(S.physics_mpc_meta.options,name),x=double(S.physics_mpc_meta.options.(name));if isscalar(x)&&isfinite(x),v=x;end,end,catch,end
end
function tau=local_short_tau(lam,Ts)
tau=NaN;v=[];
for k=1:numel(lam)
    z=lam(k);m=abs(z);a=abs(angle(z));
    % Exclude the nearly-neutral phugoid/integrator-like modes when selecting
    % the finite MPC horizon; the altitude state is handled explicitly.
    if isfinite(m)&&m>0.05&&m<0.9995&&(a>0.02||m<0.995)
        t=-Ts/log(m);if isfinite(t)&&t>0&&t<20,v(end+1)=t;end %#ok<AGROW>
    end
end
if ~isempty(v),tau=max(v);end
end
function ch=local_block_horizon(Np)
seed=[1 1 1 2 3 5 8];s=sum(seed);
if Np>s,ch=[seed Np-s];return;end
ch=[];rem=Np;
for x=seed
    if rem<=0,break;end
    y=min(x,rem);ch(end+1)=y;rem=rem-y; %#ok<AGROW>
end
if rem>0,ch(end+1)=rem;end
end
function v=local_field(s,n,d),if isfield(s,n)&&isfinite(double(s.(n))),v=double(s.(n));else,v=double(d);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function local_write_manifest(outRoot,bank,meta,pre,trans)
fid=fopen(fullfile(outRoot,'URMPC_V2_MANIFEST.txt'),'w');if fid<0,return;end;c=onCleanup(@()fclose(fid));
fprintf(fid,'AirdropX Unified Robust MPC v2.0.5 input-estimator ablation\n');fprintf(fid,'bank=%s\n',bank);fprintf(fid,'architecture=%s\n',meta.architecture);
fprintf(fid,'Ts=%.6g\nNp=%d\nControlHorizon=%s\n',meta.Ts,meta.Np,mat2str(meta.ControlHorizon));
fprintf(fid,'output_scales=%s\noutput_weights=%s\n',mat2str(meta.output_scales.'),mat2str(meta.output_weights.'));
fprintf(fid,'mv_weights=%s\nmv_rate_weights=%s\n',mat2str(meta.mv_weights.'),mat2str(meta.mv_rate_weights.'));
fprintf(fid,'vertex_preflight=%d/%d\n',sum(pre.pass),height(pre));fprintf(fid,'linear_transition_cert=%d/%d\n',sum(trans.pass),height(trans));
fprintf(fid,'external_recovery_controller=false\nheight_outer_PI=false\none_mpc_object=true\n');
end
function o=local_options(varargin)
o.ProjectRoot="";o.SourceBankMat="matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat";
o.OutputRoot="matlab/results/mpc_physics_v1/unified_robust_mpc_v2";
o.OutputBankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";
o.ReferenceAltitudeM=200;o.ReferenceMassKg=3423;o.CargoMassKg=300;o.Ts=0.1;
o.PredictionHorizon=[];o.ControlHorizon=[];
% v2.0.5 default is a zero-persistence estimator ablation.  Set to
% 'integrators' to reproduce the v2.0.4 two-integrator baseline.
o.InputDisturbancePolicy="static_white";
% Physical normalization: H/Va/vz/q scales come directly from the fixed
% certification tolerances. Pitch is NOT a user command, so it receives a
% wider 6-deg natural-trim deviation allowance. Equal normalized output
% weights then avoid hidden cfg-specific priorities.
o.OutputScales=[3.0 1.5 6.0 0.50 0.50];
o.OutputWeights=[1.0 1.0 1.0 1.0 1.0];
% Small common numerical move regularization only; physical authority is
% defined by constraints/rates, not by cfg-specific tuning.
o.MVWeights=[0.0 0.0];o.MVRateWeights=[0.05 0.05];o.ECRWeight=1e5;
% Exact software/plant actuator envelopes already present in the project.
o.PhysicalElevatorMin=-0.95;o.PhysicalElevatorMax=0.95;o.ExternalElevatorDeltaLimit=0.85;
o.ThrottleMin=0.0;o.ThrottleMax=1.0;
% Preserve physical absolute authority, but constrain one-step moves to the
% perturbation-rate envelope already certified with the source physics bank.
% These rate constraints are SOFT (ECR>0), so they are a model-trust prior,
% not an actuator/recovery cap.  No cfg-specific value is allowed.
o.ApplyModelValidityRateLimits=true;o.ModelValidityRateECR=1.0;
% Legacy values remain only as an explicit override for reproducibility.
o.ApplyLegacyArtificialRateLimits=false;o.LegacyElevatorRatePerSample=0.012;o.LegacyThrottleRatePerSample=0.020;
o.LinearTransitionSimS=30;o.PreflightSpeedStepMps=1.0;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=char(string(varargin{i}));if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
