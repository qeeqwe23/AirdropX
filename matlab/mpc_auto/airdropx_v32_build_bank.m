function bank = airdropx_v32_build_bank(varargin)
%AIRDROPX_V32_BUILD_BANK Build one continuously scheduled MPC bank from clean v32 ID nodes.
opts=local_options(varargin{:});
files=string(opts.IdentifiedMats(:)); speeds=double(opts.SpeedNodesMps(:));
if numel(files)~=numel(speeds), error('IdentifiedMats and SpeedNodesMps must have same length.'); end
nodes=repmat(struct('speed_mps',NaN,'controllers',{{}},'trim_bank',[],'plant_bank',{{}},'mpc_meta',struct()),numel(files),1);
for n=1:numel(files)
    S=load(files(n)); if ~isfield(S,'result'),error('Clean ID mat missing result: %s',files(n));end
    R=S.result; ctrls=cell(5,1);
    for k=1:5
        if isempty(R.plant_bank{k}), continue; end
        Pfull=ss(R.plant_bank{k}); if Pfull.Ts==0,Pfull=c2d(Pfull,double(opts.Ts));end
        % v32 removes altitude from the inner MPC entirely. Identified output
        % order is [h, Va, pitch, vz, q], so the aerodynamic inner plant is 2:5.
        P=Pfull(2:5,:);
        C=mpc(P,double(opts.Ts),round(opts.Np),round(opts.Nc));
        C.Model.Nominal.U=zeros(2,1); C.Model.Nominal.Y=zeros(4,1);
        C.MV(1).Min=-opts.ElevatorDeviationLimit; C.MV(1).Max=opts.ElevatorDeviationLimit;
        C.MV(2).Min=-opts.ThrottleDeviationLimit; C.MV(2).Max=opts.ThrottleDeviationLimit;
        C.MV(1).RateMin=-opts.ElevatorDeviationRateLimit; C.MV(1).RateMax=opts.ElevatorDeviationRateLimit;
        C.MV(2).RateMin=-opts.ThrottleDeviationRateLimit; C.MV(2).RateMax=opts.ThrottleDeviationRateLimit;
        % Direct altitude weight is always zero in v32. Inner MPC learns Va/vz dynamics.
        C.Weights.OutputVariables=[opts.Wva opts.Wpitch opts.Wvz opts.Wq];
        C.Weights.ManipulatedVariables=[opts.WmvElev opts.WmvThrottle];
        C.Weights.ManipulatedVariablesRate=[opts.WrateElev opts.WrateThrottle];
        sf=[opts.ScaleVa opts.ScalePitch opts.ScaleVz opts.ScaleQ];
        for j=1:4,C.OV(j).ScaleFactor=sf(j);end
        C.MV(1).ScaleFactor=opts.ElevatorDeviationLimit; C.MV(2).ScaleFactor=opts.ThrottleDeviationLimit;
        try, setoutdist(C,'model',tf(zeros(4,1))); catch, end
        ctrls{k}=C;
    end
    physical=local_physical_nominals(R);
    meta=struct('version',32,'architecture','v32_clean_inner_va_vz','input_coordinate_mode','deviation_physical',...
        'physical_elevator_nominals',physical(:),...
        'throttle_nominals',arrayfun(@(x)double(x.throttle_cmd),R.trim_bank(:)),...
        'elevator_deviation_limit',double(opts.ElevatorDeviationLimit),'throttle_deviation_limit',double(opts.ThrottleDeviationLimit),...
        'elevator_deviation_rate_limit',double(opts.ElevatorDeviationRateLimit),'throttle_deviation_rate_limit',double(opts.ThrottleDeviationRateLimit));
    nodes(n)=struct('speed_mps',speeds(n),'controllers',{ctrls},'trim_bank',R.trim_bank(:),'plant_bank',{R.plant_bank(:)},'mpc_meta',meta);
end
[speeds,ord]=sort(speeds); nodes=nodes(ord);
v32_nodes=nodes; speed_nodes=speeds; trim_bank=nodes(find(abs(speeds-median(speeds))==min(abs(speeds-median(speeds))),1)).trim_bank; %#ok<NASGU>
mpc_meta=nodes(1).mpc_meta; %#ok<NASGU>
bank=struct('v32_nodes',v32_nodes,'speed_nodes',speed_nodes,'trim_bank',trim_bank,'mpc_meta',mpc_meta,'options',opts);
if strlength(string(opts.OutputMat))>0
    outdir=fileparts(string(opts.OutputMat));if strlength(outdir)>0&&~isfolder(outdir),mkdir(outdir);end
    save(opts.OutputMat,'v32_nodes','speed_nodes','trim_bank','mpc_meta','-v7.3');
end
end

function p=local_physical_nominals(R)
p=NaN(5,1);
for cfg=0:4
    try
        if isfield(R.trim_bank(cfg+1),'physical_elevator_cmd')
            x=double(R.trim_bank(cfg+1).physical_elevator_cmd);
            if isfinite(x),p(cfg+1)=x;continue;end
        end
    catch
    end
    vals=[];
    try
        files=string(R.data.csv_files(:));
        for i=1:numel(files)
            if ~isfile(files(i)),continue;end
            T=readtable(files(i));
            if ~ismember('config_id',string(T.Properties.VariableNames))||~ismember('elevator_cmd_norm',string(T.Properties.VariableNames)),continue;end
            if round(median(double(T.config_id),'omitnan'))~=cfg,continue;end
            m=true(height(T),1);
            if ismember('elevator_excitation',string(T.Properties.VariableNames))
                ex=abs(double(T.elevator_excitation));j=find(ex>1e-7,1,'first');if ~isempty(j)&&j>4,m=false(height(T),1);m(1:j-1)=true;end
            end
            v=double(T.elevator_cmd_norm(m));v=v(isfinite(v));if ~isempty(v),vals(end+1,1)=median(v,'omitnan');end %#ok<AGROW>
        end
    catch
    end
    if ~isempty(vals),p(cfg+1)=median(vals,'omitnan');else,p(cfg+1)=double(R.trim_bank(cfg+1).elevator_cmd);end
end
end
function opts=local_options(varargin)
opts.IdentifiedMats=strings(0,1); opts.SpeedNodesMps=[]; opts.OutputMat=""; opts.Ts=0.1;
opts.Np=24;opts.Nc=5;opts.Wva=8;opts.Wpitch=0.10;opts.Wvz=35;opts.Wq=2.0;
opts.WmvElev=0.10;opts.WmvThrottle=0.10;opts.WrateElev=2.0;opts.WrateThrottle=1.5;
opts.ScaleVa=5;opts.ScalePitch=5;opts.ScaleVz=1;opts.ScaleQ=5;
opts.ElevatorDeviationLimit=0.08;opts.ThrottleDeviationLimit=0.12;
opts.ElevatorDeviationRateLimit=0.012;opts.ThrottleDeviationRateLimit=0.020;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
