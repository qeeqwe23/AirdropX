function cache=airdropx_phys_mpc_preview_precompute(models,t,dropTimes,opts)
%AIRDROPX_PHYS_MPC_PREVIEW_PRECOMPUTE Prebuild every unique known schedule before flight.
arguments
    models (5,1) cell
    t (:,1) double
    dropTimes (1,4) double
    opts.EnableQSoft (1,1) logical = false
    opts.QSoftPenaltyMultiplier (1,1) double {mustBePositive} = 1e4
end
N=models{1}.ctrl.N; Ts=models{1}.vertex.p.Ts;
keys=strings(numel(t),1); seqs=cell(numel(t),1);
for k=1:numel(t)
    seq=airdropx_phys_mpc_preview_cfg_sequence(t(k),Ts,N,dropTimes);
    seqs{k}=seq;
    keys(k)=localKey(seq);
end
[uKeys,ia]=unique(keys,'stable');
map=containers.Map('KeyType','char','ValueType','any');
buildTimes=zeros(numel(uKeys),1);
for i=1:numel(uKeys)
    seq=seqs{ia(i)};
    tic0=tic;
    c=airdropx_phys_mpc_preview_condense(models,seq,EnableQSoft=opts.EnableQSoft,QSoftPenaltyMultiplier=opts.QSoftPenaltyMultiplier);
    buildTimes(i)=toc(tic0);
    map(char(uKeys(i)))=c;
end
cache=struct("map",map,"step_keys",keys,"unique_keys",uKeys,"unique_count",numel(uKeys), ...
    "build_time_s",sum(buildTimes),"build_time_max_s",max(buildTimes),"enable_q_soft",opts.EnableQSoft);
end

function k=localKey(seq)
k=join(string(seq),""); k=k(1);
end
