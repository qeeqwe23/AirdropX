function cfgSeq=airdropx_phys_mpc_preview_cfg_sequence(t0,Ts,N,dropTimes)
%AIRDROPX_PHYS_MPC_PREVIEW_CFG_SEQUENCE Known payload cfg over one MPC horizon.
arguments
    t0 (1,1) double {mustBeFinite}
    Ts (1,1) double {mustBePositive}
    N (1,1) double {mustBeInteger,mustBePositive}
    dropTimes (1,4) double
end
if any(~isfinite(dropTimes)) || any(diff(dropTimes)<0)
    error("AirdropX:PhysMPC:BadDropSchedule","dropTimes must be finite and nondecreasing.");
end
% N control stages at t0..t0+(N-1)Ts plus the terminal state at t0+N Ts.
times=t0+(0:N)*Ts;
cfgSeq=zeros(1,N+1);
for j=1:N+1
    cfgSeq(j)=min(4,sum(times(j)+1e-10>=dropTimes));
end
cfgSeq=double(cfgSeq);
end
