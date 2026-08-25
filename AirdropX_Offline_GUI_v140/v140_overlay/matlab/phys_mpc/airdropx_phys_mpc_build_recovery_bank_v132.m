function sched=airdropx_phys_mpc_build_recovery_bank_v132(sched,opts)
%AIRDROPX_PHYS_MPC_BUILD_RECOVERY_BANK_V132 Unified transient gust-recovery cost bank.
%
% This is not cfg-by-cfg tuning: every payload configuration uses the same
% severity levels and the same dimensionless Q/R multipliers.  Only A/B/trim/Gw
% remain scheduled from physics as before.
arguments
    sched (1,1) struct
    opts.Levels (1,:) double {mustBeFinite,mustBeNonnegative} = [0 0.25 0.5 0.75 1]
    opts.MaxQMultiplier (:,1) double {mustBeFinite,mustBePositive} = [1.5;4.0;1.5;2.0;3.0;1.0;1.0]
    opts.MaxRMultiplier (:,1) double {mustBeFinite,mustBePositive} = [0.80;0.35]
end
levels=unique(double(opts.Levels(:).')); if levels(1)~=0 || levels(end)~=1, error("AirdropX:WindMPC:BadRecoveryLevels","Recovery levels must include 0 and 1."); end
if numel(opts.MaxQMultiplier)~=7 || numel(opts.MaxRMultiplier)~=2, error("AirdropX:WindMPC:BadRecoveryWeights","Expected 7 state and 2 input multipliers."); end
for cfg=0:4
    base=sched.models{cfg+1}.ctrl; bank=cell(numel(levels),1);
    for j=1:numel(levels)
        s=levels(j);
        q=1+s*(double(opts.MaxQMultiplier(:))-1);
        r=1+s*(double(opts.MaxRMultiplier(:))-1);
        bank{j}=airdropx_phys_mpc_reweight_v132(base,q,r);
        bank{j}.recovery_level=s;
    end
    sched.models{cfg+1}.recovery_ctrl=bank;
end
sched.recovery=struct("version","AirdropX v1.3.2 unified recovery bank","levels",levels, ...
    "max_q_multiplier",double(opts.MaxQMultiplier(:)),"max_r_multiplier",double(opts.MaxRMultiplier(:)));
end
