function [Q,R,meta] = airdropx_phys_bryson_qr(opts)
%AIRDROPX_PHYS_BRYSON_QR One dimensionless preference set for every cfg/H/V.
% Existing AirdropX certification data motivate h-scale=4 m (historical h-max gate)
% and a conservative 2 m/s Va scale. Engine spool states are prediction states,
% not tracking objectives, so their default state weights are zero.
arguments
    opts.StateScale (7,1) double = [4;2;deg2rad(3);deg2rad(5);deg2rad(2);10;10]
    opts.InputScale (2,1) double = [0.15;0.20]
    opts.StatePreference (7,1) double = [1;0.8;0.7;0.35;0.65;0;0]
    opts.InputPreference (2,1) double = [1;0.6]
end
assert(all(opts.StateScale>0) && all(opts.InputScale>0));
Sx=diag(1./opts.StateScale); Su=diag(1./opts.InputScale);
Q=Sx'*diag(opts.StatePreference)*Sx;
R=Su'*diag(opts.InputPreference)*Su;
meta=opts;
end
