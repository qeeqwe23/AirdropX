function [Q,R,meta] = airdropx_phys_bryson_qr(opts)
%AIRDROPX_PHYS_BRYSON_QR One dimensionless preference set for every cfg/H/V.
% Engine spool states use a fixed tiny numerical regularization, not a tuning
% preference, so DARE remains well conditioned without creating cfg-specific Q.
arguments
    opts.StateScale (7,1) double = [4;2;deg2rad(3);deg2rad(5);deg2rad(2);10;10]
    opts.InputScale (2,1) double = [0.15;0.20]
    opts.StatePreference (7,1) double = [1;0.8;0.7;0.35;0.65;0;0]
    opts.InputPreference (2,1) double = [1;0.6]
    opts.EngineRegularization (1,1) double = 1e-8
end
assert(all(opts.StateScale>0) && all(opts.InputScale>0));
assert(all(opts.StatePreference>=0) && all(opts.InputPreference>0));
assert(opts.EngineRegularization>0 && opts.EngineRegularization<1e-3);
pref=opts.StatePreference;
pref(6:7)=pref(6:7)+opts.EngineRegularization;
Sx=diag(1./opts.StateScale); Su=diag(1./opts.InputScale);
Q=Sx'*diag(pref)*Sx;
R=Su'*diag(opts.InputPreference)*Su;
meta=opts;
meta.effective_state_preference=pref;
end
