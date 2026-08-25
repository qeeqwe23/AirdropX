function w=airdropx_wind_profile_v136(name,t,opts)
%AIRDROPX_WIND_PROFILE_V136 v1.1.1 profiles plus a causal settling tail for sustained sine validation.
arguments
    name (1,1) string
    t double
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
end
name=lower(string(name)); t=double(t);
w=airdropx_wind_profile_v111(name,t);
if name~="sine_longitudinal", return; end

% The original sine profile is intentionally retained during the forcing
% window.  It is then brought continuously to zero so final/tail metrics are
% true post-forcing recovery metrics instead of measurements under a wind that
% is still being applied.  A half-cosine taper avoids creating a fake gust step.
t0=opts.SineForcingEnd_s; tr=opts.SineSettleRamp_s;
w0=double(airdropx_wind_profile_v111(name,t0));
a=t>=t0 & t<t0+tr;
if any(a,"all")
    z=(t(a)-t0)/tr;
    w(a)=w0.*0.5.*(1+cos(pi*z));
end
w(t>=t0+tr)=0;
end
