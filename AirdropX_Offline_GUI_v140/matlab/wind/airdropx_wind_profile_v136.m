function w=airdropx_wind_profile_v136(name,t,opts)
%AIRDROPX_WIND_PROFILE_V136 Formal v1.3.6 profiles + isolated GUI custom extension.
arguments
    name (1,1) string
    t double
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
end
name=lower(string(name)); t=double(t);
% GUI-only branch. All original formal scenario names continue through the
% exact original v1.3.6 code path below.
if name=="gui_custom"
    w=airdropx_wind_profile_v140_gui(t,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s);
    return
end
w=airdropx_wind_profile_v111(name,t);
if name~="sine_longitudinal", return; end

t0=opts.SineForcingEnd_s; tr=opts.SineSettleRamp_s;
w0=double(airdropx_wind_profile_v111(name,t0));
a=t>=t0 & t<t0+tr;
if any(a,"all")
    z=(t(a)-t0)/tr;
    w(a)=w0.*0.5.*(1+cos(pi*z));
end
w(t>=t0+tr)=0;
end
