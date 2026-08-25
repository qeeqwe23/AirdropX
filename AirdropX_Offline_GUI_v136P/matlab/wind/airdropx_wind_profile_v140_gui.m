function w=airdropx_wind_profile_v140_gui(t,opts)
%AIRDROPX_WIND_PROFILE_V140_GUI Compatibility wrapper for the generic GUI wind profile.
arguments
    t double
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
end
w=airdropx_wind_profile_gui(t,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s);
end
