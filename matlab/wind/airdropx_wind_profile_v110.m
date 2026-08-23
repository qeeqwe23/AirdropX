function w=airdropx_wind_profile_v110(name,t)
%AIRDROPX_WIND_PROFILE_V110 Signed along-flight wind command. Tailwind positive.
name=lower(string(name)); t=double(t);
switch name
    case "calm"
        w=zeros(size(t));
    case "tailwind_5"
        w=5*double(t>=5);
    case "headwind_5"
        w=-5*double(t>=5);
    case "tailwind_12"
        w=12*double(t>=5);
    case "headwind_12"
        w=-12*double(t>=5);
    case "step_bidirectional"
        w=zeros(size(t)); w(t>=5)=8; w(t>=15)=-8; w(t>=25)=3;
    case "ramp_minus10_plus10"
        w=zeros(size(t));
        a=t>=5 & t<25; w(a)=-10+(t(a)-5);
        w(t>=25)=10;
    case "sine_longitudinal"
        w=zeros(size(t)); a=t>=5; w(a)=2+6*sin(2*pi*(t(a)-5)/20);
    otherwise
        error("AirdropX:Wind:UnknownProfile","Unknown wind profile: %s",name);
end
end
