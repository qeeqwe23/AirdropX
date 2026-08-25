function w=airdropx_wind_profile_gui(t,opts)
%AIRDROPX_WIND_PROFILE_GUI Deterministic GUI-only longitudinal wind profile.
% Positive = tailwind, negative = headwind, matching AirdropX v1.3.6-Paper.
arguments
    t double
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
end
if ~isappdata(0,"AirdropXGuiWindConfig")
    error("AirdropX:GUI:WindConfigMissing","GUI custom wind config is not initialized.");
end
c=getappdata(0,"AirdropXGuiWindConfig");
t=double(t); A=double(c.along_track_mps); t0=double(c.start_s);
kind=lower(string(c.kind)); w=zeros(size(t));
switch kind
    case "calm"
        return
    case "constant"
        w(:)=A;
    case "step"
        w(t>=t0)=A;
    case "bidirectional_step"
        w(t>=t0)=A; w(t>=t0+10)=-A; w(t>=t0+20)=0.35*A;
    case "ramp"
        tr=max(double(c.ramp_s),eps); a=t>=t0 & t<t0+tr;
        w(a)=A.*(t(a)-t0)./tr; w(t>=t0+tr)=A;
    case "sine"
        per=max(double(c.period_s),eps); a=t>=t0;
        w(a)=A.*sin(2*pi*(t(a)-t0)./per);
        te=min(double(c.forcing_end_s),opts.SineForcingEnd_s); tr=max(double(c.settle_ramp_s),eps);
        if te>t0
            w0=A*sin(2*pi*(te-t0)/per); b=t>=te & t<te+tr;
            z=(t(b)-te)./tr; w(b)=w0.*0.5.*(1+cos(pi*z)); w(t>=te+tr)=0;
        end
    case "turbulence"
        a=t>=t0; tau=t(a)-t0;
        % Stateless deterministic broadband-like gust so carrier and cargo
        % queries see the same wind at arbitrary times.
        w(a)=A.*(0.55*sin(2*pi*tau/7.0)+0.30*sin(2*pi*tau/3.1+0.8)+0.15*sin(2*pi*tau/1.37+1.7));
    otherwise
        error("AirdropX:GUI:UnknownWindType","Unknown GUI wind type: %s",kind);
end
end
