function [H,V,Hdot,Vdot,meta]=airdropx_phys_runtime_command_profile_v101(name,t)
%AIRDROPX_PHYS_RUNTIME_COMMAND_PROFILE_V101 Smooth commands with analytic first derivatives.
arguments
    name (1,1) string
    t double
end
t=double(t); T0=10; T1=130; [s,sdot]=localSmooth(t,T0,T1);
switch lower(name)
    case "altitude_down_v45"
        H=200-180*s; Hdot=-180*sdot; V=45+zeros(size(t)); Vdot=zeros(size(t));
    case "altitude_up_v65"
        H=20+180*s; Hdot=180*sdot; V=65+zeros(size(t)); Vdot=zeros(size(t));
    case "speed_up_h20"
        H=20+zeros(size(t)); Hdot=zeros(size(t)); V=45+20*s; Vdot=20*sdot;
    case "speed_down_h200"
        H=200+zeros(size(t)); Hdot=zeros(size(t)); V=65-20*s; Vdot=-20*sdot;
    case "coupled_low_to_high"
        H=20+180*s; Hdot=180*sdot; V=45+20*s; Vdot=20*sdot;
    case "coupled_high_to_low"
        H=200-180*s; Hdot=-180*sdot; V=65-20*s; Vdot=-20*sdot;
    otherwise
        error("AirdropX:PhysMPC:UnknownRuntimeScenario","Unknown runtime scenario: %s",name);
end
if any(H<20-1e-10 | H>200+1e-10,'all') || any(V<45-1e-10 | V>65+1e-10,'all')
    error("AirdropX:PhysMPC:RuntimeProfileBounds","Generated profile escaped interval.");
end
meta=struct("name",name,"transition_start_s",T0,"transition_end_s",T1, ...
    "H_min",min(H,[],'all'),"H_max",max(H,[],'all'),"V_min",min(V,[],'all'),"V_max",max(V,[],'all'), ...
    "Hdot_max_abs",max(abs(Hdot),[],'all'),"Vdot_max_abs",max(abs(Vdot),[],'all'));
end

function [s,sdot]=localSmooth(t,t0,t1)
z=(t-t0)/(t1-t0); inside=(z>0 & z<1); zc=min(max(z,0),1);
s=3*zc.^2-2*zc.^3; sdot=zeros(size(t));
sdot(inside)=(6*z(inside)-6*z(inside).^2)/(t1-t0);
end
