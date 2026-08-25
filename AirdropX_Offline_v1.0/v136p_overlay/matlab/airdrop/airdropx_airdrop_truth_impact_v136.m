function out=airdropx_airdrop_truth_impact_v136(scenarioName,release,opts)
%AIRDROPX_AIRDROP_TRUTH_IMPACT_V136 Scoring-only cargo truth using the v1.3.6 validation wind profile.
arguments
    scenarioName (1,1) string
    release (1,1) struct
    opts.Params (1,1) struct = airdropx_airdrop_ballistic_params_v121()
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
end
p=opts.Params;
req=["x_m","h_m","vx_ground_mps","vz_up_mps","t_s"];
for i=1:numel(req)
    if ~isfield(release,req(i)), error("AirdropX:Airdrop:MissingTruthInput","Missing release.%s",req(i)); end
end
windFun=@(tau) double(airdropx_wind_profile_v136(scenarioName,double(release.t_s)+tau,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
out=localIntegrate(double(release.x_m),double(release.h_m),double(release.vx_ground_mps),double(release.vz_up_mps),windFun,p);
out.mode="scoring_only_true_wind_profile_v136";
end

function out=localIntegrate(x,h,vx,vz,windFun,p)
t=0; dt=p.integration_dt_s; x0=x;
if h<=0, out=struct("impact_x_m",x,"fall_time_s",0,"range_m",0,"steps",0); return; end
for k=1:ceil(p.max_fall_time_s/dt)
    w=windFun(t);
    vrel=vx-w;
    ax=-p.drag_per_m*vrel*abs(vrel);
    vz2=vz-p.g_mps2*dt;
    vx2=vx+ax*dt;
    h2=h+0.5*(vz+vz2)*dt;
    x2=x+0.5*(vx+vx2)*dt;
    if h2<=0
        frac=h/(h-h2);
        xImpact=x+frac*(x2-x);
        tImpact=t+frac*dt;
        out=struct("impact_x_m",xImpact,"fall_time_s",tImpact,"range_m",xImpact-x0,"steps",k);
        return
    end
    x=x2; h=h2; vx=vx2; vz=vz2; t=t+dt;
end
error("AirdropX:Airdrop:CargoDidNotImpact","Cargo did not reach ground within %.3g s.",p.max_fall_time_s);
end
