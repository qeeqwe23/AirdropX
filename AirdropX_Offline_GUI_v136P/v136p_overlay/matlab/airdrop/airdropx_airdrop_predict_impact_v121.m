function out=airdropx_airdrop_predict_impact_v121(release,opts)
%AIRDROPX_AIRDROP_PREDICT_IMPACT_V121 Wind-aware longitudinal impact predictor.
% release.wind_est_mps and wind_rate_est_mps2 are estimated quantities only.
% This predictor deliberately has no true-wind input.
arguments
    release (1,1) struct
    opts.Params (1,1) struct = airdropx_airdrop_ballistic_params_v121()
end
req=["x_m","h_m","vx_ground_mps","vz_up_mps","wind_est_mps","wind_rate_est_mps2"];
for i=1:numel(req)
    if ~isfield(release,req(i)), error("AirdropX:Airdrop:MissingPredictorInput","Missing release.%s",req(i)); end
end
p=opts.Params;
wind0=double(release.wind_est_mps);
rate=min(max(double(release.wind_rate_est_mps2),-p.max_abs_forecast_wind_rate_mps2),p.max_abs_forecast_wind_rate_mps2);
windFun=@(tau) min(max(wind0+rate*tau,-p.max_abs_forecast_wind_mps),p.max_abs_forecast_wind_mps);
out=localIntegrate(double(release.x_m),double(release.h_m),double(release.vx_ground_mps),double(release.vz_up_mps),windFun,p);
out.wind0_mps=wind0; out.wind_rate_mps2=rate; out.mode="estimated_linear_wind_forecast";
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
