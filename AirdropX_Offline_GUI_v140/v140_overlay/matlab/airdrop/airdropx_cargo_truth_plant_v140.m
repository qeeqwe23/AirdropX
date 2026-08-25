function out=airdropx_cargo_truth_plant_v140(scenarioName,release,opts)
%AIRDROPX_CARGO_TRUTH_PLANT_V140 Independent scoring-only cargo plant.
% Unlike the release predictor this plant contains vertical drag and density
% variation.  It is never used by the onboard release decision.
arguments
    scenarioName (1,1) string
    release (1,1) struct
    opts.Params (1,1) struct = airdropx_cargo_truth_params_v140()
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
    opts.CdAScale (1,1) double {mustBePositive} = 1.0
end
req=["x_m","h_m","vx_ground_mps","vz_up_mps","t_s"];
for i=1:numel(req), if ~isfield(release,req(i)), error("AirdropX:CargoTruth:MissingInput","Missing release.%s",req(i)); end, end
p=opts.Params; CdA=p.CdA_m2*opts.CdAScale; dt=p.integration_dt_s;
x=double(release.x_m); x0=x; h=double(release.h_m); vx=double(release.vx_ground_mps); vz=double(release.vz_up_mps); tt=0;
for k=1:ceil(p.max_fall_time_s/dt)
    wind=double(airdropx_wind_profile_v136(scenarioName,double(release.t_s)+tt,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
    rho=p.rho0_kgpm3*exp(-max(h,0)/p.density_scale_height_m); vrx=vx-wind; vrz=vz; sp=hypot(vrx,vrz); kd=0.5*rho*CdA/p.mass_kg;
    ax=-kd*sp*vrx; az=-p.g_mps2-kd*sp*vrz;
    vx2=vx+ax*dt; vz2=vz+az*dt; x2=x+0.5*(vx+vx2)*dt; h2=h+0.5*(vz+vz2)*dt;
    if h2<=0
        f=h/(h-h2); impact=x+f*(x2-x); fall=tt+f*dt;
        out=struct("impact_x_m",impact,"fall_time_s",fall,"range_m",impact-x0,"steps",k,"mode","independent_2d_drag_truth_v140","CdA_m2",CdA); return
    end
    x=x2; h=h2; vx=vx2; vz=vz2; tt=tt+dt;
end
error("AirdropX:CargoTruth:DidNotImpact","Independent cargo plant did not reach ground within %.3g s.",p.max_fall_time_s);
end
