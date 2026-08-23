function p=airdropx_cargo_truth_params_v140()
%AIRDROPX_CARGO_TRUTH_PARAMS_V140 Independent 2-D cargo plant parameters.
% This is deliberately NOT the guidance predictor model.  It uses vector
% quadratic drag, altitude-varying density and vertical drag.  CdA is fitted
% only to the historical 20 m / 78.6 m/s zero-wind drop datum, preserving the
% one empirical calibration point while changing the model structure.
persistent P
if ~isempty(P), p=P; return; end
p=struct();
p.version="AirdropX independent cargo truth plant v1.4.0";
p.g_mps2=9.80665; p.mass_kg=300.0; p.rho0_kgpm3=1.225; p.density_scale_height_m=8500;
p.integration_dt_s=0.005; p.max_fall_time_s=35;
p.calibration_H_m=20.0; p.calibration_Vx_mps=78.6; p.calibration_range_m=150.7649;
lo=0.01; hi=2.0;
for k=1:70
    mid=0.5*(lo+hi); r=localRange(mid,p);
    if r>p.calibration_range_m, lo=mid; else, hi=mid; end
end
p.CdA_m2=0.5*(lo+hi); p.calibration_reconstructed_m=localRange(p.CdA_m2,p); p.calibration_error_m=p.calibration_reconstructed_m-p.calibration_range_m;
P=p;
end

function range=localRange(CdA,p)
x=0; h=p.calibration_H_m; vx=p.calibration_Vx_mps; vz=0; dt=p.integration_dt_s;
for k=1:ceil(p.max_fall_time_s/dt)
    [ax,az]=localAccel(h,vx,vz,0,CdA,p);
    vx2=vx+ax*dt; vz2=vz+az*dt; x2=x+0.5*(vx+vx2)*dt; h2=h+0.5*(vz+vz2)*dt;
    if h2<=0
        f=h/(h-h2); range=x+f*(x2-x); return
    end
    x=x2; h=h2; vx=vx2; vz=vz2;
end
error("AirdropX:CargoTruth:CalibrationDidNotImpact","Independent cargo calibration did not impact.");
end

function [ax,az]=localAccel(h,vx,vz,wind,CdA,p)
rho=p.rho0_kgpm3*exp(-max(h,0)/p.density_scale_height_m);
vrx=vx-wind; vrz=vz; sp=hypot(vrx,vrz); k=0.5*rho*CdA/p.mass_kg;
ax=-k*sp*vrx; az=-p.g_mps2-k*sp*vrz;
end
