function p=airdropx_airdrop_ballistic_params_v121()
%AIRDROPX_AIRDROP_BALLISTIC_PARAMS_V121 Longitudinal cargo ballistic model.
% Uses the calibration already present in AirdropX simulation parameters:
% H=20 m, Va=78.6 m/s, measured JSBSim drop distance=150.7649 m.
% Horizontal drag is represented by dv_rel/dt=-c*v_rel*abs(v_rel).
p=struct();
p.g_mps2=9.80665;
p.calibration_H_m=20.0;
p.calibration_Va_mps=78.6;
p.calibration_range_m=150.7649;
p.integration_dt_s=0.01;
p.max_fall_time_s=30.0;
p.max_abs_forecast_wind_mps=20.0;
p.max_abs_forecast_wind_rate_mps2=3.0;
T=sqrt(2*p.calibration_H_m/p.g_mps2);
c=6.8e-4;
for k=1:12
    z=1+c*p.calibration_Va_mps*T;
    f=log(z)/c-p.calibration_range_m;
    df=(c*(p.calibration_Va_mps*T/z)-log(z))/(c*c);
    c=c-f/df;
end
p.drag_per_m=c;
p.calibration_reconstructed_m=log(1+c*p.calibration_Va_mps*T)/c;
p.calibration_error_m=p.calibration_reconstructed_m-p.calibration_range_m;
end
