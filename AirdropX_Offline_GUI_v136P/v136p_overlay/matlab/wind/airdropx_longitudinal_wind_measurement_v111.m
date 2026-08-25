function m=airdropx_longitudinal_wind_measurement_v111(Va_mps,Vz_mps,Vg_mps,opts)
%AIRDROPX_LONGITUDINAL_WIND_MEASUREMENT_V111 Geometry-only longitudinal wind observation.
% Tailwind is positive. Headwind is negative.
arguments
    Va_mps (1,1) double
    Vz_mps (1,1) double
    Vg_mps (1,1) double
    opts.SigmaGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.MinHorizontalAirspeed_mps (1,1) double {mustBePositive} = 5.0
end
m=struct("valid",false,"raw_wind_mps",NaN,"horizontal_airspeed_mps",NaN,"measurement_variance",NaN);
if ~all(isfinite([Va_mps,Vz_mps,Vg_mps])) || Va_mps<=0 || Vg_mps<0 || abs(Vz_mps)>=Va_mps
    return
end
vh2=Va_mps^2-Vz_mps^2;
if vh2<=opts.MinHorizontalAirspeed_mps^2
    return
end
Vah=sqrt(vh2);
z=Vg_mps-Vah;
dVah_dVa=Va_mps/Vah;
dVah_dVz=-Vz_mps/Vah;
R=opts.SigmaGroundspeed_mps^2 + ...
  (dVah_dVa*opts.SigmaAirspeed_mps)^2 + ...
  (dVah_dVz*opts.SigmaVerticalSpeed_mps)^2;
m.valid=true;
m.raw_wind_mps=z;
m.horizontal_airspeed_mps=Vah;
m.measurement_variance=max(R,1e-12);
end
