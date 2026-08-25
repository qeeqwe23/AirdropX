function s=airdropx_paper_sensor_init_v136p(Ts,opts)
%AIRDROPX_PAPER_SENSOR_INIT_V136P Paper-validation sensor/state interface.
% Unbiased, white-noise sensor model: enough realism to avoid simulator-truth
% feedback, without turning a controller paper into an avionics-bias study.
arguments
    Ts (1,1) double {mustBePositive}
    opts.SigmaGnssPos_m (1,1) double {mustBeNonnegative} = 0.30
    opts.SigmaGnssGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.SigmaGnssVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.07
    opts.SigmaBaroAltitude_m (1,1) double {mustBeNonnegative} = 0.20
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaPitch_rad (1,1) double {mustBeNonnegative} = deg2rad(0.03)
    opts.SigmaPitchRate_radps (1,1) double {mustBeNonnegative} = deg2rad(0.01)
    opts.SigmaN1 (1,1) double {mustBeNonnegative} = 0.05
    opts.SigmaN2 (1,1) double {mustBeNonnegative} = 0.05
    opts.PositionFilterTau_s (1,1) double {mustBePositive} = 0.15
    opts.VelocityFilterTau_s (1,1) double {mustBePositive} = 0.12
    opts.AltitudeFilterTau_s (1,1) double {mustBePositive} = 0.15
    opts.AirspeedFilterTau_s (1,1) double {mustBePositive} = 0.10
    opts.AttitudeFilterTau_s (1,1) double {mustBePositive} = 0.08
    opts.EngineFilterTau_s (1,1) double {mustBePositive} = 0.10
end
s=struct();
s.version="AirdropX paper sensor interface v1.3.6-Paper";
s.Ts=Ts; s.opts=opts; s.initialized=false;
s.est=struct("pos_n_m",NaN,"h_m",NaN,"Va_mps",NaN,"gamma_rad",NaN, ...
    "theta_rad",NaN,"q_radps",NaN,"N1",NaN,"N2",NaN,"Vg_long_mps",NaN,"Vz_up_mps",NaN);
s.sample_count=0;
end
