function s=airdropx_avionics_init_v140(Ts,opts)
%AIRDROPX_AVIONICS_INIT_V140 Sensor-realistic onboard state interface.
% The controller must never consume JSBSim truth directly.  JSBSim plant truth
% is allowed only at the input of a simulated sensor model or in scoring code.
arguments
    Ts (1,1) double {mustBePositive}
    opts.SigmaGnssPos_m (1,1) double {mustBeNonnegative} = 0.60
    opts.SigmaGnssGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaGnssVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.SigmaBaroAltitude_m (1,1) double {mustBeNonnegative} = 0.30
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaPitch_rad (1,1) double {mustBeNonnegative} = deg2rad(0.08)
    opts.SigmaPitchRate_radps (1,1) double {mustBeNonnegative} = deg2rad(0.02)
    opts.SigmaN1 (1,1) double {mustBeNonnegative} = 0.08
    opts.SigmaN2 (1,1) double {mustBeNonnegative} = 0.08
    opts.BaroBiasSigma_m (1,1) double {mustBeNonnegative} = 0.35
    opts.AirspeedBiasSigma_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.PitchBiasSigma_rad (1,1) double {mustBeNonnegative} = deg2rad(0.04)
    opts.PitchRateBiasSigma_radps (1,1) double {mustBeNonnegative} = deg2rad(0.01)
    opts.BaroBiasRw_m_sqrt_s (1,1) double {mustBeNonnegative} = 0.008
    opts.AirspeedBiasRw_mps_sqrt_s (1,1) double {mustBeNonnegative} = 0.004
    opts.PitchBiasRw_rad_sqrt_s (1,1) double {mustBeNonnegative} = deg2rad(0.002)
    opts.PitchRateBiasRw_radps_sqrt_s (1,1) double {mustBeNonnegative} = deg2rad(0.001)
    opts.PositionFilterTau_s (1,1) double {mustBePositive} = 0.30
    opts.VelocityFilterTau_s (1,1) double {mustBePositive} = 0.18
    opts.AltitudeFilterTau_s (1,1) double {mustBePositive} = 0.30
    opts.AirspeedFilterTau_s (1,1) double {mustBePositive} = 0.18
    opts.AttitudeFilterTau_s (1,1) double {mustBePositive} = 0.10
    opts.EngineFilterTau_s (1,1) double {mustBePositive} = 0.15
end
s=struct();
s.version="AirdropX avionics sensor/state interface v1.4.0";
s.Ts=Ts; s.opts=opts; s.initialized=false;
s.bias=struct("baro_m",opts.BaroBiasSigma_m*randn, ...
    "airspeed_mps",opts.AirspeedBiasSigma_mps*randn, ...
    "pitch_rad",opts.PitchBiasSigma_rad*randn, ...
    "q_radps",opts.PitchRateBiasSigma_radps*randn);
s.est=struct("pos_n_m",NaN,"h_m",NaN,"Va_mps",NaN,"gamma_rad",NaN, ...
    "theta_rad",NaN,"q_radps",NaN,"N1",NaN,"N2",NaN,"Vg_long_mps",NaN,"Vz_up_mps",NaN);
s.sample_count=0;
end
