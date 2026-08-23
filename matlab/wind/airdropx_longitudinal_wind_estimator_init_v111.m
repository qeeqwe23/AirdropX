function s=airdropx_longitudinal_wind_estimator_init_v111(Ts,opts)
%AIRDROPX_LONGITUDINAL_WIND_ESTIMATOR_INIT_V111 Two-state adaptive Kalman wind observer.
arguments
    Ts (1,1) double {mustBePositive}
    opts.SigmaGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.WindAccelerationSigma_mps2 (1,1) double {mustBePositive} = 1.00
    opts.InitialWindSigma_mps (1,1) double {mustBePositive} = 4.0
    opts.InitialWindRateSigma_mps2 (1,1) double {mustBePositive} = 2.0
    opts.StepNisThreshold (1,1) double {mustBePositive} = 16.0
    opts.StepImmediateNisThreshold (1,1) double {mustBePositive} = 64.0
    opts.StepConsecutiveSamples (1,1) double {mustBeInteger,mustBePositive} = 2
    opts.StepResetRateSigma_mps2 (1,1) double {mustBePositive} = 2.0
    opts.MinHorizontalAirspeed_mps (1,1) double {mustBePositive} = 5.0
end
s=struct();
s.version="AirdropX longitudinal wind estimator v1.1.1";
s.Ts=Ts;
s.x=[0;0]; % [wind_mps; wind_rate_mps2]
s.P=diag([opts.InitialWindSigma_mps^2,opts.InitialWindRateSigma_mps2^2]);
s.F=[1 Ts;0 1];
q=opts.WindAccelerationSigma_mps2^2;
s.Q=q*[Ts^3/3 Ts^2/2;Ts^2/2 Ts];
s.H=[1 0];
s.opts=opts;
s.high_nis_count=0;
s.sample_count=0;
s.valid_measurement_count=0;
end
