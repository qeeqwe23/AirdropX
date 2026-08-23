function [s,o]=airdropx_longitudinal_wind_estimator_step_v110(s,Va_mps,Vz_mps,Vg_mps)
%AIRDROPX_LONGITUDINAL_WIND_ESTIMATOR_STEP_V110 One 10-Hz-capable wind observer update.
if ~isstruct(s) || ~isfield(s,"version") || ~contains(string(s.version),"v1.1.0")
    error("AirdropX:Wind:BadEstimatorState","Initialize with airdropx_longitudinal_wind_estimator_init_v110.");
end
s.sample_count=s.sample_count+1;
s.x=s.F*s.x;
s.P=s.F*s.P*s.F.'+s.Q;
m=airdropx_longitudinal_wind_measurement_v110(Va_mps,Vz_mps,Vg_mps, ...
    SigmaGroundspeed_mps=s.opts.SigmaGroundspeed_mps, ...
    SigmaAirspeed_mps=s.opts.SigmaAirspeed_mps, ...
    SigmaVerticalSpeed_mps=s.opts.SigmaVerticalSpeed_mps, ...
    MinHorizontalAirspeed_mps=s.opts.MinHorizontalAirspeed_mps);
innovation=NaN; nis=NaN; stepDetected=false;
if m.valid
    s.valid_measurement_count=s.valid_measurement_count+1;
    H=s.H; R=m.measurement_variance;
    innovation=m.raw_wind_mps-H*s.x;
    S=H*s.P*H.'+R;
    nis=innovation^2/S;
    immediateStep=nis>s.opts.StepImmediateNisThreshold;
    if nis>s.opts.StepNisThreshold
        s.high_nis_count=s.high_nis_count+1;
    else
        s.high_nis_count=0;
    end
    if immediateStep || s.high_nis_count>=s.opts.StepConsecutiveSamples
        % A persistent 5-sigma-class innovation is treated as a real longitudinal
        % wind jump, not measurement noise. Re-acquire without multi-second lag.
        s.x=[m.raw_wind_mps;0];
        s.P=diag([max(R,0.20^2),s.opts.StepResetRateSigma_mps2^2]);
        s.high_nis_count=0;
        stepDetected=true;
    else
        K=(s.P*H.')/S;
        I=eye(2);
        s.x=s.x+K*innovation;
        % Joseph form protects positive-semidefiniteness.
        s.P=(I-K*H)*s.P*(I-K*H).'+K*R*K.';
        s.P=(s.P+s.P.')/2;
    end
end
o=struct();
o.valid=m.valid;
o.wind_est_mps=s.x(1);
o.wind_rate_est_mps2=s.x(2);
o.wind_sigma_mps=sqrt(max(s.P(1,1),0));
o.raw_wind_mps=m.raw_wind_mps;
o.horizontal_airspeed_mps=m.horizontal_airspeed_mps;
o.innovation_mps=innovation;
o.nis=nis;
o.step_detected=stepDetected;
end
