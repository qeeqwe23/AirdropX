function C=airdropx_wind_confidence_v132(windEst,windRate,windSigma,opts)
%AIRDROPX_WIND_CONFIDENCE_V132 Smooth significance gate for estimated wind.
%
% This is deliberately scenario-agnostic.  It uses only estimated wind,
% estimated wind-rate and the estimator's own uncertainty.  The purpose is to
% make the v1.3.x wind channels asymptotically disappear when the evidence for
% wind is comparable to sensor noise, while leaving 5/12 m/s winds essentially
% unchanged.  No truth wind or scenario name is accepted.
arguments
    windEst (1,1) double {mustBeFinite}
    windRate (1,1) double {mustBeFinite}
    windSigma (1,1) double {mustBeNonnegative,mustBeFinite}
    opts.SigmaFloor_mps (1,1) double {mustBePositive} = 0.08
    opts.WindSNRHalf (1,1) double {mustBePositive} = 2.5
    opts.WindAbsHalf_mps (1,1) double {mustBePositive} = 0.35
    opts.RateHalf_mps2 (1,1) double {mustBePositive} = 0.30
    opts.RateSigmaReference_mps (1,1) double {mustBePositive} = 0.60
    opts.Power (1,1) double {mustBePositive} = 4
end
sigma=max(windSigma,opts.SigmaFloor_mps);
z=abs(windEst)/sigma;
p=opts.Power;
snrConf=(z^p)/(z^p+opts.WindSNRHalf^p);
a=abs(windEst); absConf=(a^p)/(a^p+opts.WindAbsHalf_mps^p);
windConf=snrConf*absConf;
rateMag=abs(windRate);
rateSignal=(rateMag^p)/(rateMag^p+opts.RateHalf_mps2^p);
rateUncertainty=1/(1+(windSigma/opts.RateSigmaReference_mps)^2);
rateConf=rateSignal*rateUncertainty;
windConf=max(0,min(1,windConf)); rateConf=max(0,min(1,rateConf));
C=struct();
C.version="AirdropX v1.3.2 wind significance gate";
C.wind_confidence=windConf;
C.rate_confidence=rateConf;
C.wind_effective_mps=windConf*windEst;
C.wind_rate_effective_mps2=rateConf*windRate;
C.wind_snr=z;
C.wind_sigma_mps=windSigma;
end
