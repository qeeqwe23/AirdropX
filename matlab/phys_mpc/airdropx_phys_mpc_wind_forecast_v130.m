function F=airdropx_phys_mpc_wind_forecast_v130(windEst,windRate,windSigma,N,Ts,opts)
%AIRDROPX_PHYS_MPC_WIND_FORECAST_V130 Causal short-memory wind-increment preview.
%
% Uniform absolute wind is intentionally NOT injected as a persistent force.
% For an air-relative longitudinal state model, a spatially uniform constant
% wind changes groundspeed but does not create a permanent aerodynamic force.
% The carrier disturbance is driven by wind CHANGE (gust/ramp), so MPC previews
% delta-wind from the estimated wind rate.  An unknown instantaneous step cannot
% be cancelled before it is measured; this predictor only acts causally.
arguments
    windEst (1,1) double {mustBeFinite}
    windRate (1,1) double {mustBeFinite}
    windSigma (1,1) double {mustBeNonnegative,mustBeFinite}
    N (1,1) double {mustBeInteger,mustBePositive}
    Ts (1,1) double {mustBePositive}
    opts.RateCap_mps2 (1,1) double {mustBePositive} = 3.0
    opts.RateMemory_s (1,1) double {mustBePositive} = 2.0
    opts.WindAbsCap_mps (1,1) double {mustBePositive} = 20.0
end
r=max(min(windRate,opts.RateCap_mps2),-opts.RateCap_mps2);
% Reduce aggressive look-ahead only while the wind estimate is uncertain.
confidence=1/(1+(windSigma/1.0)^2);
confidence=max(0.25,min(1,confidence));
r=r*confidence;
rate=zeros(N,1); dw=zeros(N,1); w=zeros(N+1,1); w(1)=windEst;
for i=1:N
    rate(i)=r*exp(-((i-1)*Ts)/opts.RateMemory_s);
    dw(i)=rate(i)*Ts;
    wn=w(i)+dw(i);
    wn=max(min(wn,opts.WindAbsCap_mps),-opts.WindAbsCap_mps);
    dw(i)=wn-w(i); w(i+1)=wn;
end
F=struct("wind0_mps",windEst,"rate0_mps2",windRate,"rate_used0_mps2",r, ...
    "wind_sigma_mps",windSigma,"wind_pred_mps",w,"wind_rate_pred_mps2",rate,"delta_wind_pred_mps",dw);
end
