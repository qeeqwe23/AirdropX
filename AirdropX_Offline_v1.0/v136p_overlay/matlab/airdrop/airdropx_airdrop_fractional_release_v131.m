function S=airdropx_airdrop_fractional_release_v131(releaseState,target_m,Ts,opts)
%AIRDROPX_AIRDROP_FRACTIONAL_RELEASE_V131 Causal sub-sample release scheduler.
%
% The carrier MPC continues to run at the certified 0.1 s sample time.  This
% helper only refines the payload release EVENT inside the current sample so a
% 50 m/s aircraft is no longer quantized to roughly 5 m release-position bins.
%
% No future JSBSim truth is read.  A second impact prediction is formed from a
% short kinematic extrapolation of the CURRENT measured release state.  If the
% target lies between the current and one-sample-ahead predicted impact points,
% a fractional timer tau in [0,Ts] is returned.  The mission then holds the
% current MPC command, advances the nonlinear carrier for tau, removes the
% payload, and advances the remaining Ts-tau with the new mass configuration.
arguments
    releaseState (1,1) struct
    target_m (1,1) double {mustBeFinite}
    Ts (1,1) double {mustBePositive,mustBeFinite}
    opts.Params (1,1) struct = airdropx_airdrop_ballistic_params_v121()
    opts.MinImpactAdvance_m (1,1) double {mustBePositive,mustBeFinite} = 0.25
    opts.AlignToOracleGrid (1,1) logical = true
    opts.OracleDt_s (1,1) double {mustBePositive,mustBeFinite} = 1/120
end
req=["x_m","h_m","vx_ground_mps","vz_up_mps","wind_est_mps","wind_rate_est_mps2"];
for i=1:numel(req)
    if ~isfield(releaseState,req(i))
        error("AirdropX:Airdrop:MissingFractionalReleaseInput","Missing releaseState.%s",req(i));
    end
end
p=opts.Params;
now=airdropx_airdrop_predict_impact_v121(releaseState,Params=p);

% Causal sample-ahead kinematic prediction.  We do not pretend to know the
% future nonlinear carrier state; over <=0.1 s a constant measured Vg/Vz model
% is sufficient for release-time interpolation, and is materially better than
% snapping the release to one of the MPC sample boundaries.
tauMax=Ts;
rs1=releaseState;
rs1.x_m=double(releaseState.x_m)+double(releaseState.vx_ground_mps)*tauMax;
rs1.h_m=max(0,double(releaseState.h_m)+double(releaseState.vz_up_mps)*tauMax);
rs1.wind_est_mps=max(min(double(releaseState.wind_est_mps)+double(releaseState.wind_rate_est_mps2)*tauMax,p.max_abs_forecast_wind_mps),-p.max_abs_forecast_wind_mps);
next=airdropx_airdrop_predict_impact_v121(rs1,Params=p);
advance=next.impact_x_m-now.impact_x_m;

S=struct();
S.version="AirdropX v1.3.2 grid-aligned causal fractional release scheduler";
S.release_now=false;
S.release_within_sample=false;
S.tau_s=NaN;
S.predicted_impact_now_m=now.impact_x_m;
S.predicted_impact_next_m=next.impact_x_m;
S.predicted_impact_advance_m=advance;
S.target_m=target_m;
S.estimated_impact_at_release_m=NaN;
S.scheduler_residual_m=NaN;
S.mode="wait";

% If already at/past the target according to the causal predictor, release at
% the current sample boundary.  This preserves safe behavior after any missed
% crossing rather than waiting for another cycle.
if now.impact_x_m>=target_m
    S.release_now=true; S.tau_s=0; S.estimated_impact_at_release_m=now.impact_x_m;
    S.scheduler_residual_m=now.impact_x_m-target_m; S.mode="immediate_boundary"; return
end
if ~isfinite(advance) || advance<opts.MinImpactAdvance_m
    return
end
if next.impact_x_m<target_m
    return
end

% First interpolation, followed by one causal predictor refinement.  The
% refinement still uses only the current measured state and the frozen short
% kinematic extrapolation model.
tau=tauMax*(target_m-now.impact_x_m)/advance;
tau=max(0,min(tauMax,tau));
for it=1:2
    rsi=releaseState;
    rsi.x_m=double(releaseState.x_m)+double(releaseState.vx_ground_mps)*tau;
    rsi.h_m=max(0,double(releaseState.h_m)+double(releaseState.vz_up_mps)*tau);
    rsi.wind_est_mps=max(min(double(releaseState.wind_est_mps)+double(releaseState.wind_rate_est_mps2)*tau,p.max_abs_forecast_wind_mps),-p.max_abs_forecast_wind_mps);
    pi=airdropx_airdrop_predict_impact_v121(rsi,Params=p);
    slope=advance/max(tauMax,eps);
    if abs(slope)<1e-6, break; end
    tau=max(0,min(tauMax,tau-(pi.impact_x_m-target_m)/slope));
end
% The persistent wind Oracle advances on its native 120 Hz grid.  Align the
% event timer to that grid so the requested split time and actual JSBSim
% propagation cannot silently disagree.
if opts.AlignToOracleGrid
    tau=round(tau/opts.OracleDt_s)*opts.OracleDt_s;
    tau=max(0,min(tauMax,tau));
end
rsi=releaseState;
rsi.x_m=double(releaseState.x_m)+double(releaseState.vx_ground_mps)*tau;
rsi.h_m=max(0,double(releaseState.h_m)+double(releaseState.vz_up_mps)*tau);
rsi.wind_est_mps=max(min(double(releaseState.wind_est_mps)+double(releaseState.wind_rate_est_mps2)*tau,p.max_abs_forecast_wind_mps),-p.max_abs_forecast_wind_mps);
pi=airdropx_airdrop_predict_impact_v121(rsi,Params=p);
if tau>=tauMax-1e-6
    % Crossing is effectively at the next MPC boundary; defer it so the mass
    % transition remains exactly synchronized with the next controller sample.
    return
end
S.release_within_sample=true;
S.tau_s=tau;
S.estimated_impact_at_release_m=pi.impact_x_m;
S.scheduler_residual_m=pi.impact_x_m-target_m;
S.mode="fractional_timer";
end
