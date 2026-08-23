# Paper validation assumptions

## Allowed online information
- unbiased noisy estimates of altitude, airspeed, flight-path angle, pitch, pitch rate, N1/N2;
- unbiased noisy along-track position, ground speed and vertical speed;
- causal wind estimate from air-data and ground-velocity measurements;
- scheduled mass/CG/Iyy model indexed by payload configuration.

## Scoring-only information
- exact JSBSim carrier state and position;
- exact scripted wind;
- exact JSBSim mass/CG/Iyy for consistency checks;
- exact release-time plant state for landing scoring;
- exact gust onset for post-run metrics.

## Explicit paper simplifications
- no sensor fixed bias or random walk;
- no sensor latency/dropout/asynchronous rates;
- immediate, deterministic release actuator once the fractional timer fires;
- offline A/B/Gw may be identified from JSBSim;
- nominal cargo model is a modeling assumption, not measured flight truth.

These assumptions are appropriate if the paper contribution is the MPC/gust-control
law. If the paper claims flight-ready avionics or real-world meter-level airdrop,
use the v1.4.x realism branch instead.
