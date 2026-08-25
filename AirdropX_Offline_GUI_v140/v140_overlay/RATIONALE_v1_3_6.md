# v1.3.6 control rationale

## Observation from the real v1.3.5 runs

`calm` passed with the base solver and zero disturbance activity, proving the underlying Physics-MPC is stable and base-equivalent when no carrier transient is present.

For `headwind_12`, the release estimator correctly needed the absolute ~12 m/s wind for the whole mission, but the carrier does not need a permanent `wind-change` disturbance action after the step has finished and the air-relative state has recovered. Keeping absolute-wind evidence latched therefore mixed two different meanings of wind information.

For `sine_longitudinal`, the forced-response portion was already acceptable, but the strict final/tail test needed a longer interval after both wind forcing and transient-MPC activity had ended.

## Architecture

                 wind estimator
                       |
             +---------+---------+
             |                   |
             v                   v
      absolute wind         wind transient
       confidence             evidence
             |                   |
             v                   v
     release guidance       carrier MPC

Release guidance continues to use credible absolute wind. Carrier MPC uses transient information only.

## Carrier transient event semantics

An inactive carrier transient can start from:

- a qualified abrupt estimated wind step; or
- sustained rate evidence with both rate confidence and minimum absolute-wind confidence.

After activation, rate evidence can maintain the event through a sine zero crossing. A gust-recovery latch also maintains it while recovery is physically active. If neither condition persists, a fixed causal persistence timer allows the event to decay and then returns ownership to the exact base solver.

Absolute steady wind cannot maintain this latch by itself.

## Solver semantics

There are only two ownership states:

- no transient/recovery: exact base Physics-MPC solver;
- transient/recovery active: disturbance-aware solver.

Predicted disturbance magnitude is not used as a rapid solver-selector threshold. This avoids numerical chatter near a deadband.

## What v1.3.6 does not claim

The 12 m/s abrupt-gust 3 s recovery gate may still be limited by actuator/propulsion authority. v1.3.5 showed full throttle saturation through important early gust windows. v1.3.6 does not hide this by relaxing formal thresholds or by increasing hard actuator limits.
