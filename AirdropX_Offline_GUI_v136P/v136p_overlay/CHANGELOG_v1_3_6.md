# AirdropX Physics-MPC v1.3.6

Result-driven follow-up to the real v1.3.5 point validation.

## Why v1.3.6 exists

The v1.3.5 point runs established that the calm base-equivalence and fractional-release Iyy problems were fixed, while two controller/validation issues remained:

1. In `headwind_12`, absolute wind confidence kept the carrier disturbance evidence alive almost to the end of the mission even after the wind had become constant and the aircraft had largely recovered. This allowed the disturbance QP to repeatedly enter/leave on small predicted-disturbance changes, producing unnecessary late throttle/N1/N2 activity and hurting final/tail settling.
2. In `sine_longitudinal`, the wind was physically zero after 47 s, but the carrier transient evidence did not clear until roughly 49--51 s. A 55 s mission therefore gave too little guaranteed base-controller time before the formal final 5 s tail.

## Changes

### Separate release absolute-wind evidence from carrier transient evidence

- `release_wind_evidence_active` tracks credible absolute wind for the impact/release guidance path.
- `carrier_transient_evidence_active` tracks gust/rate transients for the carrier MPC only.
- Constant non-zero wind can remain fully available to release guidance without holding the carrier disturbance solver active indefinitely.
- Carrier transient activation requires either a qualified abrupt gust or a sustained confidence-qualified wind-rate event.
- Rate evidence can bridge ramp/sine zero crossings after an event has been established.
- Constant absolute wind amplitude is deliberately not a carrier keep-alive condition.

### Deterministic solver ownership

Removed the old `gNorm` deadband from solver ownership. The disturbance solver is now selected only while carrier transient evidence, a gust-recovery latch, or a nonzero recovery level is active. Otherwise the exact legacy/base `airdropx_phys_mpc_solve` path is used.

This is intended to eliminate the repeated QP on/off chatter observed in the real v1.3.5 `headwind_12` trace.

### Longer sine settling mission

- `sine_longitudinal` duration is now 60 s; all other formal scenarios remain 55 s.
- Original sine forcing remains through 45 s.
- Wind is half-cosine tapered to zero from 45--47 s.
- 47--60 s is true zero wind.
- Formal tail remains the last 5 s, therefore 55--60 s.

No final/tail numerical limit was relaxed.

### Stronger tail-context audit

For sine, formal certification additionally requires:

- `tail5_zero_truth_wind_fraction == 1`;
- `tail5_base_solver_fraction == 1`.

Thus a passing tail cannot accidentally include active wind forcing or the disturbance MPC.

### New diagnostics

Added/reporting retained for:

- release wind evidence fraction;
- carrier transient evidence fraction/event count/switch count;
- wind-MPC switch count;
- last carrier-transient time;
- last wind-MPC-active time;
- tail base-solver fraction;
- tail zero-wind fraction;
- zero-wind settling duration.

### Oracle / Iyy

No C++ Oracle change is made in v1.3.6. The validated v1.3.5 `mass-refresh-v135` Oracle is reused. Runners self-heal and rebuild/selftest only when the MEX or v1.3.5 Oracle marker is genuinely missing.

### Runtime UI

Child MATLAB processes remain hidden by default. `-ShowChildWindows` remains opt-in for debugging.
