AirdropX UR-MPC v2.0.2 — model-validity rate-trust patch
=========================================================

Purpose
-------
V2.0.1 correctly separated physical absolute actuator authority from the trim
manifold, but V50 data showed that unlimited one-sample MV changes caused
bang-bang commands before any QP failure. In cfg1, most elevator steps exceeded
0.012/sample and about half of throttle steps exceeded 0.025/sample, while the
older bounded run remained smooth.

This patch does NOT add tube MPC, recovery, TECS, H-PI, or cfg-specific tuning.
It keeps physical absolute MV hard bounds and adds one unified SOFT MV-increment
trust envelope derived from the already-certified source physics-bank metadata:
  elevator_deviation_rate_limit (expected 0.012/sample)
  throttle_deviation_rate_limit (expected 0.020/sample)
If unavailable, the build falls back to the source ID excitation amplitudes.

Why soft
--------
The rate envelope expresses local-model validity, not physical actuator safety.
Therefore RateMinECR/RateMaxECR are positive. The QP can exceed the envelope if
tracking/safety genuinely requires it, but ordinary small errors no longer
justify an unvalidated 0->1 throttle jump in one 0.1-s interval.

Build changes
-------------
- physical MV absolute bounds remain unchanged from v2.0.1
- trim-derived absolute envelope remains diagnostic/scaling only
- model-validity rate limits are source-bank derived and common to all cfg/speed
- linear transition CSV now records max_elevator_step, max_throttle_step,
  rate_soft_exceed_count and max_slack
- 12/12 transition certification is now an actual build gate

Run rule
--------
This changes the MPC object. Rebuild with SkipBuild=False.
First acceptance target is NOT 'all 5 pass at any cost'. It is:
1) 55/55 vertex and 12/12 transition certification still pass;
2) cfg0 remains unchanged;
3) cfg1 loses the v2.0.1 bang-bang/saturation regression;
4) cfg2 behavior is then re-evaluated before any polytopic/tube design.
