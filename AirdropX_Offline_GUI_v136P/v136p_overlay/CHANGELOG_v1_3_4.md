# v1.3.4

Targeted fixes from the three-point v1.3.3 nonlinear test.

- Continuous ramp/sine rate evidence no longer activates the recovery Q/R bank by default; Gw preview remains active.
- Calm/no-credible-disturbance samples hard-fall back to the original certified Physics-MPC solver and clear residual disturbance state.
- Strong recovery remains restricted to confidence-qualified abrupt wind jumps.
- Strong-gust recovery bank is rebalanced toward elevator/energy exchange instead of pushing an already saturated throttle.
- Energy altitude borrowing is capped by both 3 m and 2% of nominal altitude (0.4 m at 20 m, 3 m at 200 m).
- Iyy configuration validation cancels only the fixed cfg0 Oracle-vs-bank origin bias and still gates strict cfg-dependent inertia deltas.
- Plot titles use Interpreter=none, eliminating underscore interpreter warnings.
- Hidden MATLAB child-process behavior from v1.3.3 is retained.
