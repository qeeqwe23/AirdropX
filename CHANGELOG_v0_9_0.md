# Physics-MPC v0.9.0 — Continuous H×V Interval

## Goal
Move from discrete H×V node validation to a continuous operating-point scheduler over:

- H_cmd in [20, 200] m
- Va_cmd in [45, 65] m/s
- cfg 0..4
- fixed Np=Nc=100
- PreviewOnly, q-soft OFF
- drops [10.0 10.2 10.4 10.6] s

Formal Richardson certification is intentionally not a blocking criterion in this release. The v0.8.2 475-node bank is used as a computationally usable physics anchor set.

## New controller layer
For an arbitrary in-range (H,V), each cfg is built from the four surrounding H×V bank nodes by bilinear interpolation of:

- discrete A and B
- trim xref and uref

Unified Q/R and Bryson scales are inherited unchanged. Terminal P/K are recomputed from the interpolated A/B using `dlqr`; the actual MPC horizon remains 100 everywhere.

No nearest-neighbor controller substitution is used. Out-of-range requests fail rather than clamp silently.

## Interval validation
v0.9.0 performs three complementary checks:

1. **Dense computational audit:** H=20:1:200 and V=45:1:65 (3801 operating points), all cfg0..4. Checks interpolation geometry, finite A/B/trim, DARE solvability/rho<1 and trim-input margin.
2. **Every interpolation cell nonlinear coverage:** all 18×4=72 H×V cells, two strictly off-grid points per cell = 144 nonlinear JSBSim four-drop missions.
3. **Existing on-grid reference:** reads the v0.8.2 95/95 grid mission evidence when present.

The interval result is a performance/coverage validation, not a mathematical proof over infinitely many real-valued commands.
