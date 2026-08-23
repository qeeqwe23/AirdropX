# Physics-MPC v1.0.0

- Adds one-flight runtime tracking of continuously changing H_cmd(t) and Va_cmd(t) inside H=[20,200] m and Va=[45,65] m/s.
- Reuses the v0.8.2 475-anchor usable physics bank and the v0.9.0 bilinear HxV interpolation policy.
- Builds a genuinely time-varying N=100 prediction model each control sample: H/V references, A/B, trim state/input, and known cfg0->cfg4 payload schedule may all change across the horizon.
- No retrim, re-identification, BO, controller restart, q-soft, or per-speed/per-altitude Q/R tuning is introduced.
- Adds runtime total-compute timing: H/V interpolation + time-varying QP condensation + quadprog solve must meet the 100 ms sample period at p95.
- Adds six deterministic full-range command-motion missions. Four cargo releases remain 0.2 s apart and occur while commands are moving.
- Keeps a fallback CommandPreviewMode="hold_current" API for unpredictable future commands; formal v1.0 validation uses known_reference so the MPC can preview the commanded reference trajectory as well as the known payload schedule.
