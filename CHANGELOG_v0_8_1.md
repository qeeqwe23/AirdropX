# v0.8.1

- Added continue-on-certification-failure physics slice builder.
- Preserves original Richardson thresholds; no gate relaxation.
- Added `pass` vs `usable` distinction for physics rows.
- Added diagnostic vertex builder that retains finite Richardson-failed models while marking them uncertified.
- Extended vertex getter / cfg scheduler with explicit opt-in `AllowUncertified` path; default remains strict.
- Added full 475-row diagnostic bank merge.
- Added all-95-case diagnostic mission entry; hard-unusable H/V cases become MODEL_UNAVAILABLE rather than using a fake fallback.
- Added finalizer that separates formal certification from observed nonlinear mission performance.
- Reuses completed v0.8.0 strict speed slices when available; incomplete slices are rescanned.
- No changes to Q/R, state/input scales, mission gates, q-soft setting, Oracle MEX, S-Function MEX, or drop schedule.
