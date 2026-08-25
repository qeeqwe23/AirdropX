# v1.3.5

- Replaced rate-confidence OR-gating with a persistent wind-amplitude evidence latch.
- Rate confidence can extend, but cannot initiate, disturbance-MPC activity.
- Added `disturbance_evidence_active_fraction` and per-sample evidence logging.
- Added seven-state / input / release same-seed base-equivalence audit and preflight runner.
- Full runner aborts before the 25-case suite if base equivalence fails.
- Fixed JSBSim derived Iyy lag after very-late fractional cfg transitions by zero-dt mass-property refresh.
- Added 1/120-s cfg-transition Iyy regression to wind Oracle selftest.
- Changed Oracle version marker to `v1.2.1+mass-refresh-v135` and force-rebuild on install.
- Added v1.3.5 sine validation profile: forcing to 45 s, smooth 45-47 s shutdown, zero-wind settling tail.
- Added explicit sine forced-response RMS/peak metrics while retaining strict final/tail gates.
- Point runner defaults to formal manifest sensor-noise seeds 101..108.
- Hidden child-process behavior retained.
