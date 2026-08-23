# v1.3.6 pre-delivery audit

This package is a result-driven correction based on the user's real v1.3.5 `calm`, `headwind_12`, and `sine_longitudinal` point runs. The packaging environment has no MATLAB/JSBSim runtime, so no nonlinear v1.3.6 PASS result is claimed.

## Real-result observations used to drive this revision

- v1.3.5 `calm` passed with zero wind-MPC/recovery activity and near-zero final/tail error, confirming base stability/equivalence and the v1.3.5 Iyy fix.
- v1.3.5 `headwind_12` had good landing accuracy and recovered its main aircraft states, but absolute-wind evidence remained active through almost the full mission and the disturbance solver repeatedly switched late in the mission. Final/tail were then dominated by slow engine-state settling.
- v1.3.5 `sine_longitudinal` passed final and forced-response gates but failed only the strict tail gate. The real trace showed monotonic settling after wind-MPC ownership ended, with too little guaranteed base-controller time before the 50--55 s tail window.

## v1.3.6 structural changes audited

- release absolute-wind evidence and carrier transient evidence are separate signals;
- constant absolute wind is not a carrier transient keep-alive condition;
- carrier transient activation requires a qualified abrupt gust or sustained confidence-qualified wind rate;
- carrier transient persistence is causal and finite;
- rate evidence can bridge an already-established smooth event;
- solver ownership no longer depends on predicted-disturbance `gNorm` crossing a deadband;
- inactive carrier transient/recovery uses the exact base Physics-MPC solver;
- sine formal mission duration is 60 s while all other formal scenarios remain 55 s;
- sine truth wind is zero from 47 s onward;
- sine formal last-5-s tail must be both zero-wind and base-solver-only;
- formal final/tail numerical thresholds are unchanged;
- child MATLAB windows remain hidden by default;
- formal parallelism remains capped at three;
- v1.3.5 mass-refresh Oracle is reused and only rebuilt/selftested if missing.

## Independent checks performed

- `tests/static_contract_check_v136.py`: 47/47 PASS.
- `tests/independent_v136_policy_audit.py`: PASS.
- `tests/replay_v135_transient_evidence_v136.py`: replayed real v1.3.5 traces under the new evidence policy:
  - calm remains inactive;
  - headwind becomes one finite transient episode instead of a mission-long absolute-wind latch;
  - sine transient evidence clears before the new 55--60 s formal tail.
- MATLAB table-construction arity checks passed for the v1.3.6 mission and finalizer.
- no precompiled `.mexw64`, DLL, or EXE is shipped.

## Replay is not a nonlinear result

Replay only evaluates event/ownership logic against the recorded v1.3.5 estimator/time-series signals. It cannot prove the closed-loop v1.3.6 nonlinear trajectory. Expected improvements in headwind final/tail and sine settling must be confirmed on the user's MATLAB/JSBSim machine.

## Recommended acceptance order

1. `run_base_equivalence_audit_v136_D.ps1`.
2. Point `calm`.
3. Point `headwind_12`.
4. Point `sine_longitudinal`.
5. Only then the full 25-case runner.
