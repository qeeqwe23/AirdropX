# PREDELIVERY AUDIT — AirdropX Physics-MPC v1.4.0

## Scope

This package was audited statically and with independent Python numerical checks in the build environment. MATLAB/Simulink/JSBSim nonlinear v1.4.0 missions were **not** executed here because that runtime is not installed. No nonlinear PASS result is fabricated.

## Completed checks

- `tests/static_truth_isolation_audit_v140.py`: **44/44 PASS**; verifies that formal MPC/release inputs use `xCtrl`/navigation estimates rather than direct JSBSim truth, realistic avionics are default, independent cargo scoring is default, formal runners force those modes, and child windows remain hidden.
- `tests/independent_cargo_truth_audit_v140.py`: **6/6 PASS**; independently reproduces the v1.4.0 cargo-plant calibration and confirms structural mismatch from the onboard predictor at 200 m / 50 m/s.
- `tests/table_constructor_static_audit_v140.py`: **5/5 PASS**; checks value/name counts for all new MATLAB `table(...,'VariableNames',...)` constructors.
- `tests/source_balance_audit_v140.py`: **13/13 PASS**; basic delimiter/brace balance audit for new MATLAB and PowerShell sources.
- No `.mexw64`, `.dll`, or `.exe` binaries are shipped in the package.
- Package hashes/manifests are regenerated after all edits.

## Numerical cargo-model audit

Independent Python replication obtains approximately:

- fitted `CdA = 0.488313744465 m^2`;
- calibration error effectively zero at the one shared historical calibration point;
- old onboard predictor, 200 m / 50 m/s calm range: about `289.0438 m`;
- independent scoring plant range: about `286.9798 m`;
- structural difference: about `-2.0640 m`;
- old predictor fall time: about `6.3866 s`;
- independent scoring plant fall time: about `6.7628 s`.

This mismatch is intentional. It demonstrates that the landing score is no longer produced by a near-copy of the release predictor. It does **not** claim the independent scoring plant is physical flight truth.

## Truth isolation policy

Allowed uses of JSBSim truth:

- plant integration;
- simulated sensor stimulus;
- post-run state/wind/mass-property scoring;
- independent scoring initialization at the actual simulated release point.

Prohibited formal online uses:

- MPC state feedback;
- release position/height/velocity;
- release wind input;
- gust/recovery trigger from truth wind.

The source audit checks the important direct-use contracts. `-IdealStateFeedback` and `-SharedCargoTruth` remain explicit point-test ablations and are not formal v1.4.0 evidence.

## Remaining real-aircraft caveats

- `physics_bank.mat` and `wind_disturbance_model_v130.mat` are still JSBSim-derived offline models and must be replaced/validated with flight-test/CFD/manufacturer/system-ID data before real-aircraft certification.
- Current sensor models do not yet include full asynchronous update rates, transport latency, packet loss, GNSS multipath, correlated INS drift, pitot position error, icing/failure modes, or clock synchronization error.
- `FuelScale` is still a mission input; hardware should feed it from fuel quantity/flow estimation.
- simulated `cfg` changes on a successful scheduled release event; hardware should require positive release confirmation before model scheduling.
- real payload impact truth must come from an independent measurement system.

## Required first run on the project machine

1. Install v1.4.0.
2. Run `run_base_equivalence_audit_v140_D.ps1`.
3. Run `calm`, `headwind_12`, `sine_longitudinal` point cases.
4. Inspect estimator p95 errors and formal gates.
5. Only then run the full 25-case suite.
