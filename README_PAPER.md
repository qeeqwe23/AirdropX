# AirdropX Physics-MPC v1.3.6-Paper

This package is the recommended **paper-validation baseline** derived from v1.3.6.
It keeps the validated v1.3.6 carrier/wind/release controller, but removes the two
most objectionable online truth shortcuts without importing the full engineering
avionics burden of v1.4.0.

## Formal paper architecture

```text
nonlinear JSBSim plant
        | truth only as simulated-sensor stimulus
        v
unbiased noisy sensor layer
GNSS/INS + baro + pitot + AHRS/IMU + engine telemetry
        |
        v
causal state/navigation estimate
   |                    |
   v                    v
Physics-MPC        wind estimator
                        |
                        v
                  release guidance
```

Exact JSBSim state, position, wind and mass properties remain available only for
plant propagation and post-run scoring.

## Why this is preferable for an MPC paper

- no exact JSBSim position/height is used by release scheduling;
- no exact JSBSim state vector is used by the formal MPC path;
- no truth wind is used by online guidance;
- sensor errors are **zero-mean white noise only**: no fixed biases, random walks,
  latency or dropouts, because those belong to an avionics-estimation paper;
- the same deterministic noise seed is used for controller A/B comparisons;
- JSBSim truth is used for RMS/peak/final/tail scoring, which is appropriate in SIL;
- strict legacy engineering gates are retained as diagnostics, but paper conclusions
  should use quantitative comparisons (RMS, peak, recovery, input effort, QP time),
  not a single arbitrary PASS/FAIL threshold.

## Default paper sensor assumptions

- GNSS along-track position sigma: 0.30 m
- GNSS groundspeed sigma: 0.10 m/s
- GNSS vertical speed sigma: 0.07 m/s
- barometric altitude sigma: 0.20 m
- pitot/air-data airspeed sigma: 0.15 m/s
- pitch sigma: 0.03 deg
- pitch-rate sigma: 0.01 deg/s
- N1/N2 telemetry sigma: 0.05

These are **study assumptions**, not claimed hardware specifications. State them in
the paper and add a sensitivity sweep if reviewers require it.

## Cargo scoring

The main MPC paper defaults to the nominal v1.3.6 ballistic scoring model so cargo
model uncertainty does not dominate the carrier-control contribution. The point
runner provides `-IndependentCargoTruth` as a supplementary sensitivity ablation.
Do not present the nominal cargo model as measured real-world truth.

## Two result layers

Each mission reports:

- `paper_core_pass`: experiment integrity (fair sensor path, no truth leakage, four
  drops, feasible QP, physical input bounds, real-time budget, mass-property consistency,
  wind-estimator sanity).
- `engineering_pass`: the old strict recovery/final/tail/landing gates, retained for
  continuity and stress-test diagnostics.

The paper should report the actual metrics and baseline improvement ratios even when
an extreme 12 m/s instantaneous step misses an arbitrary 3 s engineering gate.

## Run

```powershell
.\install_paper_validation_v136p.ps1
cd 'D:\vscode project\AirdropX'
.\run_paper_equivalence_audit_v136p_D.ps1
.\run_paper_validation_point_v136p_D.ps1 -Scenario calm
.\run_paper_validation_point_v136p_D.ps1 -Scenario headwind_12
.\run_paper_validation_point_v136p_D.ps1 -Scenario sine_longitudinal
```

Then run the full three-way suite:

```powershell
.\run_paper_validation_v136p_D.ps1
```

Ablations:

```powershell
# Ideal full-state feedback benchmark only
.\run_paper_validation_point_v136p_D.ps1 -Scenario calm -IdealStateFeedback

# Independent cargo-model sensitivity only
.\run_paper_validation_point_v136p_D.ps1 -Scenario headwind_12 -IndependentCargoTruth
```
