# AirdropX Physics-MPC v1.3.6

v1.3.6 is the result-driven follow-up to the real v1.3.5 `calm`, `headwind_12`, and `sine_longitudinal` point runs.

v1.3.5 established that:

- calm is genuinely base-equivalent and stable when the disturbance path is off;
- the fractional-release Iyy one-substep lag is fixed in the `mass-refresh-v135` JSBSim Oracle;
- precision airdrop is already at roughly metre-class error in the tested longitudinal-wind cases;
- the remaining `headwind_12` final/tail issue is associated with long-lived carrier disturbance ownership and late engine-state settling;
- sine forced response is acceptable, but 55 s did not leave a long enough guaranteed base-controller settling interval for the strict last-5-s tail.

## Main v1.3.6 change: one wind estimate, two meanings

v1.3.6 separates wind evidence used by the release system from wind evidence used by the carrier controller.

```text
                    wind estimator
                          |
                +---------+---------+
                |                   |
                v                   v
        absolute-wind evidence   transient evidence
                |                   |
                v                   v
          release guidance       carrier MPC
```

A constant `-12 m/s` headwind remains available to release guidance for as long as it is credible, but it no longer holds the carrier disturbance QP active forever after the wind step has finished.

Carrier transient evidence starts from a qualified abrupt gust or sustained confidence-qualified wind-rate event. Rate evidence can maintain an established event through smooth ramp/sine zero crossings. If the wind has become steady and recovery has ended, the transient latch expires and control returns to the exact base `airdropx_phys_mpc_solve` path.

## Solver ownership is now deterministic

v1.3.5 could repeatedly select/deselect the disturbance QP when predicted disturbance magnitude crossed a small `gNorm` deadband. The real `headwind_12` trace showed many late switches.

v1.3.6 removes that deadband from solver ownership:

```text
carrier transient / gust recovery active
    -> disturbance-aware MPC

otherwise
    -> exact base Physics-MPC solver
```

This is intended to let throttle/N1/N2 settle cleanly after a constant-wind transient has ended.

## Sine gets a genuine post-forcing tail

The sine profile itself is unchanged during its main forcing interval:

- 0--45 s: original forcing;
- 45--47 s: half-cosine taper to zero;
- 47--60 s: zero wind.

Only the sine mission is extended to 60 s. Therefore the unchanged formal last-5-s tail is now 55--60 s.

For sine, formal tail certification additionally requires:

```text
tail5_zero_truth_wind_fraction = 1
tail5_base_solver_fraction     = 1
```

The numerical limits remain unchanged:

```text
final normalized inf <= 0.10
tail5 normalized RMS <= 0.05
```

## Iyy / JSBSim Oracle

v1.3.6 deliberately reuses the already validated v1.3.5 `mass-refresh-v135` Oracle. It does not force an unnecessary rebuild on every install. If the MEX or v1.3.5 selftest marker is missing, the runners automatically rebuild/selftest it.

## Hidden child windows

All runner-spawned MATLAB processes remain hidden by default while stdout/stderr are written to result files. Use `-ShowChildWindows` only for debugging.

## Recommended run order

Install:

```powershell
.\install_wind_disturbance_airdrop_v136.ps1
```

Then:

```powershell
cd 'D:\vscode project\AirdropX'

.\run_base_equivalence_audit_v136_D.ps1

.\run_wind_disturbance_airdrop_point_v136_D.ps1 -Scenario calm
.\run_wind_disturbance_airdrop_point_v136_D.ps1 -Scenario headwind_12
.\run_wind_disturbance_airdrop_point_v136_D.ps1 -Scenario sine_longitudinal
```

Only if those are structurally correct, run the full suite:

```powershell
.\run_wind_disturbance_airdrop_v136_D.ps1
```

Maximum formal parallelism remains 3 workers.

## Result roots

Base equivalence:

`matlab\results\physics_mpc_v136_base_equivalence_audit\`

Point tests:

`matlab\results\physics_mpc_v136_transient_energy_recovery_point\`

Full validation:

`matlab\results\physics_mpc_v136_transient_energy_recovery_airdrop_validation\`

Full summary:

`wind_transient_energy_recovery_validation_summary.txt`

## New diagnostics

In addition to the existing landing/recovery/actuator metrics, v1.3.6 records:

- `release_wind_evidence_active_fraction`;
- `carrier_transient_evidence_active_fraction`;
- `carrier_transient_event_count`;
- `carrier_transient_switch_count`;
- `wind_mpc_switch_count`;
- `last_carrier_transient_time_s`;
- `last_wind_mpc_active_time_s`;
- `tail5_base_solver_fraction`;
- `tail5_zero_truth_wind_fraction`;
- `zero_wind_settle_duration_s`.

## Offline audits and limits of the package

The packaging environment does not contain the user's MATLAB/JSBSim runtime, so no nonlinear v1.3.6 PASS result is fabricated.

Static/independent checks include:

- v1.3.6 static contract audit;
- independent transient-evidence policy audit;
- replay of the actual v1.3.5 point traces through the new transient policy;
- MATLAB table-column/name arity checks;
- archive integrity verification.

The actual formal result must come from the user's project machine.
