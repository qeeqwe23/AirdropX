# v0.3.1 change log

Date: 2026-08-17

## Why v0.3.0 stopped

The first v0.3.0 Windows Smoke proved that the new physical trim is valid at
200 m / 50 m/s / cfg0: the full fixed-point and height/physics gates passed.
The only failure was the deliberate A->B->A Oracle path-independence gate
(max state error about 2.3e-4).

This was not treated as a threshold problem.

JSBSim 1.3.1 source shows that `FGFDMExec::RunIC()` reconstructs the IC and
initializes Input/Output, but it does not call `InitModel()` on all models.
`FGFCS::InitModel()` is the routine that resets FCS channel components. A
persistent Oracle could therefore retain hidden FCS/model memory after a B call
and contaminate the next A call even though A/A/A repeatability was excellent.

## Structural fix

Every Oracle evaluation now begins with:

`FGFDMExec::ResetToInitialConditions(FGFDMExec::DONT_EXECUTE_RUN_IC)`

This invokes JSBSim `InitializeModels()` to reset model memories but deliberately
does not run IC yet. The Oracle then reconstructs the declared IC, cfg/fuel,
controls and N1/N2 and calls `RunIC()` itself. This matches the intended use of
`DONT_EXECUTE_RUN_IC`: reset first, then let the caller set control/state values
that a normal reset would otherwise erase.

No path-independence or semigroup tolerance was relaxed.

## Diagnostics hardening

- Oracle self-test can return a failed report without throwing immediately.
- Smoke saves that full self-test report before raising the named error.
- A failed path test now records and prints all seven state errors, not only the
  infinity norm.
- v0.3.1 uses a separate `physics_mpc_v031` results directory.
- The MEX header preflight now requires `ResetToInitialConditions` and
  `DONT_EXECUTE_RUN_IC` before linking.
