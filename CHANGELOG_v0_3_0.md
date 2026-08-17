# v0.3.0 change log

## Removed as primary trim method

- 5-D fsolve over `[theta,elevator,throttle,N1,N2]`
- `MaxScaledResidual=1e-5` fixed-point gate from v0.2.x

## Added

- native JSBSim longitudinal FGTrim seed
- JSBSim longitudinal seed uses wdot/alpha and udot/throttle, while qdot is explicitly remapped from native pitch-trim to the primary elevator (`EditState(tQdot,tElevator)`)
- native JSBSim propulsion steady-state solve for each throttle candidate
- bounded 3-D `lsqnonlin` polish of exact Ts-map
- rate-based acceptance criteria in physical units
- full seven-state fixed-point verification
- repeated-evaluation deterministic gate plus A→B→A path-independence/reset gate before Jacobian
- undeclared pitch/roll/yaw trim, aileron and rudder commands explicitly zeroed at every Oracle reconstruction
- debug level 0 before model load
- safer finite-difference perturbations
- transactional MEX builds with candidate load-check and automatic rollback on install/final-load failure
- full Smoke diagnostics and result persistence

## Pre-delivery hardening

- fixed `InitRunning(-1)` full-throttle initialization ordering by re-applying the requested throttle before `GetSteadyState()`
- froze fuel consumption inside each local LPV oracle map; fuel/mass remain scheduling parameters, not hidden states
- replaced heuristic fuel-tank discovery with `FGPropulsion::GetNumTanks()/GetTank()`
- added direct cargo/fuel/N1/N2/control write-readback verification
- added cfg0..4 + fuelScale mass-configuration audit to Smoke
- confirmed from JSBSim source that `RunIC()` itself does not call `FGPropulsion::InitModel()`; no unnecessary custom refresh/reset layer was added

- made trim acceptance physics-based; optimizer `exitflag` is diagnostic and cannot veto an independently valid fixed point

## Final pre-delivery hardening

- Reset JSBSim `FGInitialCondition` on every persistent-oracle reconstruction and readback-verify `h/Va/gamma/theta/q` after `RunIC()`.
- Added `theta(k+1)-theta(k)` to the final exact-map trim acceptance gate; the full 7-state fixed point is no longer inferred from q alone.
- Added an installed-header preflight for `FGInitialCondition::InitializeIC`.
