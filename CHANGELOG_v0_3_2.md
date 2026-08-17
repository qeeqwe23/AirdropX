# v0.3.2 change log

Date: 2026-08-17

## Why v0.3.1 failed

The v0.3.1 Smoke reached the real JSBSim trim and exposed an important inconsistency:

- the nonlinear solver drove the sampled endpoint residual to essentially zero;
- `Va`, `gamma`, `q`, N1 and N2 returned to their initial values after 0.1 s;
- but `theta` changed by about -5.23e-4 rad in the same interval;
- the saved initial diagnostic showed large nonzero `udot`, `wdot` and `qdot` even though the sampled endpoint happened to return to q≈0.

That is not a valid steady-flight trim. It means the old endpoint objective could cancel a transient over the sample instead of enforcing instantaneous force/moment equilibrium.

The root cause in the Oracle lifecycle was also identified: after the final elevator/throttle/engine-state writes, the stored JSBSim accelerations and propagation derivative history still belonged to the earlier zero-dt RunIC pass. The first real integration step could therefore consume stale derivatives.

## v0.3.2 corrections

1. **Post-control zero-dt dynamics prime**
   - after every final state/control/engine write, suspend integration;
   - execute one JSBSim model pass at dt=0;
   - call `FGPropagate::InitializeDerivatives()` using the now-current accelerations;
   - resume integration;
   - verify elevator/throttle echoes, mass/configuration, declared kinematics and N1/N2 again.

2. **Physical trim objective changed to the equations of motion**
   - optimizer variables remain only `[theta, elevator, throttle]`;
   - N1/N2 remain eliminated through JSBSim engine steady-state physics;
   - the optimizer now solves `udot=0`, `wdot=0`, `qdot=0` at the reconstructed state;
   - the exact 0.1 s seven-state map is retained only as an independent validation gate.

3. **No threshold relaxation**
   - path-independence tolerance remains `1e-10` in state units;
   - theta remains in the discrete fixed-point gate;
   - solver exit status remains diagnostic rather than the definition of physical PASS.

4. **Control-coordinate verification**
   - diagnostics now export command and physical normalized position for elevator and throttle;
   - every Oracle reconstruction verifies those values match the requested input before propagation;
   - self-test also requires a valid control echo.

5. **Smoke diagnostics corrected**
   - failure output now prints scalar `udot/wdot/qdot` and all six discrete rate residuals without MATLAB `error` vector-formatting failure;
   - failure MAT retains physical acceleration, control echo, self-test and derivative evidence.

6. **Version/output isolation**
   - Oracle version is v0.3.2;
   - results are written under `matlab/results/physics_mpc_v032`;
   - the supplied v0.3.1 Smoke and failure MAT are archived in this package for regression reference.
