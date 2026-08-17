# v0.3.3 — JSBSim alpha-rate algebraic consistency fix

## Evidence from v0.3.2

At H=200 m, Va=50 m/s, cfg0, v0.3.2 found an apparent instantaneous equilibrium:

- udot = -8.12e-16 m/s^2
- wdot =  1.52e-14 m/s^2
- qdot = -1.49e-16 rad/s^2

but the exact 0.1 s propagation still produced:

- dVa/dt = +4.968e-2 m/s^2
- dq/dt  = -4.562e-3 rad/s^2

The saved diagnostics also showed that elevator, throttle and N1/N2 were unchanged, while the aerodynamic force/moment changed during the first real propagation step.

## Root cause

JSBSim 1.3.1 executes `FGAuxiliary` before `FGAerodynamics` and `FGAccelerations`.
`FGAuxiliary` computes `aero/alphadot-rad_sec` from the acceleration vector supplied by the previous model pass. The AirdropX MQ9 model uses alpha-rate terms in lift and pitching moment (`Lift_alpha_rate`, `Pitch_alphadot`).

Therefore one suspended-integration `Run()` is not enough to make

`alphadot -> aerodynamic forces/moment -> accelerations -> alphadot`

self-consistent. v0.3.2 optimized the acceleration at one side of this one-pass lag.

## v0.3.3 correction

- Replace the single zero-dt dynamics prime with repeated suspended-integration model passes.
- Iterate until successive values of alphadot, udot, wdot, qdot, Fx, Fz and My converge.
- Only then call `FGPropagate::InitializeDerivatives()` and resume real integration.
- If this algebraic loop does not converge within the hard numerical iteration limit, the Oracle rejects the operating point instead of emitting an A/B matrix.
- Export `alphadot_radps`, `algebraic_settle_iterations`, `algebraic_settle_error`, and `algebraic_settle_converged` in diagnostics.
- Trim acceptance now requires both the self-consistent EOM residual and near-zero alphadot at level-flight equilibrium, followed by the independent exact-Ts fixed-point gate.
- FGTrim seeds are passed through the same algebraic settling sequence before they are exported.
- Oracle self-test requires algebraic convergence in addition to determinism, path independence and semigroup closure.

No trim/MPC performance threshold was relaxed. Q/R, cfg semantics and the seven declared controller states remain unchanged.
