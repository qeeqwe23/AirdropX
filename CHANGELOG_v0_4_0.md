# v0.4.0

- Adds the first true receding-horizon QP controller around the validated v0.3.3 physics layer.
- Nonlinear plant is the exact JSBSim Oracle at every closed-loop sample.
- Uses certified `Ad/Bd`, unified `Q/R`, and terminal `P/K`; no BO or cfg-specific tuning.
- Adds hard elevator/throttle command bounds only.
- Adds QP-vs-LQR condensation self-test before plant execution.
- Adds per-step one-step linear-vs-nonlinear prediction error logging.
- Adds QP feasibility/solve-time metrics, tracking metrics, Lyapunov recovery metric, CSV/MAT evidence, and plots.
- Controller overlay does not rebuild or overwrite the validated v0.3.3 Oracle/S-Function MEX.
