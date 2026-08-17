AirdropX UR-MPC v2.0 runtime option bridge fix

Problem fixed:
  airdropx_urmpc_run_closed_loop passed AircraftName/IcName to
  airdropx_auto_run_closed_loop, whose local_options does not define them,
  causing: Unknown option: AircraftName.

Fix:
  Remove AircraftName/IcName from the legacy auto-runner call boundary.
  The UR-MPC wrapper still accepts these options from the fixed-stability
  scan for compatibility, but the legacy runner is invoked only with options
  it actually supports.

Unchanged:
  UR-MPC bank, 55-point vertex preflight, 12 cfg transition certification,
  Bryson weights, Np/Nc, MV bounds, disturbance estimator, controller S-function,
  initial H/V/pitch/elevator/throttle values, drop schedule, and evaluation rules.
