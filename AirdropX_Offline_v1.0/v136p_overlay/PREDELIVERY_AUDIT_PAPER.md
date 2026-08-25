# Predelivery audit — Physics-MPC v1.3.6-Paper v1.0

## Completed offline checks
- `tests/static_contract_check_v136p.py`: 28/28 PASS.
- `tests/source_balance_audit_v136p.py`: all listed MATLAB/PowerShell source files pass raw delimiter/function-name checks.
- No `.mexw64`, `.dll` or `.exe` is shipped in the package.
- Formal online control/release path references sensor estimates, not JSBSim state/navigation truth.
- Formal wind guidance uses the causal wind estimator; scripted wind is scoring/environment only.
- Common noise seeds are preserved across controller A/B runs.

## Not claimed
This environment does not contain MATLAB/Simulink/JSBSim, so no nonlinear runtime result is fabricated.
Run the equivalence audit and three key point cases on the project machine before the full suite.
