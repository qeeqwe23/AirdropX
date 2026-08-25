# v1.1.3

## Confirmed v1.1.2 failure root cause
The uploaded v1.1.2 result passed the Simulink capability probe, created the private IC, and then failed in the serial `calm` preflight with:

`Simulink:SFunctions:SFcnParamCountErr`

The compiled `sfun_airdropx_jsbsim` declares four S-Function parameters, but the dynamic harness created the library S-Function block with `FunctionName` already active while its dialog `Parameters` field was still empty. Simulink therefore validated a 4-vs-0 parameter count during `add_block` before the next `set_param(...,"Parameters",...)` line could execute.

## Fix
- Create the generic S-Function library block with no active function name.
- Build the four-argument dialog parameter expression first.
- Set the block `Parameters` field first.
- Activate `FunctionName="sfun_airdropx_jsbsim"` only after the dialog parameter list is populated.
- Verify the configured FunctionName/Parameters.
- Force an explicit model `SimulationCommand="update"` preflight before calibration.
- Add `SFUNCTION_CONFIGURED`, `MODEL_UPDATE_START`, and `MODEL_UPDATE_DONE` status markers.

## Unchanged
The longitudinal wind estimator, sensor-noise Monte Carlo, performance gates, 8 wind profiles, private-IC isolation, and serial `calm` harness preflight are unchanged.
