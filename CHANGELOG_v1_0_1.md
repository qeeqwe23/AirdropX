# v1.0.1 changelog

- Replaced level-flight moving-command reference with dynamic-feasible `gamma_ref/theta_ref/q_ref`.
- Added exact moving-reference affine defect relative to the level-trim linearization point.
- Added a dedicated runtime solver that propagates the same affine defect used during condensation.
- Added kinematic reference self-check `Va*sin(gamma_ref)=Hdot_cmd`.
- Preserved Q/R, scales, Np=Nc=100, q-soft OFF, hard input bounds, and all existing performance gates.
- Reruns all six v1.0 scenarios; v1.0 results are comparison evidence only, never reused as v1.0.1 mission results.
