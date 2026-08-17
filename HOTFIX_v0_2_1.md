# Physics-MPC v0.2.2 hotfix

Fixes a MATLAB `arguments`-block interface bug in `airdropx_phys_step.m`.

## Root cause
`p.cfgId`, `p.fuelScale`, and `p.Ts` were declared as a name-value argument group, while the entire codebase calls `airdropx_phys_step(x,u,p)` with `p` as the third positional struct argument. MATLAB therefore reported that the function accepts exactly 2 positional inputs.

## Fix
`p` is now declared as a scalar positional struct. `cfgId` is required; `fuelScale` and `Ts` receive defaults inside the function.

Also removes the default from the positional `fuelScale` argument in `airdropx_phys_build_vertex.m`, keeping all required positional arguments ahead of the name-value `opts` block.

No C++ rebuild is required for this hotfix. Replace the two `.m` files and rerun Smoke.
