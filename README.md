# AirdropX Physics-Derived MPC v0.2

This version replaces the previous "continuous 5-state + assumed throttle force" concept with a stricter object:

`x(k+1) = Phi_Ts(x(k), u(k), p(k))`

where `Phi_Ts` is the **actual JSBSim transition over Ts = 0.1 s**.

## State and input

State:

`x = [h, Va, gamma, theta, q, N1, N2]'`

Input:

`u = [elevator_absolute, throttle]'`

`alpha = theta - gamma` is enforced when constructing JSBSim initial conditions.

Why N1/N2 are states: the current AirdropX wrapper explicitly warms and preserves turbine spool states. Ignoring them makes throttle-to-flight dynamics non-Markov. JSBSim turbine N1/N2 are writable properties, so the oracle can perturb them directly and derive an exact local discrete model.

## What changed compared with v0.1

1. No continuous engine approximation is required.
2. No `Ac/Bc -> ZOH` is required for the primary model. `Ad/Bd` are Jacobians of the exact 0.1 s nonlinear transition.
3. Trim is a fixed point of the same map used by MPC:

   - `Va(k+1)-Va = 0`
   - `gamma(k+1) = 0`
   - `q(k+1) = 0`
   - `N1(k+1)-N1 = 0`
   - `N2(k+1)-N2 = 0`

   Unknowns are `[theta, elevator, throttle, N1, N2]`.
4. Mass/CG/Iyy are read from JSBSim, including fuel. cfg0..cfg4 only switch point masses.
5. Existing ID/LPV data are never used to generate `A/B`; they remain cross-validation evidence.

## Integration

Copy:

- `matlab/phys_mpc/*` -> `<AirdropX>/matlab/phys_mpc/`
- patched `matlab/sfunc_jsbsim/sfun_airdropx_jsbsim.cpp` -> existing S-function folder after backing up the original.

Then in MATLAB R2026a:

```matlab
cd('D:\vscode project\AirdropX')
addpath('matlab')
addpath('matlab/phys_mpc')
addpath('matlab/sfunc_jsbsim')

build_sfun_airdropx_jsbsim
build_airdropx_jsbsim_oracle

r = airdropx_phys_smoke(pwd);
```

Do **not** launch the full envelope build until `r.pass==true`.

## First full vertex after smoke

```matlab
info = airdropx_phys_oracle_init(pwd);
c = onCleanup(@()airdropx_jsbsim_oracle_mex("close"));
z0 = airdropx_phys_seed_from_existing("N1",info.base_n1,"N2",info.base_n2);
v = airdropx_phys_build_vertex(200,50,0,1.0,z0);
```

Required gates:

- `v.trim.pass == true`
- `v.lin.converged == true`
- `v.terminal.rho < 1`

Only after those three are true should an MPC QP be connected to Simulink.

## Unified Q/R

Default state scales are tied to the current AirdropX certification scale, not cfg-specific tuning:

- h: 4 m
- Va: 2 m/s
- gamma: 3 deg
- theta: 5 deg
- q: 2 deg/s
- N1/N2: 10 percentage points

N1/N2 have zero tracking weight by default; they exist only to make the prediction model Markov. The same dimensionless preference vectors are used at all H/V/cfg.

## What this package intentionally does not claim

This environment cannot compile against your Windows MATLAB/JSBSim import library or run your Simulink model. Therefore the package is source-complete but **not runtime-certified here**. The first runtime gate is the smoke command above.

If `N1/N2 are unavailable or not writable` is raised, stop: do not fall back to a 5-state model. The MQ-9 engine interface must be inspected because that would mean hidden propulsion state is still unmodeled.

If Richardson convergence fails, stop: the local JSBSim map is nonsmooth at that vertex. Preserve left/right Jacobians or split the vertex; do not tune Q/R around it.

## Full 200 -> 20 m / V50 bank

After smoke passes, the default bank command uses the same formulation everywhere and continuation only as a numerical seed:

```matlab
r = airdropx_phys_build_bank(string(pwd), ...
    Heights=200:-10:20, Speeds=50, CfgIds=0:4, FuelScales=1.0);
```

This produces 95 physical vertices (19 heights x 5 payload configurations) without any cfg-specific Q/R or learning strategy. The output summary records trim residual, Richardson derivative convergence, closed-loop rho and automatically derived Np for every vertex.

PowerShell convenience:

```powershell
.\run_phys_mpc_D.ps1 -Mode Smoke
.\run_phys_mpc_D.ps1 -Mode Build -SkipBuild
```

`Workers` is accepted for command-line compatibility with the existing project scripts but v0.2 intentionally runs the oracle build serially. A single persistent JSBSim oracle is stateful; parallelization should use one oracle MEX instance per worker after the serial formulation is certified.
