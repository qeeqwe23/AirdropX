AirdropX Physics MPC v1.6 - Certification Horizon Consistency (NO-MEX)

Purpose
-------
Fix the final V45/cfg4 discrepancy found in v1.5:
the deterministic trim solver certified on 22 s / 7 s tail, while the formal
equilibrium gate used 28 s / 10 s tail. The same physical controls therefore
passed the short window but drifted outside the formal vz/hSlope gate later.

Changes
-------
1. The certifying deterministic 5x2 trim solver now uses EXACTLY the formal
   baseline horizon: 28 s total, 10 s tail.
2. All hard equilibrium limits are unchanged.
3. Pitch remains a state, not a Newton decision variable.
4. Physical/external elevator coordinate handling and cfg-continuation from
   v1.5 are unchanged.
5. NO MEX compilation/rebuild.
6. 3 speed workers remain supported.
7. Compatible reuse: v1.5 nodes are reused only if they already passed formal
   baseline, independent repeatability, controllability rank=4, and finite
   held-out validation. Therefore the 14 v1.5 passing nodes need not be rebuilt;
   V45/cfg4 is recomputed under the corrected horizon.

Expected next run
-----------------
V45 cfg0..cfg3, V50 cfg0..cfg4, V55 cfg0..cfg4:
  [PHYS-MPC] COMPAT-REUSE ... from v1.5 formal certification
V45 cfg4:
  recomputed with 28 s / 10 s deterministic trim evaluations.

Do NOT run dynamic validation until the model bank builds 15/15.
