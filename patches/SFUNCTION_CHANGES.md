# S-Function changes

The patched `sfun_airdropx_jsbsim.cpp` keeps output port 0 at width 20, so existing signal order is preserved.

Two corrections are made on that port:

- output 10 `mass_kg`: now reads JSBSim `inertia/mass-slugs` and therefore includes fuel + remaining cargo.
- output 11 `cg_x_m`: now reads JSBSim `inertia/cg-x-in`.
- output 20 (legacy reserved): now carries `q_dps` for backward-friendly diagnostics.

A new **unconnected-safe second output port** of width 12 is added:

1. Iyy kg m^2
2. alpha rad
3. gamma rad
4. u_aero m/s
5. w_aero m/s
6. q rad/s
7. udot m/s^2
8. wdot m/s^2
9. qdot rad/s^2
10. N1
11. N2
12. thrust N

The legacy `updateMassCg()` helper is intentionally left in the class because existing drop logic calls it, but its values are no longer used for port-0 mass/CG truth. A later cleanup can delete those cached members after Simulink regression passes.
