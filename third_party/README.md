# Third-party Dependencies

This directory vendors the JSBSim files needed by the MATLAB/Simulink plant.

- `JSBSim/`: complete JSBSim source tree copied from the project checkout. The local `.git` directory and generated `build` directory are intentionally excluded.
- `jsbsim-win64/`: Windows x64 development package used by `matlab/sfunc_jsbsim/build_sfun_airdropx_jsbsim.m`, including headers and `lib/JSBSim.lib`.

From MATLAB, rebuild the S-Function with:

```matlab
cd matlab/sfunc_jsbsim
build_sfun_airdropx_jsbsim
```

If the bundled Windows library is incompatible with a different compiler or MATLAB release, rebuild JSBSim from `third_party/JSBSim` and pass that install root explicitly:

```powershell
cd AirdropX/third_party
./build_jsbsim_win64.ps1
```

```matlab
build_sfun_airdropx_jsbsim("C:\path\to\jsbsim-install")
```
