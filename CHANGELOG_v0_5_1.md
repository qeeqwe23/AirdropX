# v0.5.1 changelog

- Added exact 2 s release schedule `[10 12 14 16]`.
- Added exact simultaneous four-payload schedule `[10 10 10 10]` with one-sample `cfg0 -> cfg4` transition.
- Generalized mission logic from strictly increasing drop times to nondecreasing times.
- Added `DropCount`, `FromCfgEvent`, `ToCfgEvent` so multiple payloads can be released in one MPC sample without faking intermediate time steps.
- Changed cargo-mass validation to release-group mass validation; simultaneous release verifies the complete group mass jump.
- Preserved per-cargo observed mass only when a payload is individually time-resolved.
- Fixed event metrics so closely spaced events no longer use overlapping 5 s windows.
- Added explicit global peaks in h/Va/gamma/theta/q and recovery time after the final release.
- Added automatic two-scenario comparison and optional import of the existing v0.5.0 0.2 s baseline.
- Added comparison CSV/TXT/MAT/PNG outputs.
- No Q/R, scale, horizon, hard bound, Oracle, S-Function, trim, Jacobian, or acceptance threshold was changed.
