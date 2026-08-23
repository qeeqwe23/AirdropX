# CHANGELOG v0.7.0

- Accepted the uploaded v0.6.1 altitude sweep only because all 19 H=20:10:200 m nonlinear missions passed every formal gate.
- Added a speed pilot at Va=[40 45 50 55 60] m/s and altitude anchors H=[20 110 200] m.
- Added four isolated physics-bank slice builds at V40/V45/V55/V60; V50 is reused exactly from the certified v0.3.3 95-point bank.
- Added merge/audit into one exact 75-vertex speed pilot bank.
- Added 15 isolated nonlinear PreviewOnly four-drop missions.
- Kept the original 0.2 s four-drop schedule and all v0.6.0 gates unchanged.
- Kept q-soft OFF.
- Added resume support, PID-scoped teardown watchdogs, per-speed summaries, and full worst-case H/V reporting.
- No MEX or C++ payloads are included and no existing physics bank is overwritten.
