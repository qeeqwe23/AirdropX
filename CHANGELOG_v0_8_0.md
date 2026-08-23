# CHANGELOG — Physics-MPC v0.8.0

- Formal speed range changed from pilot 40–60 m/s to requested **45–65 m/s**.
- Removed V40 from the formal target envelope; prior V40 boundary result is not relabeled as fixed.
- Replaced 15-mission speed pilot with complete `19 heights × 5 speeds = 95` nonlinear mission certification.
- Replaced 75-vertex pilot bank with complete `19 × 5 × 5 = 475` H/V/cfg physics bank.
- Reuses the certified 95-point V50 bank; builds complete V45/V55/V60/V65 slices.
- Adds full 475-point common-Q/R/scales/horizon and unique-grid audit.
- Adds per-speed 19-height PASS summaries and per-height 5-speed PASS summaries.
- Adds full failure map and H×V heatmap output.
- Preserves PreviewOnly, q-soft OFF, 0.2 s four-drop schedule, all control gates, and PID-scoped process isolation.
- Batch no longer stops on a mission performance FAIL; all 95 missions are collected before final judgment.
