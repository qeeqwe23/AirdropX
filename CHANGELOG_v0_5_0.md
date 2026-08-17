# v0.5.0

- Added four sequential JSBSim cargo point-mass transitions cfg0->cfg4.
- Added common-controller audit proving Q/R/scales/horizon are unified across cfg.
- Added QP self-test for all five cfg models before mission start.
- Added warm-start rebasing across changing trim inputs; this changes only solver initialization, not the control law.
- Added runtime mass/CG/Iyy recording and cargo-mass transition audit.
- Added per-drop 5 s peak metrics.
- Added switched-system-appropriate gates; deliberately removed any claim that a cfg-local P is a common Lyapunov function.
- Added mission plots and complete failure evidence.
- No changes to v0.3.3 Oracle C++, MEX, S-Function MEX, physics bank, Q/R, or certification tolerances.
