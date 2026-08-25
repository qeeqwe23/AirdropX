# GUI reconstruction / preservation notes

The Python/PyQt6 GUI is the frontend. MATLAB is a separate simulation/control backend and is not used as the GUI layer.

This v2.3 package preserves the existing three-column desktop layout and changes only the backend integration semantics:

- default backend: Physics-MPC v1.3.6-Paper;
- optional backend: Physics-MPC v1.4.0 experimental;
- free H/V and custom longitudinal wind remain GUI-layer task inputs;
- formal paper scenario names keep the original v1.3.6 code path;
- custom wind is isolated behind the `gui_custom` scenario only.

The original interface reference image is retained in this folder for visual comparison.
