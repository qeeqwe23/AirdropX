# v1.4.0 → v1.3.6-Paper GUI 后端切换说明

1. Python GUI 与 MATLAB 控制器继续保持分层，GUI 不直接改写 MPC 内部参数。
2. 默认入口改为 `airdropx_gui_backend_entry_v136p.m`。
3. 正式任务调用 `airdropx_wind_airdrop_mission_v136p.m`。
4. `gui_custom` 仅在 `airdropx_wind_profile_v136.m` 最前方增加隔离分支；八个正式 scenario 的原 v1.3.6 代码路径保持不变。
5. v1.3.6-Paper 的 `report.pass` 实际代表 `paper_core.pass`；旧严格 gate 单独存为 `engineering_pass`。GUI 已按这两个语义分别显示。
6. v1.4.0 不删除，作为可选实验后端保留。
