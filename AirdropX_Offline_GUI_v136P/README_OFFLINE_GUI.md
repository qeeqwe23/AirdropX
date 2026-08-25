# AirdropX Offline GUI — v1.3.6-Paper 默认后端

这个版本保留原 PyQt6 GUI 的布局和使用方式，但把默认控制后端从尚未通过全部 formal gate 的 v1.4.0 切换为 **Physics-MPC v1.3.6-Paper**。

## 后端层次

- **默认：Physics-MPC v1.3.6-Paper**
  - 使用 `airdropx_wind_airdrop_mission_v136p.m`。
  - 保留经过验证的 v1.3.6 carrier / wind / release 控制逻辑。
  - 使用论文公平传感器路径：无偏白噪声 GNSS/INS、Baro、Pitot、AHRS/IMU、发动机遥测。
  - GUI 显示 `Paper Core` 与 `Engineering Gate` 两套状态，避免把严格工程 gate FAIL 误认为软件运行失败。
- **可选实验：Physics-MPC v1.4.0**
  - 仅当项目中仍存在 v1.4.0 mission 文件时可选。
  - 继续作为实验后端，不作为软件默认论文控制器。

## GUI 保留/新增

- 原三栏式界面结构、实时曲线、投放状态、日志与精度显示。
- 自由设置目标高度 H 与目标速度 V。
- 自定义风：无风、恒定、阶跃、双向阶跃、Ramp、正弦、确定性复合阵风。
- 八个 v1.3.6-Paper 论文风场预设保持原正式代码路径。
- 任意风向先投影为沿航迹分量；当前纵向 MPC 不伪装处理横风控制。
- 任意 H/V 使用精确 Physics Bank + Wind Disturbance 模型；不存在时可自动生成并缓存。

## 安装

在 PowerShell：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\install_into_airdropx.ps1 -ProjectRoot 'D:\vscode project\AirdropX'
```

安装脚本会先调用 v1.3.6-Paper 官方安装器，其公共 v1.3.6 文件与 v1.4.0 包中的对应公共文件保持一致；新增加的 `v136p` 文件独立存在。被替换文件仍按官方逻辑备份到 `matlab\_backup_v136p_*`。

## 启动

```powershell
cd 'D:\vscode project\AirdropX\offline_gui_v136p'
.\launch_gui.bat
```

启动器依次尝试：

1. `offline_gui_v136p\.venv\Scripts\python.exe`
2. 已存在的 `offline_gui_v140\.venv\Scripts\python.exe`
3. 已知 Python 3.12 安装位置
4. PATH 中的 `python`

如果需要为新目录单独创建环境：

```powershell
.\install_python_env.ps1
```

## 结果解释

GUI 会分开显示：

- `软件执行: SUCCESS/FAIL`：GUI → MATLAB → JSBSim → MPC → CSV 链路是否正常。
- `Paper Core: PASS/FAIL`：v1.3.6-Paper 的实验完整性核心 gate。
- `Engineering Gate: PASS/FAIL`：旧的严格恢复/最终/尾段/落点压力 gate。

因此 `Engineering Gate: FAIL` 不再被显示成“软件失败”。

## 论文正式点与自由 H/V

`200 m / 50 m/s / FuelScale=1` + 论文预设风场可以按 v1.3.6-Paper 正式实验语义解释。

其他 H/V、任意自定义风属于 **GUI 工程探索工况**。即使 `Paper Core PASS`，也不应声称它已经成为论文冻结工况的正式认证结果；它表示公平传感器路径、QP、输入约束、实时性、质量配置和风估计等核心完整性条件通过。

## 自由 H/V 依赖

第一次运行新的 H/V 点时仍需要项目中已有：

- `airdropx_phys_build_bank.m`
- Physics/JSBSim Oracle MEX
- MATLAB + MPC/Optimization 相关工具箱

生成结果缓存于：

```text
matlab/results/offline_gui_v136p_model_cache/
```
