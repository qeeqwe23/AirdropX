# AirdropX 到 MATLAB/Simulink 迁移方案

## 1. 迁移目标

本仓库当前是 Python + PyQt6 + JSBSim 架构。迁移到 Simulink 时，不建议照搬 UI，而是迁移闭环仿真链路：

```text
任务参数/风场/目标点
        ↓
控制器 PD 或 ADRC
        ↓
JSBSim/MQ-9 Plant
        ↓
遥测 h, vz, vtrue, qbar, mass, position
        ↓
CARP 释放点求解 + 投放状态机
        ↓
质量/重心更新 + 货物弹道/CEP 评估
```

## 2. 原仓库模块映射

| 功能 | Python 文件 | Simulink 建议实现 |
| --- | --- | --- |
| 主循环 | `ui/simulation_thread.py` | 顶层 Simulink 模型，固定步长 1/120 s |
| 飞行动力学 | `core/jsbsim_wrapper.py` | 第一阶段用 Python/C++ 联仿，后续可替换为 Aerospace Blockset 6DOF |
| PD 控制 | `core/pd_baseline_controller.py` | MATLAB Function Block |
| ADRC V17 | `core/longitudinal_adrc_v17.py`, `core/adrc_controller_v17.py` | MATLAB Function Block 或 Stateflow + MATLAB Function |
| CARP 释放点 | `core/carp_release_solver_v2.py` | MATLAB Function Block |
| 货物弹道 | `core/cargo_trajectory.py` | MATLAB Function Block 或离线验证脚本 |
| 质量/重心 | `core/mass_cg_manager.py` | Data Store Memory + MATLAB Function |
| CEP/指标 | `core/impact_metrics.py` | MATLAB 脚本或 Simulink Test |

## 3. 第一阶段：最快跑通闭环

第一版目标是复现仓库的 20 m 低空投放闭环，不追求纯 Simulink。

建议模型名：

```text
airdropx_mq9_20m.slx
```

模型设置：

- Solver: Fixed-step
- Step size: `1/120`
- Stop time: 30 到 120 s
- 坐标: North/East/Down 或 North/East/Altitude，内部保持 SI 单位
- 舵面约定: 正 elevator_delta 表示低头，负值表示抬头，保持与 MQ-9 Python 代码一致

顶层子系统：

```text
MissionConfig
WindModel
Controller_PD
Plant_JSBSim
CARP_ReleaseLogic
DropStateMachine
MassCG
CargoTrajectory
Metrics
```

第一阶段 plant 可以通过 MATLAB System block 或 S-Function 调用 JSBSim。控制器、CARP、质量重心先使用 `matlab/` 目录下的 `.m` 函数。

## 4. MATLAB 函数骨架

已添加：

- `matlab/airdropx_carp_release_point.m`
- `matlab/airdropx_pd_controller.m`
- `matlab/airdropx_mass_cg_update.m`
- `matlab/airdropx_cargo_trajectory_point.m`

这些函数对应 Python 中的纯算法部分，可以先在 MATLAB 命令行验证：

```matlab
addpath("matlab")
r = airdropx_carp_release_point(0, 200, 20, 78.6, 0, 0, 0, 0, []);
disp(r.release_n_m)

state = [];
gains = [];
[de, th, state, diag] = airdropx_pd_controller(20, 20.5, 0.1, 78.6, 0, 3423, state, gains);
disp([de, th])

[m, cg, onboard] = airdropx_mass_cg_update(1, true, []);
disp([m, cg])
```

## 5. 投放状态机建议

用 Stateflow 做 4 个货物的投放序列：

```text
Idle
  -> Armed
  -> WaitReleaseWindow
  -> Drop1
  -> WaitInterval
  -> Drop2
  -> WaitInterval
  -> Drop3
  -> WaitInterval
  -> Drop4
  -> Complete
```

触发条件：

- `abs(current_n - release_n) <= release_window_m`
- `altitude >= min_safe_alt_m`
- `abs(v_z_up) <= stable_vz_mps`
- `drop_count < drop_total`

对于 NW20 高度稳定基线，可不用 CARP，直接使用固定投放时刻，例如 10 s 起投、间隔 0.5 或 0.6 s。

## 6. JSBSim 联仿方案

优先级如下：

1. MATLAB System block 调 Python `JSBSimWrapper`
   - 优点: 改动最小，最快验证。
   - 缺点: 不适合代码生成，调试速度一般。

2. C++ S-Function 调 JSBSim C++ API
   - 优点: Simulink 工程化更稳，速度好。
   - 缺点: 需要配置 JSBSim 头文件、库文件和 aircraft 路径。

3. Aerospace Blockset 6DOF 重建 MQ-9
   - 优点: 纯 Simulink，可扩展到控制设计和代码生成。
   - 缺点: 需要从 `aircraft/MQ9_Reaper/*.xml` 导入/重建气动、质量、发动机模型，工作量最大。

### 6.1 C++ S-Function 工程路线

本仓库已添加 C++ S-Function 骨架：

```text
matlab/sfunc_jsbsim/
  sfun_airdropx_jsbsim.cpp
  build_sfun_airdropx_jsbsim.m
  README.md
```

推荐工程步骤：

1. 在 Windows 上编译或安装 JSBSim C++，位数必须与 MATLAB 一致，通常都是 x64。
2. 确认有 JSBSim 头文件和库文件：
   - `include/FGFDMExec.h` 或 `include/JSBSim/FGFDMExec.h`
   - `lib/jsbsim.lib`、`libJSBSim.lib` 或同等 import library
3. 在 MATLAB 中设置 C++ 编译器：

```matlab
mex -setup C++
```

4. 编译 S-Function：

```matlab
cd matlab/sfunc_jsbsim
build_sfun_airdropx_jsbsim("C:\path\to\jsbsim\install")
```

5. 在 Simulink 中放入 S-Function block，函数名写：

```matlab
sfun_airdropx_jsbsim
```

参数写：

```matlab
projectRoot, aircraftName, icName, dt
```

典型参数：

```matlab
projectRoot = "path/to/AirdropX";
aircraftName = "MQ9_Reaper";
icName = fullfile(projectRoot, "aircraft", "MQ9_Reaper", "reset_20m");
dt = 1/120;
```

6. S-Function 输入向量宽度为 6：

```text
[elevator_delta, throttle_cmd, wind_speed_mps, wind_dir_from_deg, drop_cmd, reset_cmd]
```

7. S-Function 输出向量宽度为 20，建议接 Demux 或 Bus Creator：

```text
time, altitude_m, vz_up_mps, airspeed_mps, groundspeed_mps,
pitch_deg, roll_deg, heading_deg, qbar_pa, mass_kg, cg_x_m,
pos_n_m, pos_e_m, elevator_cmd_norm, throttle_norm,
wind_n_mps, wind_e_mps, drop_count, valid, reserved
```

第一版闭环建议：

```text
S-Function(JSBSim Plant)
    -> Demux/Bus
    -> PD Controller MATLAB Function
    -> Mux input vector
    -> S-Function(JSBSim Plant)
```

跑通后再把 `CARP_ReleaseLogic` 和 `DropStateMachine` 接进去。

## 7. ADRC 迁移顺序

ADRC V17 状态较多，建议等 PD 闭环跑通后再迁移：

1. 先迁移 `fal`, `fhan`, ESO 状态更新。
2. 再迁移前馈 `ff_state` 和质量突变补偿。
3. 再加入爬升抑制、低空保护、速率限制。
4. 最后和 Python 输出逐步对齐。

ADRC MATLAB Function Block 需要持久化这些状态：

```text
v1, v2, z1, z2, z3, u_prev, sum_e, ff_state,
climb_suppress_active, max_altitude_error, min_altitude
```

## 8. 验证指标

建议先对齐仓库中的验收指标：

- 初始高度: 20 m 附近
- 初始空速: 约 78.6 m/s
- 油门 trim: 0.80
- 高度保持 p95: 小于 3 m，目标小于 1.5 m
- 4 连投瞬态高度波动: 小于 4 m
- CEP50: 小于 20 m

## 9. 推荐实施顺序

1. 在 MATLAB 中单独跑通 `matlab/` 下的函数。
2. 搭 `Controller_PD + MassCG + CARP` 子系统。
3. 用简化 plant 或 Python JSBSim 联仿接通闭环。
4. 用 Python 仓库的 `data_logs` 结果对比高度、投放时刻、释放点和落点。
5. 再迁移 ADRC。
6. 如果需要工程交付，再把 JSBSim plant 从 Python 改为 C++ S-Function 或纯 Simulink 6DOF。
