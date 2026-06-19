# AirdropX TC-NW20 无风 20m 4连投稳定性基线报告（UI 对齐修订版）

## 1. 验收边界

本报告仅对应 **TC-NW20：无风、20m、JSBSim、真实点质量、4件连投、高度稳定性基线**。本阶段不评价 CARP 释放点精度、CEP50-to-target、侧风航迹控制或 HY100 实机能力。

固定场景如下：

- 仿真引擎：JSBSim
- 代理机型：MQ-9 Reaper Cargo
- 风速：0 m/s
- 目标高度：20 m
- 控制器：PD Baseline
- 投放方式：固定时间 4 件连投
- 投放计划：10.0 s、10.6 s、11.2 s、11.8 s
- 投放载荷：4 × 300 kg
- 评价指标：高度误差、真实点质量归零、无 NaN/Inf、控制输出饱和率

## 2. 已完成的脚本层证据

`data_logs/nw20_final_v1/` 中的脚本层证据表明：

- `scripts/nw20_final_v1.py` 已使用 `sim.trigger_drop()`，会将 JSBSim FDM 中的 `inertia/pointmass-weight-lbs[i]` 写为 0。
- `drop_events.json` 与 `fdm_mass_check.json` 记录 4 个点质量由约 661.386 lb 变为 0.0 lb。
- Python 侧质量从 3423 kg 逐次降低至 2223 kg，累计减载 1200 kg。
- `summary.json` 与 `summary_recomputed.json` 可由 `telemetry.jsonl` 复算一致。

脚本层核心指标：

| 指标 | 结果 | 门限 | 结论 |
|---|---:|---:|---|
| 4件真实 JSBSim 点质量投放 | 4/4 | 4/4 | PASS |
| 最大高度偏差 | 0.4622 m | ≤ 4.0 m | PASS |
| 全程 p95 高度误差 | 0.2143 m | ≤ 2.0 m | PASS |
| 高度范围 | 19.9048–20.4622 m | 14–26 m | PASS |
| 升降舵饱和率 | 0.0% | ≤ 5% | PASS |
| NaN/Inf | 未出现 | 不允许 | PASS |

因此，**独立脚本层 NW20 高度稳定基线可判定为通过**。

## 3. 本次 UI 对齐修订

上一版 UI 截图仍混入 CARP/CEP 评价，且显示当前高度 26.1 m、最大高度偏差 6.21 m，与脚本层 `nw20_final_v1` 结果不一致。为解决该问题，本次修订增加了 UI 专用 NW20 模式：

- 新增任务模式：`NW20高度稳定4连投`。
- 该模式强制使用：0 m/s 风、20 m 目标高度、PD Baseline、固定时间 4 连投。
- 该模式关闭 CARP/CEP 评价，UI 显示“本模式不考核 CEP”。
- `SimulationThread.trigger_drop()` 在 NW20 模式下直接启动固定时间投放，不等待 CARP 窗口。
- 每次启动仿真时清空旧曲线、旧最大高度偏差和旧 CEP，避免 UI 缓存污染结论。
- UI 控制器面板直接显示 `Kp/Kd/u_limit/K_mass`，不再用 `omega_o/omega_c/b0_base` 伪装 PD 参数。

## 4. 当前结论

当前可以正式声明：

> AirdropX 在独立脚本层面已经完成“无风、20m、JSBSim真实点质量、4件连投、高度稳定”基线验收。

本次代码修订的目标是让附件界面与脚本层验收完全对齐。由于当前审计环境无法加载 cp311 的 JSBSim wheel，UI 层仍需在 Python 3.11 + JSBSim 运行环境中重新截图验证。

## 5. 后续验收要求

UI 层重新验收时，必须生成 `data_logs/nw20_ui_v1/`，至少包含：

- `summary.json`
- `summary_recomputed.json`
- `telemetry.jsonl`
- `drop_events.json`
- `fdm_mass_check.json`
- `screenshots/`
- `hashes.json`

UI 截图必须显示：

- 任务模式：NW20高度稳定4连投
- 仿真引擎：JSBSim
- 控制器：PD Baseline
- 风速：0 m/s
- 投放计数：4/4
- 高度曲线：4 个 DROP marker
- CEP 标签：本模式不考核
- 最大高度偏差：≤ 4 m
- 结论：NW20高度稳定合格

