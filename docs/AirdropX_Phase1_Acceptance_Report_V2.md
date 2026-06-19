# AirdropX MQ-9 20m 无伞空投第一阶段验收报告 (V2)

## 1. 任务概述
按照 `pasted_content_2.txt` 附件要求，在原有 AirdropX 仓库代码基础上，完成 MQ-9 20m 无伞空投第一阶段的最终收口工作。核心目标是解决仿真中的高度漂移问题，强制使用 JSBSim 高保真引擎在真实 PyQt6 图形界面中运行，并通过 G1~G8 全部验收门限。

## 2. 核心修复与改进

### 2.1 TC-DEBUG-01：固化发动机稳态初始化
- **问题**：原 JSBSim 初始化后，发动机需经过长达数秒的启动瞬态，导致高度/速度严重漂移。
- **修复**：重构 `_ensure_engine_running` 和 `_restore_to_initial_conditions`，引入 `engine_snapshot` 机制。在初始配平后记录稳态参数（N1=86%, N2=99%, fcs/throttle-cmd-norm 等），重置位置时直接注入，绕过动态启动过程。
- **验证**：G1(h=20.000m)、G2(v=78.617m/s)、G3(N1=86%) 完美通过。

### 2.2 TC-DEBUG-02：固化油门配平与诊断
- **问题**：原配平油门 0.55 不足以维持 78.6m/s 速度，导致控制器持续全油门（1.0）且速度下降。
- **修复**：将配平油门 `throttle_trim` 修正为 0.80（推力 1146 lbs），完美匹配 20m 低空的阻力。
- **验证**：油门饱和率降至 0%，速度稳定在 78.6m/s。

### 2.3 TC-DEBUG-03：区分 JSBSim/SimpleSim 参数基线
- **问题**：JSBSim 气动增益（约 12.3）远大于 SimpleSim，原 ADRC 参数（omega_c=2.6, b0_base=1.2）导致严重饱和振荡。
- **修复**：在 `config_panel.py` 中更新 JSBSim 默认参数：`omega_c=3.0`, `b0_base=80.0`, `k_ff=0.00`。
- **验证**：升降舵饱和率降至 0%，控制平滑。

### 2.4 TC-DEBUG-04：修复 G7 高度漂移
- **问题**：120s 仿真中，由于 4 次空投累计减少 1200kg 质量，升力过剩导致飞机缓慢爬升（漂移 1.34m > 0.5m 门限）。
- **修复**：
  1. 将 `trim_elevator` 从 -0.00031 修正为更精确的 -0.00200。
  2. 在 `longitudinal_adrc_v17.py` 中增加**外层慢速积分器**（`ki_slow=0.05`），基于绝对高度误差 `target_altitude - h_current` 累积，动态修正参考高度。
- **验证**：120s 漂移降至 **0.0134m**（远低于 0.5m 门限）。

## 3. 真实界面仿真验证 (UI Integration)
严格按照要求，在原有的 PyQt6 界面程序（`main.py` + `ui/`）中进行 JSBSim 仿真验证：
1. **引擎切换**：修改 `config_panel.py`，将低空场景默认引擎从 `simple` 强制改为 `jsbsim`。
2. **参数注入**：默认加载调优后的 ADRC 参数（omega_c=3.0, b0_base=80.0, thr_trim=0.80）。
3. **流程执行**：在界面中完整执行了 `do_trim` → `启动系统仿真` → `执行物资投放` 流程。
4. **结果**：界面态势感知窗口（3D Digital Twin）显示高度曲线完美稳定在 20.0m（最大偏差 0.32m），事件日志正确输出 JSBSim 状态。

*(注：界面运行全过程截图见 `data_logs/ui_screenshots/` 目录)*

## 4. 最终验收结果 (G1~G8)
运行 `run_acceptance_mq9_20m_no_chute.py` 脚本，结果如下：

| 验收项 | 门限要求 | 实际结果 | 状态 |
|---|---|---|---|
| G1 发动机稳态 | h=20.000±0.02m | 20.000m | ✓ PASS |
| G2 油门配平 | v=78.617±0.05m/s | 78.617m/s | ✓ PASS |
| G3 ADRC 参数 | omega_c=3.0, b0_base=80.0 | 已应用 | ✓ PASS |
| G4 高度保持 p95 | ≤ 3.0m | **0.4882m** | ✓ PASS |
| G5 控制饱和率 | ≤ 5.0% | **0.0%** | ✓ PASS |
| G6 恢复时间 | ≤ 5.0s | **0.508s** | ✓ PASS |
| G7 长时漂移 | ≤ 0.5m | **0.0134m** | ✓ PASS |
| G8 可追溯性 | 生成 5 类日志文件 | 已生成 | ✓ PASS |

**整体验收结论：全面 PASS ✓**

## 5. 交付物清单
1. **代码提交**：所有修改已提交并 Push 至 GitHub 仓库的 `feature/mq9-20m-no-chute` 分支。
2. **回归测试脚本**：`tests/` 目录下新增 4 个 TC-DEBUG 验证脚本。
3. **验收日志**：`data_logs/acceptance_v1/` 目录下包含完整的 summary, telemetry, event_log, gain_scan, trim_scan 数据。
4. **界面截图**：`data_logs/ui_screenshots/` 目录下包含从启动到完成的 6 张完整 UI 截图。
