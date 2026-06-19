# NW20 UI 对齐补丁说明

## 修改目标

将 UI 从“CARP/CEP 混合演示模式”中解耦出一个明确的 `NW20高度稳定4连投` 模式，使界面运行逻辑与 `scripts/nw20_final_v1.py` 的脚本层验收保持一致。

## 修改内容

1. `ui/config_panel.py`
   - 新增“任务模式”下拉框，默认 `NW20高度稳定4连投`。
   - NW20 模式强制配置：`mission_mode=nw20_height_4drop`、`evaluate_cep=false`、`fixed_drop_schedule=true`、`target_altitude=20`、`wind_speed=0`、`drop_total=4`、`interval=0.6`、`sim_duration_s=60`。
   - 控制器面板改为直接显示 PD 参数：`Kp=0.025`、`Kd=0.080`、`u_limit=0.15`、`K_mass=0.15`。

2. `ui/simulation_thread.py`
   - 新增 `mission_mode/evaluate_cep/fixed_drop_schedule/fixed_drop_start_s`。
   - NW20 模式下 4 连投不再等待 CARP 窗口，而是在固定时间表下执行真实 `sim.trigger_drop()`。
   - NW20 模式下跳过蒙特卡洛散点与 CEP 评价。
   - Telemetry 中输出 `mission_mode/controller_mode/evaluate_cep/target_n_m`，供 UI 判定当前评价口径。

3. `ui/analysis_panel.py`
   - NW20 模式下 CEP 标签显示“本模式不考核”。
   - NW20 模式下结论由高度最大偏差与 4/4 投放完成决定，不再依赖 settling_time 或 CEP。

4. `ui/main_window.py`
   - 每次启动新仿真时清空旧曲线、旧最大高度偏差、旧 CEP 与旧日志，避免缓存污染。
   - 自动模式下设置 PD Baseline 参数为脚本验收值。
   - NW20 模式下 CARP 倒计时区域显示“固定时间4连投；本模式不评价CARP/CEP”。

5. `core/pd_baseline_controller.py`
   - 默认增益更新为脚本验收参数：`Kp=0.025`、`Kd=0.080`、`u_limit=0.15`、`K_mass=0.15`。

6. `profiles/nw20_height_4drop_ui_profile.json`
   - 新增 UI 专用 NW20 验收 profile。

## 不能在当前环境完成的事项

当前执行环境为 Python 3.13，而离线包内 JSBSim wheel 为 cp311，因此无法直接复跑 JSBSim。补丁已通过 Python 语法编译检查；真实仿真需在 Python 3.11 + JSBSim 环境中重新运行。

