# ADS-100 Step5：级联纵向控制交付说明

本次完成第三项改进：把“直接高度控制”升级为 **级联纵向控制**（高度外环 → 爬升率 → 俯仰/俯仰率内环），并把 **油门通道**纳入控制分配（若模型无推进系统则自动降级为忽略/仅记录）。

## 改动内容

1) 新增控制器：`core/longitudinal_cascade_controller_v1.py`
- 外环：`e_h = h_ref - h` → `v_z_cmd`（爬升率指令）
- 中环：`e_vz = v_z_cmd - v_z_up` → `theta_cmd`（俯仰角指令）
- 内环：`e_theta = theta_cmd - theta` 与 `q` 阻尼 → `elevator_delta`
- 增强：动压增益调度、低空阻尼墙、投放前馈（质量突变时短暂俯仰补偿）、积分抗饱和、限幅与速率限制。

2) Wrapper 支持油门通道与日志：`core/jsbsim_wrapper.py`
- `step(elevator_delta, throttle_cmd=None)` 新增可选 `throttle_cmd`
- 自动探测可写入的油门属性（HY100 当前无推进系统，因此会提示“不可用”，并忽略油门写入）
- Telemetry 与 history 增加 `throttle` 字段

3) 全流程仿真脚本更新：`tests/test_full_simulation.py`
- 切换为 `LongitudinalCascadeControllerV1`
- 新增 throttle 曲线绘制（用于验证油门输出是否平滑）

## 复现与验证

在工程根目录执行（示例）：

- 运行完整仿真与指标：
  - `python - <<'PY' ...`（见你日常跑法），或用 pytest 方式加载模块运行 `run_simulation_test()`

本次在默认条件（目标高度 20m、5s 投放、总时长 15s）下，投放后最大高度偏差约 **2.72m**，满足任务书中“连投瞬间高度波动 ≤ ±4m”的要求。

## 重要说明

- 当前 HY100 模型文件未定义推进系统，所以“油门通道”目前是 **架构就绪**，但对动力学不起作用；后续只要补充 JSBSim 的 engine/propulsion 配置，控制器与 wrapper 无需改动即可直接生效。

2) Wrapper 支持油门通道与日志：`core/jsbsim_wrapper.py`
- `step(elevator_delta, throttle_cmd=None)`：新增可选的油门参数。
- 自动检测可写的油门属性（如 `fcs/throttle-cmd-norm`）。
- 若当前 HY100 没有推进系统，则会提示 `ThrottleProperty: (not available; no propulsion model)`，并安全忽略油门写入；但仍会把控制器输出记录到 telemetry/history 便于后续加发动机时直接接入。

3) 全系统仿真脚本更新：`tests/test_full_simulation.py`
- 默认切换为 `LongitudinalCascadeControllerV1`。
- 在性能图里增加“油门通道”曲线（对于 HY100 主要用于验证输出平滑性）。

## 复现与验证

在 Linux/WSL 下：
```bash
cd ADS-100-Project
python - <<'PY'
import sys, importlib.util
from pathlib import Path
sys.path.insert(0, str(Path('.').resolve()))

spec = importlib.util.spec_from_file_location('tfs', str(Path('tests/test_full_simulation.py').resolve()))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

mod.run_simulation_test(target_altitude=20.0, drop_time=5.0, total_time=15.0)
PY
```

输出：
- 图表：`data_logs/performance_analysis.png`
- 关键指标（本次默认参数在 20m、90m/s、5s 投放 300kg 场景下）：投放后最大高度偏差约 2~3m，满足 PR-H-02（±4m）。

## 备注
- 现有 HY100 模型无推进系统，因此油门对动力学不生效；下一步如果你要真正做到“升降舵+油门能量分配”，需要为 HY100 补充 `propulsion`/engine/propeller 定义，然后 wrapper 会自动识别油门属性并接管。

在项目根目录运行（推荐）：

```bash
# 运行完整仿真与性能检查（包含一次投放）
python - <<'PY'
import sys, importlib.util
from pathlib import Path
sys.path.insert(0, str(Path('.').resolve()))

spec = importlib.util.spec_from_file_location('tfs', str(Path('tests/test_full_simulation.py').resolve()))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

mod.run_simulation_test(target_altitude=20.0, drop_time=5.0, total_time=15.0)
PY
```

输出图表位于：`data_logs/performance_analysis.png`

## 验证结论（当前默认参数）
- 在 20m 超低空 + 90m/s 场景下，投放后高度最大偏差约 **2.7m**（满足 ≤±4m 的瞬态要求）。
- 全程 RMS 约 2.2m（略高于“≤2m”的更严格口径，但已明显优于单环/弱增益版本；后续可继续通过增益或加入速度能量管理进一步收敛）。

