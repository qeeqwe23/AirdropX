# AirdropX Standalone v2.0 RC1

AirdropX Standalone 是从已跑通的 v1.3.6-Paper 软件基线迁移出来的 **独立 JSBSim 桌面应用**。

最终发布目录的运行链路是：

```text
AirdropX.exe
  -> PyQt6 GUI
  -> Physics-MPC v1.3.6-Paper standalone numerical runtime
  -> JSBSim 1.3.1 Python/native runtime
  -> MQ9_Reaper + 4 × 300 kg cargo
```

**运行时不启动 MATLAB、不调用 MEX、不读取 `.mat`。**

MATLAB 在这里仅是历史研发/验证工具：第一次 Windows 构建时，脚本会从现有研究工程中读取已经生成好的 Physics-MPC `physics_bank.mat`，把经过验证区间所需的数值模型一次性转换成 `controller_bank.npz`。构建完成后，`dist\AirdropX` 不再需要 MATLAB、原 AirdropX 工程或 Python 安装。

## 1. 高度/速度只允许已验证连续包线

软件硬限制：

- 高度 `20–200 m`
- 空速 `45–65 m/s`

GUI 保留自由输入，但输入控件自身有限位，后端还有第二层校验。任何越界任务都不能启动。

运行时也不再“现场训练/生成未知模型”。构建器要求完整存在：

```text
H = 20:10:200 m
V = 45, 50, 55, 60, 65 m/s
cfg = 0,1,2,3,4
```

共 `19 × 5 × 5 = 475` 个锚点，而且现在检查的是**完整笛卡尔组合**，不是只看总数够不够。

### 区间内自定义 H/V 的调度规则

Standalone 按已经验证过的连续 H×V 思路运行：

1. 从四个相邻 H×V 锚点双线性插值 `A / B / xtrim / utrim`；
2. `Q / R / StateScale / InputScale` 保持统一，不做 cfg/工况单独调参；
3. 对插值后的 `A / B` 重新求离散 DARE，得到 terminal `P / K`；
4. 固定 `Np = Nc = 100`。

因此 `137 m / 52.5 m/s` 这类区间内自定义任务可以运行，但 `201 m`、`44 m/s` 等会被硬拒绝。

`200 m / 50 m/s` 在界面中标记为“论文基准点”；其他区间内任务标记为“已验证 H×V 连续包线”。这不等于宣称所有 H/V × 所有风场都已经逐点重新做过 v1.3.6-Paper formal certification。

## 2. v1.3.6-Paper 控制逻辑

Standalone 不是重新发明一套 MPC。现有 Paper 行为按原源码关系移植，包括：

- 7 状态、2 输入 Physics-MPC；
- `Np=Nc=100`；
- cfg0–cfg4 载荷调度；
- v1.3.6-Paper 无偏白噪声传感器/因果状态估计；
- v1.1.1 两状态因果纵向风估计；
- v1.3.2 wind-confidence；
- **绝对风证据用于投放，瞬态风证据用于载机 disturbance MPC**；
- 恒定非零风稳定后回到基础 MPC，不让 disturbance QP 永久在线；
- v1.3.0 delta-wind `Gw` 预览；
- 因果 residual disturbance observer；
- 原 Paper 的事件门控 gust-recovery cost bank；
- 原 Paper 的 energy-aware second pass；
- cfg 切换 warm-start rebase；
- v1.3.1 grid-aligned fractional release；
- 4 连投与风补偿弹道预测。

没有加入为了 `tail5s_normalized_rms=0.0650` 单独设计的新补丁，也没有提高 actuator 限制。

正式 Paper 风场继续使用固定 seed：

```text
calm                    101
tailwind_5              102
headwind_5              103
tailwind_12             104
headwind_12             105
step_bidirectional      106
ramp_minus10_plus10     107
sine_longitudinal       108
```

## 3. 实时轨迹是主视觉

中央最大区域显示实时：

- 飞机沿航迹距离 `x` — 高度 `H` 轨迹；
- 当前飞机位置；
- 4 个空投目标；
- 已释放货物的下落轨迹与落点；
- 当前时间、cfg、高度、空速、Pitch、真实/估计纵向风。

高度、空速、升降舵等时间曲线只放在下方辅助区域，不再抢占主画面。

## 4. 风场

保留：

- 无风
- 恒定风
- 阶跃风
- 双向阶跃
- Ramp
- 正弦阵风
- 复合阵风/湍流
- 8 个论文验证预设

当前控制器是纵向 Physics-MPC。自定义“风向”会投影为沿航迹风分量；横风分量不会被冒充为已经进入纵向控制器。

## 5. Windows 构建

推荐直接把本目录放到 AirdropX 工程根目录附近，然后双击：

```text
Build_Standalone.bat
```

或者：

```powershell
.\Build_Standalone.ps1 -ProjectRoot "D:\vscode project\AirdropX"
```

构建器会：

1. 优先寻找已有 Python 3.12（包括原 GUI `.venv`），避开 Windows Store 假 `python.exe`；
2. 创建 `.venv_standalone`；
3. **强制验证 Python 必须是 3.12**；
4. 安装固定的 `jsbsim==1.3.1` 以及 PyQt6 / NumPy / SciPy / h5py / PyInstaller；
5. 搜索完整 475 锚点 controller bank；
6. 只导出验证区间需要的锚点到 `assets\controller\controller_bank.npz`；
7. 复制 MQ9_Reaper / engine / systems 到独立 JSBSim 资产目录；
8. 先执行 `compileall`；
9. 执行全部 Python 回归测试；
10. 执行真实 `JSBSim + MPC` smoke；
11. **只有前面全部 PASS 才运行 PyInstaller**。

外部命令的 `$LASTEXITCODE` 在每次调用后立即保存并判断；PyInstaller 的 dist/build/spec 使用脚本自身绝对路径，不依赖当前 PowerShell 工作目录。

## 6. 真实 JSBSim smoke 的发布门槛

构建器不会用闭环 MPC 掩盖一个错误的 JSBSim 初始化。冻结 EXE 之前必须同时通过：

- `200 m / 50 m/s / cfg0` 初始状态与保存 trim 的归一化偏差门槛；
- trim input 开环保持 1 s；
- 零风 MPC QP 1 s；
- 顺风 `Gw(Va)` 符号/有限性；
- cfg0 → cfg1 载荷变化约 `300 kg`。

预计成功标志：

```text
STANDALONE_RUNTIME_SMOKE_PASS
...
BUILD PASS
Standalone EXE: ...\dist\AirdropX\AirdropX.exe
```

如果 Python JSBSim binding 与原 C++ Oracle 的初始化/发动机/point-mass 语义不一致，构建会直接停止，不会生成“假成功” EXE。

## 7. 启动

构建成功后：

```text
AirdropX.bat
```

或者直接：

```text
dist\AirdropX\AirdropX.exe
```

真正需要复制到其他 Windows 电脑的是整个：

```text
dist\AirdropX\
```

目标是该目录离开源码工程后仍可运行，且不需要 MATLAB、Python 或原 AirdropX 工程。

## 8. 结果目录

每次任务写入：

```text
Documents\AirdropX\results\<timestamp>\
  mission_config.json
  timeseries.csv
  cargo.csv
  summary.json
```

CSV 使用 UTF-8 BOM，JSON 使用 UTF-8，避免中文乱码。

## 9. RC1 的诚实边界

当前生成环境可以做 Python 编译、数值回归和模型导出一致性检查，但没有这里所需的 Windows PyQt6/JSBSim runtime，因此不能在这里声称新的 Windows EXE 已经真实跑通。

本包因此标为 `2.0.0-rc1`。在项目 Windows 电脑上执行 `Build_Standalone.bat` 后，脚本会先真实运行 JSBSim smoke，再允许生成 EXE。生成后建议第一条完整任务仍用：

```text
200 m / 50 m/s / 论文-无风
```

用它与已经保存的 v1.3.6-Paper 软件基线做迁移等价性检查，再冻结为 v2.0 正式发布版。
