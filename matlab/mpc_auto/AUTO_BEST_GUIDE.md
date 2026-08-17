# AirdropX 全自动 MPC 寻优

目标：高度和速度必须达到验收线；pitch 不跟踪人工指定角度，而是每个载荷配置自动寻找配平角，并要求 pitch 稳定、q 小、漂移小。

## 一条命令运行

在项目根目录 MATLAB Command Window：

```matlab
addpath("matlab");
addpath("matlab/mpc");
addpath("matlab/mpc_auto");

r = airdropx_auto_full_auto( ...
    "DataRoot", "matlab/results/mpc_auto_train_trimmed_r1/data", ...
    "OutputRoot", "matlab/results/mpc_auto_full_auto_best", ...
    "TargetAltitudeM", 20.0, ...
    "TargetAirspeedMps", 50.0, ...
    "Orders", 3:8, ...
    "RefineWithSsest", false, ...
    "TrimMaxObjectiveEvaluations", 30, ...
    "BatchEvaluations", 15, ...
    "MaxTotalEvaluations", 90, ...
    "MaxSteadyAltitudeRmsM", 1.0, ...
    "MaxSteadyAirspeedRmsMps", 1.0, ...
    "MaxSteadyVzRmsMps", 0.70, ...
    "MaxSteadyQRmsDps", 1.0, ...
    "MaxPitchStdDeg", 0.75, ...
    "MaxPitchDriftDegps", 0.12, ...
    "MinAltitudeM", 15.0);
```

## 三阶段

1. `airdropx_auto_find_trim`：对 config 0..4 自动搜索 elevator、throttle、pitch0。pitch 没有人工目标，只要求高度/速度稳定、vz 和 q 小。
2. 使用现有 trimmed CSV 重建偏差数据，并用 `n4sid` 进行 3..8 阶选阶。默认不运行 `ssest`。
3. `airdropx_auto_tune_until_best`：真实 JSBSim 闭环 Bayesian Optimization。高度、速度、vz、q、pitch 标准差、pitch 漂移、最低高度作为 coupled constraints，只有全部通过才是 feasible controller。

## 自动停止

默认每批 15 个候选。达到全部硬指标以后，如果连续 2 批最佳 feasible score 的相对改善都小于 1%，停止；否则继续，最多 90 个候选。

中断以后再次执行相同命令，会从：

```text
matlab/results/mpc_auto_full_auto_best/closed_loop_tuning/auto_tuning_checkpoint.mat
```

继续，而不是从 0 开始。

## 最重要输出

```text
closed_loop_tuning/airdropx_learned_mpc_best.mat
closed_loop_tuning/best_case_metrics.csv
closed_loop_tuning/best_mpc_parameters_logspace.csv
closed_loop_tuning/optimization_trace.csv
closed_loop_tuning/auto_tune_until_best_result.mat
```

`best_case_metrics.csv` 中重点看：

- `steady_h_rms_m`
- `steady_airspeed_rms_mps`
- `steady_vz_rms_mps`
- `steady_q_rms_dps`
- `steady_pitch_mean_deg`：实际自然稳定下来的 pitch 平衡角
- `steady_pitch_std_deg`
- `steady_pitch_drift_degps`
- `min_altitude_m`

## pitch 平衡点

控制器默认使用 `trim_bank(config).pitch_deg`，而不是固定的 `TargetPitchDeg`。五个载荷状态有五个自动配平 pitch。MPC 中 pitch 跟踪权重保持很小，真正压制 pitch 振荡的是 q、pitch std 和 drift 指标。
