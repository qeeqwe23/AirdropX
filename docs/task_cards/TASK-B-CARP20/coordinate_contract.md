# TASK-B-CARP20 坐标系与目标点契约

**版本**：v1.0  
**生效范围**：Task-B 全部 TC（TC-B0 ～ TC-B7）  
**强制级别**：必须遵守，任何违反本契约的代码不得纳入验收

---

## 1 坐标系定义

本项目采用局部北东坐标系（NED 局部水平面投影），原点为仿真初始位置。

| 符号 | 含义 | 单位 | 正方向 |
|------|------|------|--------|
| `N` | 北向坐标（North） | m | 向北为正 |
| `E` | 东向坐标（East） | m | 向东为正 |
| `Alt` | 高度（Altitude） | m | 向上为正 |

---

## 2 三类坐标定义

### 2.1 目标点（Target）

> 任务预设目标点，**在仿真开始前固定，仿真过程中不允许修改**。

```
Target_E  [m]  : 目标点东向坐标
Target_N  [m]  : 目标点北向坐标
```

**禁止事项**：
- 不得把 Impact 均值重新定义为 Target
- 不得在仿真过程中动态修改 Target
- 不得用 UI 演示目标点替代验收目标点

### 2.2 释放点（Release）

> CARP 求解器根据目标点反算的货物释放位置。

```
Release_E  [m]  : 释放点东向坐标 = Target_E - 东向弹道补偿
Release_N  [m]  : 释放点北向坐标 = Target_N - 弹道前飞距离
```

**计算原则**：Release_N = Target_N - D_ballistic（弹道前飞距离）

### 2.3 落点（Impact）

> 货物实际落地坐标，由 JSBSim 弹道积分得到。

```
Impact_E  [m]  : 货物落点东向坐标
Impact_N  [m]  : 货物落点北向坐标
```

---

## 3 误差定义

```
Error_E = Impact_E - Target_E
Error_N = Impact_N - Target_N
Impact_Error = sqrt(Error_E^2 + Error_N^2)
```

---

## 4 精度指标定义

### 4.1 CEP50_to_target（圆概率误差 50%）

> **定义**：以目标点为圆心，包含 50% 落点的最小圆半径。

```
CEP50_to_target = percentile(Impact_Error_array, 50)
```

**禁止替代**：不得用 `CEP_around_mean`（以落点均值为圆心）替代 `CEP_to_target`。

### 4.2 CEP90_to_target（圆概率误差 90%）

```
CEP90_to_target = percentile(Impact_Error_array, 90)
```

---

## 5 文件格式契约

所有 telemetry/drop/impact 输出文件必须包含以下三类坐标字段：

```json
{
  "target": {
    "target_e_m": 0.0,
    "target_n_m": 200.0
  },
  "release": {
    "release_e_m": 0.0,
    "release_n_m": 123.4
  },
  "impact": {
    "impact_e_m": 0.3,
    "impact_n_m": 199.7
  },
  "error": {
    "error_e_m": 0.3,
    "error_n_m": -0.3,
    "impact_error_m": 0.424
  }
}
```

---

## 6 验收门限

| Gate | 要求 |
|------|------|
| G1 | 所有 telemetry/drop/impact 文件包含 target/release/impact 三类坐标 |
| G2 | target 在仿真开始前固定，记录于 runtime_env.json |
| G3 | 仿真过程中 target 不允许被修改（代码层面禁止赋值） |
| G4 | CEP50_to_target 由 impact_errors_to_target 数组复算，不得使用 CEP_around_mean |

---

## 7 禁止行为清单

以下行为在 Task-B 中**严格禁止**，一经发现立即判定该 TC 不通过：

(1) 把 Impact 均值重新定义为 Target，再声称 CEP 很小  
(2) 用 CEP_around_mean 替代 CEP_to_target  
(3) 在仿真过程中修改 Target_N 或 Target_E  
(4) 用 UI 演示目标点（如 5000m 或随机值）作为验收目标点  
(5) Release_N = Target_N（不做弹道前飞补偿）  
(6) 混用 ground_speed 和 airspeed 计算弹道前飞距离  
