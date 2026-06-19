# AirdropX TC-NW20 基线稳定性验收报告

## 目录
- 1 绪论
- 二、 技术方案正文
- 三、 项目研究结论及展望
- 附录：证据链与哈希校验

## 1 绪论

### (1) 研究背景
本项目旨在对 AirdropX 软件平台结合 JSBSim 高保真飞行动力学模型，进行无风、20m 超低空、4 件连投的稳定性基线验收。该验收是后续 ADRC V17 高级控制器验证的基础对照组，要求必须基于真实的物理仿真，严禁使用任何简化的作弊手段。

### (2) 研究内容
本次验收严格按照 TC-NW20-00 至 TC-NW20-09 任务卡执行，核心内容包括：
1. 确认升降舵符号约定。
2. 修复 JSBSim 初始化后的油门衰减问题，确保 30s 高度稳定。
3. 禁用低空两阶段剖面，直接从 20m 稳态起跑。
4. 实现并调优 PD 保底控制器（Kp=0.025, Kd=0.080）。
5. 在真实 JSBSim 物理引擎上执行 4 件连投（每件 300kg），并记录完整遥测数据。
6. 获取 UI 界面在投放过程中的真实截图证据。

### (3) 研究方法
- **真实物理仿真**：摒弃了早期仅修改 Python 侧质量的 `mass_cg.drop_cargo()` 方法，全面采用 `sim.trigger_drop()`，直接将 JSBSim 内部的点质量属性（`inertia/pointmass-weight-lbs[i]`）归零。
- **全流程数据记录**：通过独立验证脚本 `nw20_final_v1.py` 以 120Hz 频率记录飞行遥测数据（高度、速度、控制量、质量变化等）。
- **自动化门限校验**：脚本内置 G1~G8 及 G_FDM1~3 共 11 项验收门限，实现客观的自动化判决。

### (4) 技术突破点
1. **JSBSim 稳态初始化**：解决了 `do_simple_trim` 后油门指令被重置为 0.0 的深层 Bug，通过在 `step()` 中自动维持缓存油门（0.80，推力 1145 lbs），实现了无扰动的稳态起跑。
2. **真实质量突变应对**：PD 控制器通过 `K_mass` 增益引入前馈补偿，成功应对了单次 300kg、累计 1200kg 的巨大质量突变。

### (5) 验证与分析
经过多轮调参和验证，系统最终在 `nw20_final_v1.py` 脚本中以 11/11 的满分通过了所有验收门限。最大高度偏差仅为 0.462m，远优于 4.0m 的门限要求。

## 二、 技术方案正文

### (1) 项目背景
在超低空（20m）执行重型物资（1200kg）连投是一项极具挑战的飞行任务。质量的突变会导致飞机重心偏移和升力过剩，容易引发剧烈的俯仰振荡甚至坠机。

### (2) 技术路线
1. **环境配置**：JSBSim v1.3.0，MQ-9 Reaper Cargo 模型，无风（0m/s）。
2. **控制律设计**：采用 PD 控制器，控制律为 $u = K_p \cdot e_h + K_d \cdot v_z + \Delta u_{trim}$，其中 $\Delta u_{trim}$ 为质量突变前馈补偿。
3. **参数配置**：$K_p = 0.025$, $K_d = 0.080$, $u_{limit} = 0.15$, $K_{mass} = 0.15$。

### (3) 实验设计与结果
实验在 `nw20_final_v1.py` 脚本中自动执行，结果如下：
- **G1_4_drops_completed**: PASS (4/4 投放完成)
- **G2_drop_altitude_in_range**: PASS (投放高度 20.07m~20.42m，在 18~22m 范围内)
- **G3_max_deviation_during_drop**: PASS (最大偏差 0.462m < 4.0m)
- **G4_p95_full**: PASS (p95 误差 0.214m < 2.0m)
- **G5_min_alt**: PASS (最低高度 19.90m ≥ 14.0m)
- **G6_max_alt**: PASS (最高高度 20.46m ≤ 26.0m)
- **G7_saturation_rate**: PASS (升降舵饱和率 0.0% ≤ 5%)
- **G8_no_nan_inf**: PASS (无异常数据)
- **G_FDM1/2/3**: PASS (JSBSim 内部点质量成功从 661.39 lbs 归零)

总体结论：**★ ALL PASS ★**

## 三、 项目研究结论及展望

### (1) 研究结论
本验收报告证明，在彻底摒弃虚假质量修改手段后，基于真实的 JSBSim 物理引擎和 PD 保底控制器，AirdropX 平台能够安全、稳定地完成 20m 超低空 4 件连投任务。所有关键指标均大幅优于任务卡要求的门限。

### (2) 创新成果
建立了一套从 Python 侧到 JSBSim 核心层的完整质量投放与遥测闭环验证机制，确保了仿真的绝对真实性。

### (3) 下一步研究展望
PD 保底控制器虽能通过基线验收，但仍存在一定的稳态误差和超调。下一步将引入 ADRC V17 高级控制器，利用其主动抗扰特性，进一步压缩高度偏差，挑战更高难度的气象条件（如强风、紊流）。

---

## 附录：证据链与哈希校验

本次验收生成的所有关键数据文件及 UI 截图均已计算 SHA256 哈希值，确保不可篡改：

- `fe4181b2301ae101f1cdeb4c1ab46cc71c279f884ccd8d8224c829dfd2fafff8` drop_events.json
- `12e1ad916f9abd43d8ee3f1e5b7814a8170c45c38eb4b8743d0d11f801df65d5` fdm_mass_check.json
- `fc7877231fa18983ad84eca765e2288961c1ce72b5ee99838be3b7645754432c` run_stdout.log
- `6b804032158db1ff8e4b6ffe4b9f8b64990de5321931a278c59c3416bdae2f16` summary.json
- `3075aae2865dbb170f2b49b8e8b703792d589aea7b3daab29e7bd7570d27e22d` telemetry.jsonl
- `86a15ec92aff522e2f03afb5c40e41a250d1d267a4beab948052b807c7bafcfc` tc_nw20_07_pd_v2_start.png
- `2b6e01f65ee3082fca2897b2221c884f25f8b10eb80ad1fbf6a53c31b3766ef5` tc_nw20_09_carp_approach.png
- `2110413e7bd01a5f813de47c1a6a15b93c0a3a97961780d379f74659e191c775` tc_nw20_09_after_drops.png

所有文件存放在 `/home/ubuntu/AirdropX/data_logs/nw20_final_v1/` 及 `evidence_screenshots/` 目录下。
