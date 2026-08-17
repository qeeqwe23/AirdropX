AirdropX Physics MPC v1
=======================

目标
----
停止 v32 自动学习路线。控制器改为：
  物理配平候选 -> 零激励长期平衡认证 -> JSBSim 小扰动局部雅可比
  -> 固定规则 MPC -> 速度连续调度 -> 高度外层 vz governor。

硬性设计
--------
1. 运行中 Hcmd / Vcmd 可以随时改变，不重新训练、不重新辨识、不重启。
2. 高度不进入 inner MPC；Hcmd 只生成可实现的 vz_ref。
3. Inner MPC 状态为 [Va, pitch, vz, q]。
4. pitch 不强制为 0 度；每个工况以物理配平 pitch 为自然中心，只要求稳定/有界。
5. pitch(k+1)=pitch(k)+Ts*q(k) 是精确运动学，不由辨识算法估计。
6. 45/50/55 m/s 只是局部物理模型支撑节点；47.3、52.4 等任意中间速度可直接命令。
7. cfg0..cfg4 是真实质量/CG 状态。投放时切换模型，但保持实际舵量连续。
8. 无 bayesopt、无 n4sid/ssest 阶次搜索、无 persistent learning、无失败后自动放宽门限。
9. JSBSim 内部 autoTrimSettle 升降舵隐藏偏置按速度节点实测并连续插值，不再假定一个全局常数。

安装
----
把本包内容覆盖到 AirdropX 根目录。它只新增文件，不应删除旧 v32 文件。

预检查
------
PowerShell:
  .\run_physics_mpc_preflight_D_temp.ps1

构建
----
PowerShell:
  .\run_physics_mpc_build_D_temp.ps1

构建过程会读取：
  matlab/results/mpc_auto_v32_clean/knowledge_bank/physics/V045.000/v32_trim_bank.mat
  matlab/results/mpc_auto_v32_clean/knowledge_bank/physics/V050.000/v32_trim_bank.mat
  matlab/results/mpc_auto_v32_clean/knowledge_bank/physics/V055.000/v32_trim_bank.mat

这些 trim 只是“候选平衡点”，每个都会重新做零激励认证；若某个点漂移，程序直接停止并报告，不会再自动优化。

输出：
  matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat
  matlab/results/mpc_physics_v1/physics_mpc_model_report.csv
  每个 V/cfg 的 baseline_metrics.csv / linear_model_validation.csv / physics_linear_model.mat

动态验证
--------
  .\run_physics_mpc_validation_D_temp.ps1

默认在一趟连续飞行中测试非节点命令：
  200m/50 -> 137m/47.3 -> 184m/52.4 -> 76m/49.1
  -> 28m/54.2 -> 121m/46.6 -> 200m/50
并在过程中进行 4 次投放。

运行中人工改目标
--------------
如果 Simulink 正在 Normal 模式交互运行，在 MATLAB 命令窗执行：
  airdropx_physics_mpc_set_reference(137,52.4)
下一次 0.1 s 控制采样即读取新命令；内部 governor 会限制爬升/下降和加减速率，不会把阶跃直接打到舵面。

当前正式认证范围
--------------
高度：20..200 m
空速：45..55 m/s
速度节点只用于模型调度，不限制用户只能输入 45/50/55。

如果构建报 MissingTrim
---------------------
说明你当前结果目录中某个 V45/V50/V55 的 v32_trim_bank.mat 不存在。这时只需要提供/恢复对应 trim bank；不需要重新提供整个飞机模型。
