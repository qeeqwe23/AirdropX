AirdropX Physics MPC v1.1
=========================

本版针对 Physics MPC v1 首次构建暴露的两个工程问题：
1) trim struct merge 字段不一致导致 heterogeneous struct assignment；
2) 单节点失败会使整批 15 个 V×cfg 节点提前停止、CPU 利用率低。

v1.1 改动
---------
1. local_merge_trim 只更新 source trim bank 已存在的共有字段，不允许线性化诊断字段污染 struct-array schema。
2. 默认使用 3 个 MATLAB process workers，并行执行 15 个独立 V×cfg 节点。
3. 每个 worker 使用独立 OutputRoot；底层 airdropx_auto_run_id_experiment 已具备 per-worker Simulink file-generation 隔离。
4. 一个节点失败不会中断其它节点。所有节点结束后统一写：
     matlab/results/mpc_physics_v1/physics_mpc_build_failures.csv
5. 节点结果可恢复：physics_linear_model.mat 版本匹配 v1.1 时自动 REUSE。
6. 若旧 trim candidate 未通过 equilibrium，不进行 BO/学习，而调用：
     airdropx_physics_trim_solve.m
   该函数仅求解两个物理控制变量 elevator/throttle，以固定有限差分测得
   [Va error, h slope] 对 [elevator, throttle] 的 2×2 Jacobian，再做阻尼 Newton 修正。
7. Newton 求解仍必须通过原来的硬认证：Va、vz、q、hSlope、VaSlope、pitchStd 全部达标才接受。
8. 每次物理求解都保存：
     deterministic_retrim/deterministic_trim_trace.csv
   不存在随机搜索、自动放宽门限或无限重试。
9. preparation 初始 flight-path 与 v32 sequential trim 路径保持一致，使用 cfg0 bank 的 initial_flight_path_deg。

为什么 V45 cfg3 需要重新求物理平衡
---------------------------------
用户提供的旧 cfg3 trim_result 已显示其长尾并非真正稳态：
  tailVzMed ≈ -0.314 m/s
  tailHeightSlope ≈ -0.305 m/s
  tailVaSlope ≈ +0.0503 m/s^2
而新的 Physics-MPC 再认证得到：
  vz ≈ +0.225 m/s
  hSlope ≈ +0.236 m/s
这说明旧点对 preparation/reset 条件敏感，不能作为严谨 MPC 线性化基点。
因此 v1.1 不直接“调 MPC”，先通过确定性平衡方程求解得到可重复的物理工作点。

运行
----
覆盖到 AirdropX 根目录后直接：
  .\run_physics_mpc_build_D_temp.ps1

默认：
  ParallelWorkers = 3
  ReuseExistingNodeResults = true
  AllowDeterministicRetrim = true

观察：
  matlab/results/mpc_physics_v1/linearization/V045.000/cfg3/deterministic_retrim/deterministic_trim_trace.csv
  matlab/results/mpc_physics_v1/physics_mpc_build_failures.csv
  matlab/results/mpc_physics_v1/physics_mpc_model_report.csv

注意
----
旧 physics_mpc_v1 的 physics_linear_model.mat 不会被 v1.1 当作缓存复用，因为版本签名不同。
这是有意的：v1.1 对 preparation 初始路径和 equilibrium solver 做了修正，应重新认证全部节点。
