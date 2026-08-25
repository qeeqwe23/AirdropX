# AirdropX 空投仿真与测控软件 v1.0

## 软件定位

本软件固定使用 **Physics-MPC v1.3.6-Paper** 作为控制后端。

不加入针对 Engineering Gate 的额外尾段补丁，不修改论文 MPC 控制策略。GUI 只负责任务参数配置、调用 MATLAB/JSBSim、结果读取与可视化。

## 已验证基线

当前已在 Windows 本机完成端到端验证：

- GUI → v1.3.6-Paper → MATLAB → JSBSim：SUCCESS
- 工况：200 m / 50 m/s / 论文-无风
- Paper Core：PASS
- 4/4 投放完成
- max landing error：0.5921 m
- RMS landing error：0.3869 m
- predicted impact error at release max：0.1561 m
- wind estimate P95 error：0.1691 m/s
- QP success fraction：1.0

严格 Engineering Gate 的尾段稳定性诊断在该次测试中未通过（tail5s_normalized_rms=0.0650），但它不改变 Paper Core 的 PASS，也不代表 GUI/后端执行失败。软件版保留该值作为诊断，不围绕它叠加新控制逻辑。

## 主要功能

- 固定 Physics-MPC v1.3.6-Paper 控制器
- 自由设置目标高度
- 自由设置目标速度
- 自定义风速与风向
- 无风 / 恒定风 / 阶跃 / 双向阶跃 / Ramp / 正弦 / 复合阵风
- 保留论文正式风场预设
- 4 件连续空投
- 高度、速度、俯仰、控制量、质量、轨迹显示
- 落点误差与风估计误差统计
- 任意 H/V 精确模型点自动生成并缓存

说明：当前控制器是纵向 MPC。GUI 中任意风向会投影为沿航迹风分量；横风分量不伪装成已进入纵向控制器。

## 一键安装

解压后双击：

```text
安装AirdropX软件.bat
```

默认会尝试项目路径：

```text
D:\vscode project\AirdropX
```

安装过程会：

1. 安装/确认 v1.3.6-Paper overlay；
2. 安装 GUI bridge；
3. 安装自定义风场适配层；
4. 将软件复制到 `AirdropX_Software`；
5. 使用本机 Python 构建 Windows GUI 可执行程序。

## 启动

安装完成后运行：

```text
D:\vscode project\AirdropX\AirdropX_Software\AirdropX.bat
```

如果 PyInstaller 构建成功，它会优先启动：

```text
AirdropX_Software\dist\AirdropX\AirdropX.exe
```

否则会回退到本地 Python GUI。

## 软件结果判定

界面重点看两项：

- **软件执行**：GUI → MATLAB → JSBSim → MPC → CSV 是否完整运行。
- **论文 MPC**：`paper_core_pass` 是否 PASS。

`Engineering Gate` 仅作为工程诊断显示，不用于把一个 Paper Core PASS 的软件任务标记成“软件失败”。

## 依赖

当前 v1.0 是离线桌面软件，但控制后端仍调用本机 MATLAB，因此目标电脑需要：

- MATLAB（当前工程使用 R2026a）
- 对应 MPC / Optimization 工具箱
- JSBSim / AirdropX 工程文件
- 首次生成新 H/V 时需要现有 Physics Oracle MEX

生成 `AirdropX.exe` 后，Python/PyQt6 不再是启动软件的必需环境；MATLAB 后端依赖仍然存在。
