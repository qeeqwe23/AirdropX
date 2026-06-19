# MQ-9 Reaper 非武器货运空投构型

## 构型概述

本目录包含 MQ-9 Reaper 无人机的非武器货运空投构型（Non-Weapon Cargo Airdrop Configuration），用于 AirdropX 第一阶段 20m 低空无伞投放任务。

## 文件说明

| 文件 | 说明 |
|------|------|
| `MQ9_Reaper.xml` | JSBSim 飞行动力学模型，`fdm_config name="MQ9_Reaper_Cargo"` |
| `reset_20m.xml` | 20m 低空无伞任务初始条件（高度 20m，空速 90m/s） |
| `profiles/mq9_20m_no_chute_profile.json` | 任务配置文件 |

## 与原 HY100 构型的主要差异

| 项目 | HY100（旧） | MQ9_Reaper_Cargo（新） |
|------|-------------|------------------------|
| fdm_config name | MQ-9 Reaper | MQ9_Reaper_Cargo |
| 点质量 | Payload + station1~6（武器挂点） | CARGO_1~CARGO_4（货运点质量） |
| 武器系统 | `<system file="Armament"/>` | 已移除 |
| 粒子系统 | `<system file="Particles"/>` | 已移除 |
| 货物质量 | 混合（武器+载荷） | 4 × 661.386 lb（≈300 kg） |

## 货物点质量分布

| 名称 | x（英寸） | y（英寸） | z（英寸） | 质量（lb） | 质量（kg） |
|------|-----------|-----------|-----------|-----------|-----------|
| CARGO_1 | 190.0 | 0.0 | -12.0 | 661.386 | ≈300 |
| CARGO_2 | 202.0 | 0.0 | -12.0 | 661.386 | ≈300 |
| CARGO_3 | 214.0 | 0.0 | -12.0 | 661.386 | ≈300 |
| CARGO_4 | 226.0 | 0.0 | -12.0 | 661.386 | ≈300 |

CG 参考点：x = 207.84 in，货物围绕 CG 前后对称分布（±18 in 范围）。

## 任务参数

- 目标高度：20 m
- 初始空速：90 m/s
- 投放模式：单件无伞投放
- CARP 前置距离：≈182 m（H=20m，V=90m/s，t_fall≈2.02s）

## 历史备份

原 `aircraft/HY100/` 目录保留作为历史备份，不删除。
