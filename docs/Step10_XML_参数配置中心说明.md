# Step10：XML 参数配置中心（JSBSim 输入参数可视化配置）

## 目的
在 ADS-100 工程中，JSBSim 的关键输入（飞机模型、发动机、初始条件等）主要来自 XML。
以往需要手动编辑 XML，容易出错且难以固化配置。

本 Step10 增加：
- **UI：XML 参数配置中心**（浏览/搜索/编辑/保存）
- **profile：导入/导出 JSON**（固化参数、复现实验、对接企业参数）
- **保存自动备份 + diff**（便于回滚与审计）

## UI 使用方式
在主界面左侧“任务配置面板”中点击：**XML 参数配置中心**。

界面包含：
- XML 文件下拉框：选择 aircraft/HY100/HY100.xml、aircraft/HY100/reset.xml、engine/HY100_Turbojet.xml、engine/direct.xml
- 树形浏览：Tag / Value / Attrib
- 搜索：按 tag/xpath/值过滤
- 节点编辑：
  - text：编辑节点文本（通常为数值，表格/长文本也支持）
  - attrib：编辑属性键值（例如 unit="KG"）
  - “应用到节点（内存）”：只修改内存，不写入磁盘
- 保存：**保存XML（备份+diff）**
  - 自动备份：xml_backups/<timestamp>/...
  - 自动 diff：xml_diffs/<filename>.diff

> 提示：若仿真正在运行，修改 XML 并保存后，建议停止并重启仿真，使模型重新加载。

## Profile（JSON）
### 1. 导出 profile
在 UI 中点击“导出Profile”，默认导出当前文件的叶子节点（含 text/attrib）。
也可使用命令行：

```bash
python tools/xml_profile_export.py --out profiles/hy100_profile.json
```

### 2. 导入 profile（仅加载到内存）
在 UI 中点击“导入Profile”，仅将 profile 内容加载到 UI 内存并刷新显示。
若确认无误，再点击“保存XML”。

### 3. 应用 profile 到 XML（直接写入，自动备份）
UI：点击“应用Profile→XML”

或命令行：
```bash
python tools/xml_profile_apply.py --profile profiles/hy100_profile.json
```

## 二次开发建议
- 若要实现“完全不改原始 XML”的影子配置：
  - 建议在 core/path_utils.py 中增加 ADS100_MODEL_ROOT 覆盖
  - 在 profiles/<name>/ 下生成 aircraft/engine 的完整镜像
  - JSBSim wrapper 从 ADS100_MODEL_ROOT 加载模型

- 若要把“企业参数需求表”自动写回：
  - 可将 CSV → profile JSON 的转换做成 tools/csv_to_profile.py
  - 企业填表后直接生成 profile，确保符号与单位一致。
