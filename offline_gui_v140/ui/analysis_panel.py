from __future__ import annotations

from pathlib import Path
from PyQt6.QtWidgets import QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget

from .widgets import ScatterChart


class AnalysisPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(330)
        self.setMaximumWidth(430)
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 4, 6, 4)
        title = QLabel("【精度统计分析】")
        title.setObjectName("Section")
        root.addWidget(title)

        scatter_box = QGroupBox("载荷落点分布")
        sb = QVBoxLayout(scatter_box)
        self.scatter = ScatterChart()
        sb.addWidget(self.scatter)
        root.addWidget(scatter_box, 2)

        metric = QGroupBox("精度指标")
        ml = QVBoxLayout(metric)
        self.cep = QLabel("CEP50_to_target: --")
        self.maxdev = QLabel("最大高度偏差: --")
        self.winderr = QLabel("风估计 P95: --")
        self.conclusion = QLabel("结论: 等待仿真")
        self.conclusion.setObjectName("Good")
        for w in [self.cep, self.maxdev, self.winderr, self.conclusion]:
            ml.addWidget(w)
        root.addWidget(metric)

        logbox = QGroupBox("任务事件日志")
        ll = QVBoxLayout(logbox)
        tools = QHBoxLayout()
        self.export = QPushButton("导出报告")
        self.filter = QComboBox()
        self.filter.addItems(["全部", "INFO", "WARN", "ERROR"])
        self.clear = QPushButton("清空")
        tools.addWidget(self.export)
        tools.addWidget(QLabel("过滤:"))
        tools.addWidget(self.filter)
        tools.addWidget(self.clear)
        ll.addLayout(tools)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        ll.addWidget(self.log)
        root.addWidget(logbox, 2)
        self.clear.clicked.connect(self.log.clear)
        self.current_output: Path | None = None

    def append_log(self, text: str):
        self.log.append(text)

    def reset(self):
        self.scatter.set_points([])
        self.cep.setText("CEP50_to_target: --")
        self.maxdev.setText("最大高度偏差: --")
        self.winderr.setText("风估计 P95: --")
        self.conclusion.setText("结论: 等待仿真")
        self.log.clear()
        self.current_output = None

    def show_result(self, res, cfg):
        errs = []
        for row in res.cargo:
            try:
                if float(row.get("released", "0")) > 0.5:
                    errs.append(float(row.get("landing_error_m", "nan")))
            except Exception:
                pass
        signed = []
        for row in res.cargo:
            try:
                signed.append(float(row["truth_impact_m"]) - float(row["target_m"]))
            except Exception:
                pass
        self.scatter.set_points(signed)
        if errs:
            a = sorted(abs(x) for x in errs)
            cep = a[(len(a) - 1) // 2]
            self.cep.setText(f"CEP50_to_target: {cep:.3f} m")
        h = res.series("h_est_m")
        maxdev = max((abs(x - cfg.target_altitude_m) for x in h), default=float("nan"))
        self.maxdev.setText(f"最大高度偏差: {maxdev:.3f} m")
        wt = res.series("wind_truth_mps")
        we = res.series("wind_est_mps")
        e = sorted(abs(a - b) for a, b in zip(wt, we) if a == a and b == b)
        if e:
            idx = min(len(e) - 1, max(0, round(0.95 * (len(e) - 1))))
            self.winderr.setText(f"风估计 P95: {e[idx]:.3f} m/s")
        pass_text = res.metric_from_summary("pass")
        if pass_text in {"1", "true", "TRUE"}:
            text = "结论: PASS"
        else:
            text = "结论: 任务完成（自定义工况不等同论文正式认证）"
        self.conclusion.setText(text)
        self.current_output = res.output_root
