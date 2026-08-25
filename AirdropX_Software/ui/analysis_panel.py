from __future__ import annotations

from pathlib import Path
from PyQt6.QtWidgets import QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget

from .widgets import ScatterChart


class AnalysisPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(350)
        self.setMaximumWidth(460)
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

        metric = QGroupBox("软件与控制结果")
        ml = QVBoxLayout(metric)
        self.cep = QLabel("CEP50_to_target: --")
        self.maxdev = QLabel("最大高度偏差: --")
        self.winderr = QLabel("风估计 P95: --")
        self.software = QLabel("软件执行: 等待仿真")
        self.paper_core = QLabel("论文 MPC: --")
        self.engineering = QLabel("工程诊断: --")
        self.conclusion = QLabel("结论: 等待仿真")
        self.software.setObjectName("Good")
        self.paper_core.setObjectName("Good")
        self.engineering.setObjectName("Warn")
        self.conclusion.setWordWrap(True)
        for w in [self.cep, self.maxdev, self.winderr, self.software, self.paper_core, self.engineering, self.conclusion]:
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
        self.software.setText("软件执行: 等待仿真")
        self.paper_core.setText("论文 MPC: --")
        self.engineering.setText("工程诊断: --")
        self.conclusion.setText("结论: 等待仿真")
        self.log.clear()
        self.current_output = None

    @staticmethod
    def _bool_text(value: bool | None) -> str:
        if value is None:
            return "--"
        return "PASS" if value else "FAIL"

    @staticmethod
    def _set_status_label(widget: QLabel, text: str, value: bool | None):
        widget.setText(text)
        widget.setObjectName("Good" if value is True else "Bad" if value is False else "Warn")
        widget.style().unpolish(widget)
        widget.style().polish(widget)

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
        valid_h = [x for x in h if x == x]
        maxdev = max((abs(x - cfg.target_altitude_m) for x in valid_h), default=float("nan"))
        self.maxdev.setText(f"最大高度偏差: {maxdev:.3f} m")
        wt = res.series("wind_truth_mps")
        we = res.series("wind_est_mps")
        e = sorted(abs(a - b) for a, b in zip(wt, we) if a == a and b == b)
        if e:
            idx = min(len(e) - 1, max(0, round(0.95 * (len(e) - 1))))
            self.winderr.setText(f"风估计 P95: {e[idx]:.3f} m/s")

        self._set_status_label(self.software, "软件执行: SUCCESS", True)
        paper = res.bool_from_summary("paper_core_pass")
        engineering = res.bool_from_summary("engineering_gate_pass")
        self._set_status_label(self.paper_core, f"论文 MPC: {self._bool_text(paper)}", paper)
        engineering_text = f"工程诊断: {self._bool_text(engineering)}（不改变论文 MPC）"
        self._set_status_label(self.engineering, engineering_text, engineering)

        published_point = (
            cfg.wind.mode == "formal"
            and abs(cfg.target_altitude_m - 200.0) < 1e-9
            and abs(cfg.target_speed_mps - 50.0) < 1e-9
            and abs(cfg.fuel_scale - 1.0) < 1e-12
        )
        if paper is True and published_point:
            self.conclusion.setText("结论: 软件运行正常，v1.3.6-Paper 论文核心控制验证 PASS。")
        elif paper is True:
            self.conclusion.setText("结论: 软件运行正常，Paper Core PASS；当前自由 H/V/自定义风属于软件扩展工况。")
        else:
            self.conclusion.setText("结论: 软件链路已完成，但论文核心控制 gate 未通过；请查看日志和指标。")
        self.current_output = res.output_root