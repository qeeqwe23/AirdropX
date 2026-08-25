from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QLabel, QLineEdit, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from core.app_config import CUSTOM_WIND_TYPES, FORMAL_SCENARIOS, MissionConfig, WindConfig


class ConfigPanel(QWidget):
    start_requested = pyqtSignal(object)
    stop_requested = pyqtSignal()
    reset_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(390)
        self.setMaximumWidth(480)
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 4, 8, 4)
        root.setSpacing(6)

        hdr = QLabel("【参数配置与控制台】")
        hdr.setObjectName("Section")
        root.addWidget(hdr)

        env = QGroupBox("环境建模 / 飞行任务")
        g = QGridLayout(env)
        self.controller = QLabel("Physics-MPC v1.3.6-Paper")
        self.controller.setObjectName("Good")
        self.alt = self._dspin(5, 10000, 200, 1, " m")
        self.speed = self._dspin(10, 150, 50, 0.5, " m/s")
        self.wind_speed = self._dspin(0, 60, 0, 0.5, " m/s")
        self.wind_dir = self._dspin(0, 360, 0, 5, " deg")
        self.wind_mode = QComboBox()
        self.wind_mode.addItems(["自定义风场", "论文验证预设"])
        self.wind_type = QComboBox()
        self.wind_type.addItems(CUSTOM_WIND_TYPES.keys())
        self.formal = QComboBox()
        self.formal.addItems(FORMAL_SCENARIOS.keys())
        self.formal.setVisible(False)
        self.wind_component = QLabel("沿航迹分量: +0.000 m/s")
        self.wind_component.setObjectName("Good")

        g.addWidget(QLabel("控制器"), 0, 0)
        g.addWidget(self.controller, 0, 1, 1, 3)
        g.addWidget(QLabel("目标高度(m)"), 1, 0)
        g.addWidget(self.alt, 1, 1)
        g.addWidget(QLabel("目标速度(m/s)"), 1, 2)
        g.addWidget(self.speed, 1, 3)
        g.addWidget(QLabel("风速(m/s)"), 2, 0)
        g.addWidget(self.wind_speed, 2, 1)
        g.addWidget(QLabel("风向(deg 来向)"), 2, 2)
        g.addWidget(self.wind_dir, 2, 3)
        g.addWidget(QLabel("风场模式"), 3, 0)
        g.addWidget(self.wind_mode, 3, 1, 1, 3)
        g.addWidget(QLabel("风型"), 4, 0)
        g.addWidget(self.wind_type, 4, 1, 1, 3)
        g.addWidget(self.formal, 5, 0, 1, 4)
        g.addWidget(self.wind_component, 6, 0, 1, 4)
        note = QLabel("当前为纵向 MPC：任意风向按沿航迹分量进入控制；横风分量不参与纵向控制。")
        note.setWordWrap(True)
        note.setStyleSheet("color:#d0a900;font-size:10px")
        g.addWidget(note, 7, 0, 1, 4)
        root.addWidget(env)

        mission = QGroupBox("任务模式")
        fm = QFormLayout(mission)
        self.task = QLineEdit("Physics-MPC v1.3.6-Paper · 公平传感器路径 · 4 连投")
        self.task.setReadOnly(True)
        fm.addRow("当前任务", self.task)
        root.addWidget(mission)

        drop = QGroupBox("投放任务")
        gd = QGridLayout(drop)
        self.drop_count = QSpinBox()
        self.drop_count.setRange(4, 4)
        self.drop_count.setValue(4)
        self.target_start = self._dspin(100, 100000, 1200, 10, " m")
        self.target_spacing = self._dspin(1, 10000, 80, 5, " m")
        self.duration = self._dspin(30, 600, 55, 5, " s")
        gd.addWidget(QLabel("模式"), 0, 0)
        gd.addWidget(QLabel("4 件自动连续投放"), 0, 1, 1, 3)
        gd.addWidget(QLabel("目标起点"), 1, 0)
        gd.addWidget(self.target_start, 1, 1)
        gd.addWidget(QLabel("目标间距"), 1, 2)
        gd.addWidget(self.target_spacing, 1, 3)
        gd.addWidget(QLabel("任务时长"), 2, 0)
        gd.addWidget(self.duration, 2, 1)
        gd.addWidget(QLabel("件数"), 2, 2)
        gd.addWidget(self.drop_count, 2, 3)
        self.start_btn = QPushButton("启动系统仿真")
        self.drop_btn = QPushButton("执行物资投放")
        self.drop_btn.setObjectName("Drop")
        self.drop_btn.setToolTip("投放由论文 CARP/分数采样调度自动执行；按钮保留原界面语义。")
        self.stop_btn = QPushButton("停止仿真")
        self.reset_btn = QPushButton("重置")
        gd.addWidget(self.start_btn, 3, 0, 1, 4)
        gd.addWidget(self.drop_btn, 4, 0, 1, 4)
        gd.addWidget(self.stop_btn, 5, 0, 1, 2)
        gd.addWidget(self.reset_btn, 5, 2, 1, 2)
        root.addWidget(drop)

        ctrl = QGroupBox("控制器状态")
        gc = QGridLayout(ctrl)
        values = {
            "控制器": "Physics-MPC v1.3.6-Paper",
            "预测域": "Np=Nc=100",
            "状态源": "论文公平传感器路径",
            "投放评分": "v1.3.6 Paper nominal ballistic",
            "风估计": "Causal onboard estimate",
            "模型策略": "H/V 精确点自动缓存",
        }
        for i, (key, value) in enumerate(values.items()):
            gc.addWidget(QLabel(key), i, 0)
            lab = QLabel(value)
            lab.setObjectName("Good")
            gc.addWidget(lab, i, 1)
        root.addWidget(ctrl)

        paths = QGroupBox("离线后端")
        fp = QFormLayout(paths)
        self.project_root = QLineEdit()
        self.project_root.setPlaceholderText("AirdropX 项目根目录（可自动识别）")
        self.matlab = QLineEdit()
        self.matlab.setPlaceholderText("matlab.exe（可自动识别）")
        self.model_policy = QComboBox()
        self.model_policy.addItems(["自动生成/缓存新 H/V", "仅允许现有模型点"])
        fp.addRow("项目根目录", self.project_root)
        fp.addRow("MATLAB", self.matlab)
        fp.addRow("H/V 策略", self.model_policy)
        root.addWidget(paths)
        root.addStretch(1)

        self.wind_mode.currentIndexChanged.connect(self._wind_mode_changed)
        self.wind_speed.valueChanged.connect(self._update_projection)
        self.wind_dir.valueChanged.connect(self._update_projection)
        self.start_btn.clicked.connect(lambda: self.start_requested.emit(self.get_config()))
        self.stop_btn.clicked.connect(self.stop_requested)
        self.reset_btn.clicked.connect(self.reset_requested)
        self._wind_mode_changed(self.wind_mode.currentIndex())

    def _dspin(self, a, b, v, step, suffix):
        w = QDoubleSpinBox()
        w.setRange(a, b)
        w.setDecimals(2)
        w.setSingleStep(step)
        w.setValue(v)
        w.setSuffix(suffix)
        return w

    def _wind_mode_changed(self, idx):
        custom = idx == 0
        self.wind_type.setVisible(custom)
        self.formal.setVisible(not custom)
        self.wind_speed.setEnabled(custom)
        self.wind_dir.setEnabled(custom)
        self._update_projection()

    def _update_projection(self):
        w = WindConfig(speed_mps=self.wind_speed.value(), direction_from_deg=self.wind_dir.value())
        self.wind_component.setText(f"沿航迹分量: {w.along_track_mps():+.3f} m/s（正=顺风，负=逆风）")

    def get_config(self):
        if self.wind_mode.currentIndex() == 0:
            kind = CUSTOM_WIND_TYPES[self.wind_type.currentText()]
            mode = "custom"
        else:
            kind = FORMAL_SCENARIOS[self.formal.currentText()]
            mode = "formal"
        wind = WindConfig(
            mode=mode,
            kind=kind,
            speed_mps=self.wind_speed.value(),
            direction_from_deg=self.wind_dir.value(),
        )
        return MissionConfig(
            backend_mode="v136p",
            target_altitude_m=self.alt.value(),
            target_speed_mps=self.speed.value(),
            duration_s=self.duration.value(),
            target_start_m=self.target_start.value(),
            target_spacing_m=self.target_spacing.value(),
            wind=wind,
            project_root=self.project_root.text().strip(),
            matlab_exe=self.matlab.text().strip(),
            model_policy="auto_cache" if self.model_policy.currentIndex() == 0 else "certified_only",
        )

    def set_running(self, running: bool):
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        self.drop_btn.setEnabled(not running)
        self.alt.setEnabled(not running)
        self.speed.setEnabled(not running)
        self.wind_mode.setEnabled(not running)