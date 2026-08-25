from __future__ import annotations

import math
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QGridLayout, QLabel, QTabWidget, QVBoxLayout, QWidget

from .widgets import AttitudeWidget, EnergyGauge, LineChart


class MonitorPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.backend_label = "Physics-MPC v1.3.6-Paper"
        lay = QVBoxLayout(self)
        lay.setContentsMargins(5, 4, 5, 4)
        title = QLabel("【实时态势感知窗口 (3D Digital Twin)】")
        title.setObjectName("Section")
        lay.addWidget(title)
        self.banner = QLabel(f"{self.backend_label} · 等待任务")
        self.banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.banner.setStyleSheet("color:#aaa;font-size:14px;padding:10px")
        lay.addWidget(self.banner)

        self.tabs = QTabWidget()
        lay.addWidget(self.tabs, 1)
        self.height_chart = LineChart("高度保持稳定性监控", "高度(m)")
        self.pitch_chart = LineChart("俯仰角 / q 监控", "deg")
        self.control_chart = LineChart("控制指令", "command")
        self.mass_chart = LineChart("质量 / CG / Iyy", "mass")
        self.tabs.addTab(self.height_chart, "高度监控")
        self.tabs.addTab(self.pitch_chart, "俯仰角监控")
        self.tabs.addTab(self.control_chart, "控制指令")
        self.tabs.addTab(self.mass_chart, "质量重心")

        bottom = QGridLayout()
        self.trajectory = LineChart("空投货物弹道轨迹（沿航迹距离-高度）", "高度(m)")
        self.energy = EnergyGauge()
        self.attitude = AttitudeWidget()
        self.damping = QLabel("Damping Wall\nidle")
        self.damping.setStyleSheet("border:1px solid #00bcd4;padding:8px;color:#ddd")
        bottom.addWidget(self.trajectory, 0, 0, 2, 4)
        bottom.addWidget(self.energy, 2, 0, 1, 2)
        bottom.addWidget(self.attitude, 2, 2)
        bottom.addWidget(self.damping, 2, 3)
        lay.addLayout(bottom)

    def set_backend(self, mode: str):
        self.backend_label = "Physics-MPC v1.3.6-Paper" if mode == "v136p" else "Physics-MPC v1.4.0 (实验)"
        self.banner.setText(f"{self.backend_label} · 等待任务")

    def reset(self):
        for c in [self.height_chart, self.pitch_chart, self.control_chart, self.mass_chart, self.trajectory]:
            c.clear()
        self.banner.setText(f"{self.backend_label} · 等待任务")
        self.attitude.set_pitch(0)
        self.energy.set_value(0)
        self.damping.setText("Damping Wall\nidle")

    def show_result(self, res, cfg):
        t = res.series("t_s")
        h = res.series("h_est_m")
        ht = res.series("h_truth_m")
        va = res.series("Va_est_mps")
        theta = res.series("theta_est_rad")
        q = res.series("q_est_radps")
        elev = res.series("elevator_cmd")
        thr = res.series("throttle_cmd")
        mass = res.series("mass_truth_kg")
        pos = res.series("pos_n_est_m")
        rec = res.series("recovery_level")
        drops = []
        for r in res.timeseries:
            try:
                if float(r.get("drop_event", "0")) > 0.5:
                    drops.append(float(r["t_s"]))
            except Exception:
                pass
        self.height_chart.clear()
        self.height_chart.add_line(t, ht, "#00ff55", "truth")
        self.height_chart.add_line(t, h, "#00dfff", "estimate")
        self.height_chart.add_line(t, [cfg.target_altitude_m] * len(t), "#ff3030", "target", Qt.PenStyle.DashLine)
        for d in drops:
            self.height_chart.add_marker(d)
        self.pitch_chart.clear()
        self.pitch_chart.add_line(t, [x * 180 / math.pi for x in theta], "#00dfff", "pitch")
        self.pitch_chart.add_line(t, [x * 180 / math.pi for x in q], "#ffd000", "q")
        self.control_chart.clear()
        self.control_chart.add_line(t, elev, "#ff9f00", "elevator")
        self.control_chart.add_line(t, thr, "#00ff66", "throttle")
        self.mass_chart.clear()
        self.mass_chart.add_line(t, mass, "#00dfff", "mass")
        self.trajectory.clear()
        self.trajectory.add_line(pos, h, "#ffe000", "carrier")
        for row in res.cargo:
            try:
                if float(row.get("released", "0")) > 0.5:
                    x0 = float(row["release_x_est_m"])
                    hh = float(row["release_h_est_m"])
                    xi = float(row["truth_impact_m"])
                    xs = [x0 + (xi - x0) * i / 40 for i in range(41)]
                    ys = [hh * (1 - (i / 40) ** 2) for i in range(41)]
                    self.trajectory.add_line(xs, ys, "#ffe000", "cargo")
            except Exception:
                pass
        if theta:
            self.attitude.set_pitch(theta[-1] * 180 / math.pi)
        if va:
            err = abs(va[-1] - cfg.target_speed_mps)
            self.energy.set_value(max(0, min(1, 1 - err / max(1, cfg.target_speed_mps * 0.2))))
        if rec:
            self.damping.setText(f"Damping Wall\nrecovery={rec[-1]:.3f}")
        self.banner.setText(
            f"完成 · {cfg.backend_label} · H={cfg.target_altitude_m:g}m · V={cfg.target_speed_mps:g}m/s · {len(res.cargo)} cargo records"
        )