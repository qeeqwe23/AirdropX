from __future__ import annotations
from PyQt6.QtWidgets import QWidget,QVBoxLayout,QLabel,QTabWidget,QHBoxLayout
from core.wind import wind_event_markers
from .widgets import TrajectoryView,MiniChart

class MonitorPanel(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent); lay=QVBoxLayout(self); lay.setContentsMargins(4,3,4,3)
        title=QLabel('【实时飞行与空投轨迹】'); title.setObjectName('Section'); lay.addWidget(title)
        self.trajectory=TrajectoryView(); lay.addWidget(self.trajectory,5)
        self.tabs=QTabWidget(); page=QWidget(); row=QHBoxLayout(page); self.h=MiniChart('高度 H (m)'); self.va=MiniChart('空速 Va (m/s)'); self.ctrl=MiniChart('升降舵 command'); row.addWidget(self.h); row.addWidget(self.va); row.addWidget(self.ctrl); self.tabs.addTab(page,'辅助实时曲线'); lay.addWidget(self.tabs,2)
    def configure(self,cfg):
        self.reset(); targets=[cfg.target_start_m+i*cfg.target_spacing_m for i in range(4)]
        self.trajectory.configure(cfg.target_altitude_m,targets,cfg.target_speed_mps*cfg.duration_s*1.05)
        events=wind_event_markers(cfg.wind,cfg.duration_s)
        for chart in (self.h,self.va,self.ctrl): chart.set_events(events)
    def append_frame(self,f): self.trajectory.append_frame(f); self.h.append(f.t,f.h_m); self.va.append(f.t,f.va_mps); self.ctrl.append(f.t,f.elevator)
    def reset(self): self.trajectory.reset(); self.h.reset(); self.va.reset(); self.ctrl.reset(); self.h.set_events([]); self.va.set_events([]); self.ctrl.set_events([])
