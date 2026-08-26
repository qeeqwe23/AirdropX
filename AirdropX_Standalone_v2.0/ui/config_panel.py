from __future__ import annotations
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget,QVBoxLayout,QLabel,QGroupBox,QGridLayout,QDoubleSpinBox,QComboBox,QPushButton,QFormLayout
from core.app_config import *

class ConfigPanel(QWidget):
    start_requested=pyqtSignal(object); stop_requested=pyqtSignal(); reset_requested=pyqtSignal()
    def __init__(self,parent=None):
        super().__init__(parent); self.setMinimumWidth(330); self.setMaximumWidth(390); root=QVBoxLayout(self); root.setContentsMargins(7,4,7,4)
        h=QLabel('【任务配置】'); h.setObjectName('Section'); root.addWidget(h)
        box=QGroupBox('已验证飞行包线'); g=QGridLayout(box)
        self.alt=self._spin(VALIDATED_ALTITUDE_MIN_M,VALIDATED_ALTITUDE_MAX_M,200,1,' m'); self.speed=self._spin(VALIDATED_SPEED_MIN_MPS,VALIDATED_SPEED_MAX_MPS,50,.5,' m/s')
        g.addWidget(QLabel('目标高度'),0,0); g.addWidget(self.alt,0,1); g.addWidget(QLabel('目标速度'),1,0); g.addWidget(self.speed,1,1)
        self.envelope=QLabel('允许范围：H 20-200 m · V 45-65 m/s\n超出范围不可启动，运行时不生成未知模型。'); self.envelope.setWordWrap(True); self.envelope.setObjectName('Good'); g.addWidget(self.envelope,2,0,1,2); root.addWidget(box)

        wb=QGroupBox('风场'); wg=QGridLayout(wb)
        self.mode=QComboBox(); self.mode.addItems(['自定义风场','论文验证预设'])
        self.kind=QComboBox(); self.kind.addItems(CUSTOM_WIND_TYPES.keys())
        self.formal=QComboBox(); self.formal.addItems(FORMAL_SCENARIOS.keys()); self.formal.hide()
        self.ws=self._spin(0,20,0,.5,' m/s')
        self.wdir=QComboBox(); self.wdir.addItems(['顺风','逆风'])
        self.projected=QLabel('沿航迹: +0.00 m/s'); self.projected.setObjectName('Good')
        self.wind_note=QLabel('当前为纵向 Physics-MPC：自定义风只提供沿航迹分量，顺风为正、逆风为负。'); self.wind_note.setWordWrap(True)
        for row,(name,w) in enumerate([('模式',self.mode),('风型',self.kind),('论文预设',self.formal),('风速',self.ws),('方向',self.wdir),('',self.projected)]): wg.addWidget(QLabel(name),row,0); wg.addWidget(w,row,1)
        wg.addWidget(self.wind_note,6,0,1,2); root.addWidget(wb)

        mb=QGroupBox('任务'); mf=QFormLayout(mb); self.duration=self._spin(30,120,55,5,' s'); self.factor=QComboBox(); self.factor.addItems(['1x 实时','2x','5x']); mf.addRow('任务时长',self.duration); mf.addRow('显示倍率',self.factor); root.addWidget(mb)

        tb=QGroupBox('投放目标'); tg=QGridLayout(tb); defaults=[1200,1280,1360,1440]; self.targets=[]
        for i,val in enumerate(defaults):
            spin=self._spin(100,6500,val,10,' m'); self.targets.append(spin)
            tg.addWidget(QLabel(f'T{i+1}'),i//2,(i%2)*2); tg.addWidget(spin,i//2,(i%2)*2+1)
        note=QLabel('4 个目标需递增，相邻至少 20 m；最后目标需在当前速度/时长可达范围内。'); note.setWordWrap(True); tg.addWidget(note,2,0,1,4); root.addWidget(tb)

        info=QGroupBox('独立运行时'); il=QVBoxLayout(info); lab=QLabel('Physics-MPC v1.3.6-Paper 数值移植\nJSBSim Python/native runtime\n运行时不调用 MATLAB / MEX / .mat'); lab.setObjectName('Good'); lab.setWordWrap(True); il.addWidget(lab); root.addWidget(info)
        self.start=QPushButton('启动实时仿真'); self.stop=QPushButton('停止'); self.reset=QPushButton('重置'); self.stop.setEnabled(False); root.addWidget(self.start); root.addWidget(self.stop); root.addWidget(self.reset); root.addStretch(1)
        self.mode.currentIndexChanged.connect(self._mode); self.kind.currentIndexChanged.connect(self._proj); self.ws.valueChanged.connect(self._proj); self.wdir.currentIndexChanged.connect(self._proj); self.start.clicked.connect(lambda:self.start_requested.emit(self.get_config())); self.stop.clicked.connect(self.stop_requested); self.reset.clicked.connect(self.reset_requested); self._mode(0)
    def _spin(self,a,b,v,step,suf): w=QDoubleSpinBox(); w.setRange(a,b); w.setDecimals(2); w.setSingleStep(step); w.setValue(v); w.setSuffix(suf); return w
    def _direction_from_deg(self): return 180.0 if self.wdir.currentIndex()==0 else 0.0
    def _mode(self,i):
        custom=i==0; self.kind.setVisible(custom); self.formal.setVisible(not custom); self.ws.setEnabled(custom); self.wdir.setEnabled(custom); self._proj()
    def _proj(self):
        speed=self.ws.value()
        try:
            calm=CUSTOM_WIND_TYPES[self.kind.currentText()]=='calm'
        except KeyError:
            calm=False
        along=0.0 if calm else WindConfig(speed_mps=speed,direction_from_deg=self._direction_from_deg()).along_track_mps()
        self.projected.setText(f'沿航迹: {along:+.2f} m/s')
    def get_config(self):
        custom=self.mode.currentIndex()==0
        kind=CUSTOM_WIND_TYPES[self.kind.currentText()] if custom else FORMAL_SCENARIOS[self.formal.currentText()]
        wind=WindConfig(
            mode='custom' if custom else 'formal',
            kind=kind,
            speed_mps=self.ws.value(),
            direction_from_deg=self._direction_from_deg(),
            random_seed=2401 if custom else FORMAL_SENSOR_SEEDS[kind],
        )
        sensor_seed=2401 if custom else FORMAL_SENSOR_SEEDS[kind]
        factor=[1.,2.,5.][self.factor.currentIndex()]
        return MissionConfig(
            target_altitude_m=self.alt.value(),
            target_speed_mps=self.speed.value(),
            duration_s=self.duration.value(),
            target_positions_m=[w.value() for w in self.targets],
            sensor_noise_seed=sensor_seed,
            wind=wind,
            realtime_factor=factor,
        )
    def set_running(self,r):
        self.start.setEnabled(not r); self.stop.setEnabled(r); self.reset.setEnabled(not r)
        controls=(self.alt,self.speed,self.mode,self.kind,self.formal,self.ws,self.wdir,self.duration,self.factor,*self.targets)
        for w in controls: w.setEnabled(not r if w not in (self.ws,self.wdir) else (not r and self.mode.currentIndex()==0))