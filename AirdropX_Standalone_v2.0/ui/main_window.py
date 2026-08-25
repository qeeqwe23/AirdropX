from __future__ import annotations
from pathlib import Path
import platform
from PyQt6.QtCore import QThread,pyqtSignal
from PyQt6.QtWidgets import QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QProgressBar,QSplitter,QMessageBox
from core.backend import StandaloneBackend
from core.app_config import APP_NAME,APP_VERSION,CONTROLLER_NAME
from .config_panel import ConfigPanel
from .monitor_panel import MonitorPanel
from .analysis_panel import AnalysisPanel
from .theme import STYLE

class Worker(QThread):
    log=pyqtSignal(str); progress=pyqtSignal(float); frame=pyqtSignal(object); done=pyqtSignal(object,object); failed=pyqtSignal(str)
    def __init__(self,backend,cfg): super().__init__(); self.backend=backend; self.cfg=cfg
    def run(self):
        try:
            root,summary=self.backend.run(self.cfg,self.frame.emit,self.log.emit,self.progress.emit); self.done.emit(str(root),summary)
        except Exception as e: self.failed.emit(str(e))

class MainWindow(QMainWindow):
    def __init__(self,app_root:Path):
        super().__init__(); self.root=Path(app_root); self.backend=None; self.worker=None; self.cfg=None; self.setWindowTitle(f'{APP_NAME} v{APP_VERSION}'); self.resize(1680,960); self.setStyleSheet(STYLE)
        c=QWidget(); outer=QVBoxLayout(c); outer.setContentsMargins(6,5,6,3); top=QHBoxLayout(); title=QLabel(f'AirdropX 独立 JSBSim 空投仿真软件 v{APP_VERSION}'); title.setObjectName('Title'); self.state=QLabel('待机 · MATLAB: 不使用'); self.state.setObjectName('Good'); top.addWidget(title); top.addStretch(); top.addWidget(self.state); outer.addLayout(top)
        self.progress=QProgressBar(); self.progress.setRange(0,100); self.progress.setMaximumHeight(14); outer.addWidget(self.progress)
        split=QSplitter(); self.config=ConfigPanel(); self.monitor=MonitorPanel(); self.analysis=AnalysisPanel(); split.addWidget(self.config); split.addWidget(self.monitor); split.addWidget(self.analysis); split.setStretchFactor(0,0); split.setStretchFactor(1,1); split.setStretchFactor(2,0); outer.addWidget(split,1); self.setCentralWidget(c)
        self.statusBar().showMessage(f'Python {platform.python_version()} | PyQt6 | JSBSim native | {CONTROLLER_NAME} | MATLAB runtime: NO')
        self.config.start_requested.connect(self.start); self.config.stop_requested.connect(self.stop); self.config.reset_requested.connect(self.reset)
    def _backend(self):
        if self.backend is None: self.backend=StandaloneBackend(self.root)
        return self.backend
    def start(self,cfg):
        e=cfg.validate()
        if e: QMessageBox.warning(self,'参数超出已验证范围','\n'.join(e)); return
        try:
            backend=self._backend()
        except Exception as exc:
            QMessageBox.critical(self,'AirdropX 独立运行时未就绪',str(exc)); return
        self.cfg=cfg; self.monitor.configure(cfg); self.analysis.reset(); self.config.set_running(True); self.state.setText(f'运行中 · {cfg.envelope_label} · MATLAB: NO'); self.progress.setValue(0)
        self.worker=Worker(backend,cfg); self.worker.log.connect(self.analysis.append_log); self.worker.progress.connect(lambda x:self.progress.setValue(int(round(100*x)))); self.worker.frame.connect(self._frame); self.worker.done.connect(self._done); self.worker.failed.connect(self._failed); self.worker.start()
    def _frame(self,f): self.monitor.append_frame(f); self.analysis.live(f)
    def stop(self):
        if self.backend: self.backend.stop(); self.analysis.append_log('[GUI] 已请求停止独立 JSBSim 仿真。')
    def reset(self):
        if self.worker and self.worker.isRunning(): self.stop()
        self.monitor.reset(); self.analysis.reset(); self.progress.setValue(0); self.state.setText('待机 · MATLAB: 不使用')
    def _done(self,root,summary):
        self.config.set_running(False)
        ok=bool(summary.get('software_success',False))
        self.progress.setValue(100 if ok else self.progress.value())
        self.state.setText(('完成' if ok else '已停止')+' · 独立 JSBSim · MATLAB: NO')
        self.analysis.done(summary,root)
    def _failed(self,msg): self.config.set_running(False); self.state.setText('运行失败'); self.analysis.append_log('[ERROR] '+msg); QMessageBox.critical(self,'AirdropX 独立后端错误',msg)
