from __future__ import annotations

from pathlib import Path
import platform

from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QMainWindow, QMessageBox, QProgressBar, QSplitter, QVBoxLayout, QWidget

from core.backend import MatlabBackend
from core.result_loader import load_result
from .analysis_panel import AnalysisPanel
from .config_panel import ConfigPanel
from .monitor_panel import MonitorPanel
from .theme import STYLE


class SimulationWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(float)
    done = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, backend, cfg):
        super().__init__()
        self.backend = backend
        self.cfg = cfg

    def run(self):
        try:
            self.done.emit(str(self.backend.run(self.cfg, self.log.emit, self.progress.emit)))
        except Exception as exc:
            self.failed.emit(str(exc))


class MainWindow(QMainWindow):
    def __init__(self, app_root: Path):
        super().__init__()
        self.app_root = Path(app_root)
        self.backend = MatlabBackend(app_root)
        self.worker = None
        self.current_cfg = None
        self.setWindowTitle("AirdropX 空投仿真与测控软件 v1.0 · Physics-MPC v1.3.6-Paper")
        self.resize(1600, 900)
        self.setStyleSheet(STYLE)

        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(6, 5, 6, 3)
        outer.setSpacing(3)
        top = QHBoxLayout()
        title = QLabel("AirdropX MQ-9 Reaper Cargo 空投仿真与测控软件 v1.0")
        title.setObjectName("Title")
        self.state = QLabel("任务状态: 待机  控制器: Physics-MPC v1.3.6-Paper")
        self.state.setObjectName("Good")
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.state)
        outer.addLayout(top)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setMaximumHeight(14)
        outer.addWidget(self.progress)

        split = QSplitter()
        self.config = ConfigPanel()
        self.monitor = MonitorPanel()
        self.analysis = AnalysisPanel()
        split.addWidget(self.config)
        split.addWidget(self.monitor)
        split.addWidget(self.analysis)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        outer.addWidget(split, 1)
        self.setCentralWidget(central)

        self.statusBar().showMessage(
            f"Python {platform.python_version()} | PyQt6 | Offline | Physics-MPC v1.3.6-Paper"
        )
        self.config.start_requested.connect(self.start)
        self.config.stop_requested.connect(self.stop)
        self.config.reset_requested.connect(self.reset)

    def start(self, cfg):
        errs = cfg.validate()
        if errs:
            QMessageBox.warning(self, "参数错误", "\n".join(errs))
            return
        self.current_cfg = cfg
        self.config.set_running(True)
        self.state.setText("任务状态: 运行中  控制器: Physics-MPC v1.3.6-Paper")
        self.progress.setValue(1)
        self.analysis.append_log("[GUI] 启动 Physics-MPC v1.3.6-Paper...")
        self.worker = SimulationWorker(self.backend, cfg)
        self.worker.log.connect(self.analysis.append_log)
        self.worker.progress.connect(lambda x: self.progress.setValue(round(x * 100)))
        self.worker.done.connect(self._done)
        self.worker.failed.connect(self._failed)
        self.worker.start()

    def stop(self):
        self.backend.stop()
        self.analysis.append_log("[GUI] 已请求停止 MATLAB 后端。")

    def reset(self):
        if self.worker and self.worker.isRunning():
            self.stop()
        self.monitor.reset()
        self.analysis.reset()
        self.progress.setValue(0)
        self.state.setText("任务状态: 待机  控制器: Physics-MPC v1.3.6-Paper")

    def _done(self, root):
        try:
            result = load_result(root)
            self.monitor.show_result(result, self.current_cfg)
            self.analysis.show_result(result, self.current_cfg)
            paper = result.bool_from_summary("paper_core_pass")
            control_text = "PASS" if paper is True else ("FAIL" if paper is False else "--")
            self.state.setText(f"任务状态: 完成  论文 MPC: {control_text}")
            self.progress.setValue(100)
            self.analysis.append_log(f"[GUI] 结果已载入: {root}")
        except Exception as exc:
            self._failed(f"结果读取失败: {exc}")
            return
        self.config.set_running(False)

    def _failed(self, msg):
        self.config.set_running(False)
        self.state.setText("任务状态: 失败")
        self.analysis.append_log("[ERROR] " + msg)
        QMessageBox.critical(self, "AirdropX 后端错误", msg)