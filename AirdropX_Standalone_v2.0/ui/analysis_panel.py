from __future__ import annotations
from PyQt6.QtWidgets import QWidget,QVBoxLayout,QLabel,QGroupBox,QTextEdit
from .widgets import ImpactScatter

class AnalysisPanel(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent); self.setMinimumWidth(330); self.setMaximumWidth(420); lay=QVBoxLayout(self)
        h=QLabel('【运行状态】'); h.setObjectName('Section'); lay.addWidget(h)
        b=QGroupBox('独立软件状态'); v=QVBoxLayout(b)
        self.runtime=QLabel('运行时: 等待'); self.runtime.setObjectName('Good')
        self.ctrl=QLabel('控制器: Physics-MPC v1.3.6-Paper')
        self.env=QLabel('包线: H20-200 / V45-65')
        self.drops=QLabel('投放: 0 / 4')
        self.qp=QLabel('QP: --')
        self.result=QLabel('结果: --'); self.result.setWordWrap(True)
        for w in (self.runtime,self.ctrl,self.env,self.drops,self.qp,self.result): v.addWidget(w)
        lay.addWidget(b)
        sb=QGroupBox('投放蒙特卡洛散点'); sl=QVBoxLayout(sb); self.scatter=ImpactScatter(); sl.addWidget(self.scatter); lay.addWidget(sb)
        lb=QGroupBox('事件日志'); ll=QVBoxLayout(lb); self.log=QTextEdit(); self.log.setReadOnly(True); ll.addWidget(self.log); lay.addWidget(lb,1)
    def append_log(self,s): self.log.append(str(s))
    def live(self,f):
        self.drops.setText(f'当前载荷配置: cfg{f.cfg} / 4')
        self.scatter.set_impacts(getattr(f,'impact_scatters',[]))
    def done(self,summary,root):
        ok=bool(summary.get('software_success',False)); stopped=bool(summary.get('stopped_by_user',False))
        self.runtime.setText(('运行时: SUCCESS' if ok else ('运行时: 用户停止' if stopped else '运行时: 未完成'))+' · MATLAB=NO · JSBSim=YES')
        self.drops.setText(f"投放: {summary.get('drops_completed',0)} / 4"); self.qp.setText(f"QP success: {summary.get('qp_success_fraction',0):.3f}")
        e=summary.get('max_landing_error_m')
        self.result.setText(f"结果目录: {root}\n最大落点误差: {e:.3f} m" if isinstance(e,(int,float)) else f'结果目录: {root}')
    def reset(self):
        self.runtime.setText('运行时: 等待'); self.drops.setText('投放: 0 / 4'); self.qp.setText('QP: --'); self.result.setText('结果: --'); self.log.clear(); self.scatter.reset()
