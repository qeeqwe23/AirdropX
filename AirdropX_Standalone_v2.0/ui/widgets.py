from __future__ import annotations
from math import isfinite
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor,QPainter,QPen,QPolygonF,QBrush,QFont
from PyQt6.QtWidgets import QWidget

class TrajectoryView(QWidget):
    """Primary real-time along-track/altitude situation display."""
    def __init__(self,parent=None):
        super().__init__(parent); self.setMinimumSize(760,480); self.reset()
    def reset(self):
        self.x=[]; self.h=[]; self.targets=[]; self.cargo=[]; self.current=None; self.target_h=200.; self.expected_xmax=1600.; self.update()
    def configure(self,target_h,targets,expected_distance=None):
        self.target_h=float(target_h); self.targets=list(targets); self.expected_xmax=max(1400.,float(expected_distance or 0.),max(self.targets,default=0.)+250.); self.update()
    def append_frame(self,frame):
        self.x.append(float(frame.x_m)); self.h.append(float(frame.h_m)); self.current=frame; self.cargo=list(frame.cargo_paths); self.update()
    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.fillRect(self.rect(),QColor('#070a0c'))
        r=self.rect().adjusted(66,48,-26,-58); p.setPen(QPen(QColor('#33505a'),1)); p.drawRect(r)
        p.setPen(QColor('#00eaff')); f=QFont(p.font()); f.setPointSize(12); f.setBold(True); p.setFont(f); p.drawText(18,26,'实时任务轨迹 · Along-track / Altitude')
        if self.current:
            p.setPen(QColor('#d9faff')); sf=QFont(p.font()); sf.setPointSize(9); sf.setBold(True); p.setFont(sf)
            status=(f"t={self.current.t:4.1f}s  cfg={self.current.cfg}  "
                    f"x={self.current.x_m:6.1f}m  H={self.current.h_m:6.2f}m  "
                    f"Va={self.current.va_mps:5.2f}m/s  Wind={self.current.wind_mps:+5.2f}/est {self.current.wind_est_mps:+5.2f}m/s")
            p.drawText(330,26,status)
            p.setFont(f)
        xmax=max(self.expected_xmax,max(self.x,default=0.)+100.); xmin=0.; ymax=max(30.,self.target_h*1.22,max(self.h,default=0.)*1.12); ymin=0.
        def mp(x,y): return QPointF(r.left()+(x-xmin)/(xmax-xmin)*r.width(),r.bottom()-(y-ymin)/(ymax-ymin)*r.height())
        for i in range(6):
            yy=r.top()+i*r.height()/5; val=ymax*(1-i/5); p.setPen(QPen(QColor('#18262c'),1)); p.drawLine(r.left(),int(yy),r.right(),int(yy)); p.setPen(QColor('#779099')); p.drawText(8,int(yy+4),f'{val:5.0f} m')
        for i in range(8):
            xx=r.left()+i*r.width()/7; val=xmax*i/7; p.setPen(QPen(QColor('#122026'),1)); p.drawLine(int(xx),r.top(),int(xx),r.bottom()); p.setPen(QColor('#779099')); p.drawText(int(xx-18),r.bottom()+22,f'{val:.0f}')
        y=mp(0,self.target_h).y(); p.setPen(QPen(QColor('#3d8b45'),1,Qt.PenStyle.DashLine)); p.drawLine(r.left(),int(y),r.right(),int(y)); p.setPen(QColor('#5dce6a')); p.drawText(r.right()-135,int(y-6),f'Href {self.target_h:g} m')
        for i,t in enumerate(self.targets):
            q=mp(t,0); p.setPen(QPen(QColor('#ff5964'),2)); p.drawLine(int(q.x()),r.bottom()-12,int(q.x()),r.bottom()); p.drawText(int(q.x()-18),r.bottom()-18,f'T{i+1}')
        if len(self.x)>1:
            pts=QPolygonF([mp(a,b) for a,b in zip(self.x,self.h) if isfinite(a) and isfinite(b)]); p.setPen(QPen(QColor('#00dfff'),2.3)); p.drawPolyline(pts)
        for path in self.cargo:
            pts=QPolygonF([mp(a,b) for a,b in path]); p.setPen(QPen(QColor('#ffd84a'),1.8)); p.drawPolyline(pts)
        if self.current:
            q=mp(self.current.x_m,self.current.h_m); p.setBrush(QBrush(QColor('#ffffff'))); p.setPen(QPen(QColor('#00dfff'),2)); p.drawEllipse(q,6,6)
        p.setPen(QColor('#8fa7af')); p.drawText(r.center().x()-55,self.height()-14,'沿航迹距离 x (m)')

class MiniChart(QWidget):
    def __init__(self,title,parent=None):
        super().__init__(parent); self.title=title; self.data=[]; self.events=[]; self.setMinimumHeight(145)
    def reset(self): self.data=[]; self.update()
    def set_events(self,events): self.events=list(events or []); self.update()
    def append(self,t,y): self.data.append((float(t),float(y))); self.data=self.data[-800:]; self.update()
    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.fillRect(self.rect(),QColor('#0b0f11')); r=self.rect().adjusted(42,28,-12,-28); p.setPen(QColor('#40505a')); p.drawRect(r); p.setPen(QColor('#a9c2cb')); p.drawText(8,17,self.title)
        vals=[v for _,v in self.data if isfinite(v)]
        if len(vals)<2: return
        ymin,ymax=min(vals),max(vals); d=max(1e-6,ymax-ymin); ymin-=.1*d; ymax+=.1*d; t0=self.data[0][0]; t1=max(t0+1e-6,self.data[-1][0])
        for ev_t,label in self.events:
            if t0-1e-9 <= ev_t <= t1+1e-9:
                x=int(r.left()+(ev_t-t0)/(t1-t0)*r.width())
                p.setPen(QPen(QColor('#ffb84a'),1,Qt.PenStyle.DashLine)); p.drawLine(x,r.top(),x,r.bottom())
                p.setPen(QColor('#ffcf75')); short=str(label).split()[0]
                p.drawText(x+3,r.top()+12,f'{ev_t:g}s {short}')
        pts=[]
        for t,v in self.data:
            if isfinite(v): pts.append(QPointF(r.left()+(t-t0)/(t1-t0)*r.width(),r.bottom()-(v-ymin)/(ymax-ymin)*r.height()))
        p.setPen(QPen(QColor('#00dfff'),1.5)); p.drawPolyline(QPolygonF(pts)); p.setPen(QColor('#72858e')); p.drawText(3,r.top()+8,f'{ymax:.2f}'); p.drawText(3,r.bottom(),f'{ymin:.2f}')

class ImpactScatter(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent); self.items=[]; self.setMinimumHeight(230)
    def reset(self): self.items=[]; self.update()
    def set_impacts(self,items): self.items=list(items or []); self.update()
    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.fillRect(self.rect(),QColor('#0b0f11'))
        r=self.rect().adjusted(44,34,-18,-34); p.setPen(QColor('#40505a')); p.drawRect(r)
        p.setPen(QColor('#a9c2cb')); p.drawText(8,20,'落点误差 Monte Carlo (m)')
        if not self.items:
            p.setPen(QColor('#6f8790')); p.drawText(r.center().x()-28,r.center().y(),'等待投放')
            return
        errors=[]
        for item in self.items:
            target=float(item.get('target_m',0.0))
            errors.extend([float(x)-target for x in item.get('samples_m',[]) if isfinite(float(x))])
            for key in ('predicted_impact_m','truth_impact_m'):
                val=float(item.get(key,float('nan')))
                if isfinite(val): errors.append(val-target)
        half=max(5.0,min(80.0,max(abs(x) for x in errors)*1.15 if errors else 5.0))
        def mx(err): return r.left()+((err+half)/(2*half))*r.width()
        p.setPen(QPen(QColor('#27414a'),1));
        for frac,label in ((0.0,f'{-half:.0f}'),(0.5,'0'),(1.0,f'{half:.0f}')):
            x=int(r.left()+frac*r.width()); p.drawLine(x,r.top(),x,r.bottom()); p.setPen(QColor('#72858e')); p.drawText(x-10,r.bottom()+18,label); p.setPen(QPen(QColor('#27414a'),1))
        rows=4; row_h=r.height()/rows
        for i in range(rows):
            y=r.top()+row_h*(i+0.5)
            p.setPen(QPen(QColor('#18262c'),1)); p.drawLine(r.left(),int(y),r.right(),int(y))
            p.setPen(QColor('#ff5964')); p.drawText(8,int(y+4),f'T{i+1}')
        p.setPen(QPen(QColor('#5a6970'),1)); p.setBrush(QBrush(QColor('#8a9aa0')))
        for item in self.items:
            idx=int(item.get('index',1)); target=float(item.get('target_m',0.0)); y=r.top()+row_h*(idx-0.5)
            for xval in item.get('samples_m',[]):
                err=float(xval)-target
                if isfinite(err): p.drawEllipse(QPointF(mx(err),y),1.7,1.7)
        for item in self.items:
            idx=int(item.get('index',1)); target=float(item.get('target_m',0.0)); y=r.top()+row_h*(idx-0.5)
            pred=float(item.get('predicted_impact_m',float('nan'))); truth=float(item.get('truth_impact_m',float('nan')))
            if isfinite(pred):
                p.setBrush(QBrush(QColor('#00dfff'))); p.setPen(QPen(QColor('#04242c'),1)); p.drawEllipse(QPointF(mx(pred-target),y-5),4.2,4.2)
            if isfinite(truth):
                p.setBrush(QBrush(QColor('#ff5964'))); p.setPen(QPen(QColor('#2c0508'),1)); p.drawEllipse(QPointF(mx(truth-target),y+5),4.2,4.2)
                p.setPen(QColor('#d9faff')); p.drawText(int(r.right()-74),int(y+4),f'{truth-target:+.2f} m')
        p.setPen(QColor('#00dfff')); p.drawText(r.left()+4,self.height()-10,'青=预测')
        p.setPen(QColor('#ff5964')); p.drawText(r.left()+72,self.height()-10,'红=真实')
        p.setPen(QColor('#8a9aa0')); p.drawText(r.left()+136,self.height()-10,'灰=MC样本')
