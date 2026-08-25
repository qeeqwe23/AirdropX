from __future__ import annotations

from math import isfinite
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QWidget


class LineChart(QWidget):
    def __init__(self, title: str = "", y_label: str = "", parent=None):
        super().__init__(parent)
        self.title = title
        self.y_label = y_label
        self.lines: list[tuple[list[float], list[float], QColor, str, Qt.PenStyle]] = []
        self.markers: list[tuple[float, QColor, str]] = []
        self.setMinimumHeight(180)

    def clear(self):
        self.lines.clear()
        self.markers.clear()
        self.update()

    def add_line(self, x, y, color="#00d4ff", label="", style=Qt.PenStyle.SolidLine):
        self.lines.append((list(x), list(y), QColor(color), label, style))
        self.update()

    def add_marker(self, x: float, color="#ff9f00", label=""):
        self.markers.append((float(x), QColor(color), label))
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#101010"))
        r = self.rect().adjusted(50, 30, -18, -35)
        p.setPen(QPen(QColor("#555"), 1))
        p.drawRect(r)
        p.setPen(QColor("#bdbdbd"))
        p.drawText(8, 18, self.title)
        allx = []
        ally = []
        for x, y, _, _, _ in self.lines:
            allx += [v for v in x if isfinite(v)]
            ally += [v for v in y if isfinite(v)]
        if not allx or not ally:
            p.setPen(QColor("#666"))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "等待仿真数据")
            return
        xmin, xmax = min(allx), max(allx)
        ymin, ymax = min(ally), max(ally)
        if xmax <= xmin:
            xmax = xmin + 1
        if ymax <= ymin:
            ymax = ymin + 1
        pad = (ymax - ymin) * 0.08
        ymin -= pad
        ymax += pad
        for i in range(6):
            yy = r.top() + r.height() * i / 5
            p.setPen(QPen(QColor("#252525"), 1))
            p.drawLine(r.left(), int(yy), r.right(), int(yy))
            val = ymax - (ymax - ymin) * i / 5
            p.setPen(QColor("#888"))
            p.drawText(4, int(yy + 4), f"{val:.2f}")

        def mappt(xv, yv):
            return QPointF(
                r.left() + (xv - xmin) / (xmax - xmin) * r.width(),
                r.bottom() - (yv - ymin) / (ymax - ymin) * r.height(),
            )

        for x, y, c, _label, style in self.lines:
            pts = [mappt(a, b) for a, b in zip(x, y) if isfinite(a) and isfinite(b)]
            if len(pts) >= 2:
                p.setPen(QPen(c, 1.5, style))
                p.drawPolyline(QPolygonF(pts))
        for xv, c, _label in self.markers:
            if xmin <= xv <= xmax:
                xx = r.left() + (xv - xmin) / (xmax - xmin) * r.width()
                p.setPen(QPen(c, 1.5, Qt.PenStyle.DashLine))
                p.drawLine(int(xx), r.top(), int(xx), r.bottom())
        p.setPen(QColor("#00dfff"))
        p.drawText(r.left() + r.width() // 2 - 25, self.height() - 8, "时间 (s)")


class ScatterChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.setMinimumHeight(300)

    def set_points(self, points):
        self.points = list(points)
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#101010"))
        r = self.rect().adjusted(45, 25, -15, -35)
        p.setPen(QColor("#555"))
        p.drawRect(r)
        p.setPen(QColor("#00dfff"))
        p.drawText(8, 16, "载荷落点误差分布（沿航迹）")
        span = max([abs(x) for x in self.points] + [20.0])
        span *= 1.2
        cx = r.center().x()
        cy = r.center().y()
        p.setPen(QPen(QColor("#777"), 1))
        p.drawLine(r.left(), int(cy), r.right(), int(cy))
        p.drawLine(int(cx), r.top(), int(cx), r.bottom())
        p.setPen(QColor("#00ff66"))
        p.setBrush(QColor("#00ff66"))
        for i, x in enumerate(self.points):
            px = cx + (x / span) * (r.width() / 2)
            py = cy + ((i % 3) - 1) * 7
            p.drawEllipse(QPointF(px, py), 4, 4)
        p.setPen(QColor("#aaa"))
        p.drawText(r.left(), r.bottom() + 22, f"-{span:.1f} m")
        p.drawText(r.right() - 50, r.bottom() + 22, f"+{span:.1f} m")


class AttitudeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0
        self.setMinimumSize(130, 100)

    def set_pitch(self, pitch_deg):
        self.pitch = float(pitch_deg)
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height()) - 20
        r = QRectF((self.width() - side) / 2, (self.height() - side) / 2, side, side)
        p.setPen(QPen(QColor("#00c9ff"), 1))
        p.setBrush(QColor("#0a4a79"))
        p.drawEllipse(r)
        offset = max(-side / 3, min(side / 3, self.pitch * 1.5))
        y = r.center().y() + offset
        p.setPen(QPen(QColor("#fff"), 2))
        p.drawLine(int(r.left() + 8), int(y), int(r.right() - 8), int(y))
        p.setPen(QColor("#00eaff"))
        p.drawText(4, 14, f"Pitch: {self.pitch:+.1f}°")


class EnergyGauge(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.value = 0.0
        self.setMinimumHeight(65)

    def set_value(self, v):
        self.value = max(0.0, min(1.0, float(v)))
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#101010"))
        p.setPen(QColor("#00dfff"))
        p.drawText(5, 15, "Energy Gauge")
        r = self.rect().adjusted(5, 30, -5, -12)
        p.setPen(QColor("#444"))
        p.drawRect(r)
        p.fillRect(r.adjusted(1, 1, -int(r.width() * (1 - self.value)) - 1, -1), QColor("#ffd000"))