"""Mapa del espacio de trabajo (vista superior) con el recorrido, la posición actual y el destino."""
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen

from . import kinematics as kin
from . import theme
from .config import C, SIN_MIN
from PySide6.QtWidgets import QWidget

X_RANGE = (-240.0, 400.0)
Y_RANGE = (-50.0, 380.0)
GOOD_QUALITY = 0.45        # medida de singularidad por encima de la cual la zona es "recomendada"
RES = 4.0                  # mm por celda del mapa precalculado

_CACHE = {}


def _region_image():
    """Imagen del área de trabajo del modo activo (se recalcula si cambian geometría o modo)."""
    key = kin.STATE["version"]
    if _CACHE.get("key") == key:
        return _CACHE["img"]
    nx = int((X_RANGE[1] - X_RANGE[0]) / RES)
    ny = int((Y_RANGE[1] - Y_RANGE[0]) / RES)
    img = QImage(nx, ny, QImage.Format.Format_ARGB32)
    img.fill(QColor(0, 0, 0, 0))
    good, ok, bad = QColor("#CFE7E9"), QColor("#E9EDF1"), QColor("#F6D3D3")
    for j in range(ny):
        y = Y_RANGE[1] - (j + 0.5) * RES
        for i in range(nx):
            x = X_RANGE[0] + (i + 0.5) * RES
            q = kin.ik(x, y)
            if q is None:
                continue
            s = kin.sing_measure(x, y, q[0], q[1])
            img.setPixelColor(i, j, bad if s < SIN_MIN else (good if s >= GOOD_QUALITY else ok))
    _CACHE["key"], _CACHE["img"] = key, img
    return img


class WorkspaceMap(QWidget):
    clicked = Signal(float, float)     # (x, y) en mm
    hovered = Signal(float, float)

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.path = []          # recorrido propuesto [(x, y)]
        self.points = []        # puntos numerados [(x, y)]
        self.target = None      # destino (x, y)
        self.bad_point = None   # primer punto fuera de alcance
        self.path_ok = True
        self.corners = []       # puntos donde el robot frena (esquinas)
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.CrossCursor)
        controller.pose_changed.connect(self.update)
        controller.geometry_changed.connect(self.update)

    def set_overlay(self, path=None, points=None, target=None, ok=True, bad_point=None, corners=None):
        self.corners = list(corners or [])
        self.path = list(path or [])
        self.points = list(points or [])
        self.target = target
        self.path_ok = ok
        self.bad_point = bad_point
        self.update()

    # ------------------------------------------------------------ coordenadas
    def _frame(self):
        w, h = self.width() - 24, self.height() - 24
        sx = w / (X_RANGE[1] - X_RANGE[0])
        sy = h / (Y_RANGE[1] - Y_RANGE[0])
        s = min(sx, sy)
        ox = 12 + (w - s * (X_RANGE[1] - X_RANGE[0])) / 2
        oy = 12 + (h - s * (Y_RANGE[1] - Y_RANGE[0])) / 2
        return s, ox, oy

    def to_screen(self, x, y):
        s, ox, oy = self._frame()
        return QPointF(ox + (x - X_RANGE[0]) * s, oy + (Y_RANGE[1] - y) * s)

    def to_world(self, pt):
        s, ox, oy = self._frame()
        return X_RANGE[0] + (pt.x() - ox) / s, Y_RANGE[1] - (pt.y() - oy) / s

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(*self.to_world(ev.position()))

    def mouseMoveEvent(self, ev):
        self.hovered.emit(*self.to_world(ev.position()))

    # ------------------------------------------------------------ dibujo
    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.fillRect(self.rect(), QColor("#FAFBFC"))
        s, ox, oy = self._frame()
        area = QRectF(ox, oy, s * (X_RANGE[1] - X_RANGE[0]), s * (Y_RANGE[1] - Y_RANGE[0]))
        qp.drawImage(area, _region_image())

        # cuadrícula cada 50 mm
        qp.setPen(QPen(QColor(22, 32, 42, 22), 1))
        x = int(X_RANGE[0] // 50 + 1) * 50
        while x < X_RANGE[1]:
            qp.drawLine(self.to_screen(x, Y_RANGE[0]), self.to_screen(x, Y_RANGE[1]))
            x += 50
        y = int(Y_RANGE[0] // 50 + 1) * 50
        while y < Y_RANGE[1]:
            qp.drawLine(self.to_screen(X_RANGE[0], y), self.to_screen(X_RANGE[1], y))
            y += 50
        qp.setFont(theme.mono(8))
        qp.setPen(QColor(C["muted2"]))
        for xv in (-200, -100, 0, 100, 200, 300):
            qp.drawText(self.to_screen(xv, Y_RANGE[0]) + QPointF(3, -4), str(xv))
        for yv in (100, 200, 300):
            qp.drawText(self.to_screen(X_RANGE[0], yv) + QPointF(4, -3), str(yv))
        qp.drawText(self.to_screen(X_RANGE[1], Y_RANGE[0]) + QPointF(-46, -4), "x [mm]")
        qp.drawText(self.to_screen(X_RANGE[0], Y_RANGE[1]) + QPointF(4, 14), "y [mm]")

        # motores
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(C["motor"]))
        for mx in (0.0, kin.GEOM["D"]):
            c = self.to_screen(mx, 0)
            qp.drawRoundedRect(QRectF(c.x() - 9, c.y() - 9, 18, 18), 3, 3)

        # recorrido propuesto
        if len(self.path) > 1:
            pen = QPen(QColor(C["accent"] if self.path_ok else C["danger"]), 2.6)
            if not self.path_ok:
                pen.setStyle(Qt.PenStyle.DashLine)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            qp.setPen(pen)
            qp.setBrush(Qt.BrushStyle.NoBrush)
            path = QPainterPath(self.to_screen(*self.path[0]))
            for pt in self.path[1:]:
                path.lineTo(self.to_screen(*pt))
            qp.drawPath(path)

        # esquinas: puntos donde el robot frena
        if self.corners:
            qp.setPen(QPen(QColor(C["accent"]), 2))
            qp.setBrush(QColor("white"))
            for pt in self.corners:
                qp.drawEllipse(self.to_screen(*pt), 4.5, 4.5)

        # rastro de la ejecución
        tr = self.ctrl.trail
        if len(tr) > 1:
            qp.setPen(QPen(QColor(C["text"]), 1.4))
            path = QPainterPath(self.to_screen(*tr[0]))
            for pt in tr[1:]:
                path.lineTo(self.to_screen(*pt))
            qp.drawPath(path)

        # puntos numerados
        qp.setFont(theme.sans(8.5, QFont.Weight.DemiBold))
        for n, (px, py) in enumerate(self.points, 1):
            c = self.to_screen(px, py)
            qp.setPen(QPen(QColor(C["accent"]), 2))
            qp.setBrush(QColor("white"))
            qp.drawEllipse(c, 10, 10)
            qp.setPen(QColor(C["text"]))
            qp.drawText(QRectF(c.x() - 10, c.y() - 10, 20, 20), Qt.AlignmentFlag.AlignCenter, str(n))

        # destino
        if self.target is not None:
            c = self.to_screen(*self.target)
            pen = QPen(QColor(C["accent"]), 2)
            qp.setPen(pen)
            qp.setBrush(Qt.BrushStyle.NoBrush)
            qp.drawEllipse(c, 11, 11)
            qp.drawLine(c + QPointF(-17, 0), c + QPointF(17, 0))
            qp.drawLine(c + QPointF(0, -17), c + QPointF(0, 17))

        # primer punto fuera de alcance
        if self.bad_point is not None:
            c = self.to_screen(*self.bad_point)
            qp.setPen(QPen(QColor(C["danger"]), 3))
            qp.drawLine(c + QPointF(-8, -8), c + QPointF(8, 8))
            qp.drawLine(c + QPointF(-8, 8), c + QPointF(8, -8))

        # posición actual del efector
        x, y = self.ctrl.effector()
        c = self.to_screen(x, y)
        qp.setPen(QPen(QColor("white"), 2.5))
        qp.setBrush(QColor(C["accent"]))
        qp.drawEllipse(c, 7, 7)
        qp.end()
