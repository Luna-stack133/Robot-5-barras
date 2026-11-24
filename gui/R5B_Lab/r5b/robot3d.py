"""Gemelo 3D del robot dibujado con QPainter (proyección ortográfica, sin OpenGL).

- Arrastrar con el botón izquierdo: rotar la cámara.
- Rueda del ratón: acercar / alejar.
- Doble clic: volver a la vista inicial.
"""
import math

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF

from . import kinematics as kin
from . import theme
from .config import C, H_CUP, H_DISTAL, H_PROXIMAL
from PySide6.QtWidgets import QWidget

VIEWS = {"3d": (-28.0, 32.0), "top": (0.0, 90.0), "front": (0.0, 6.0)}
def _target():
    """Punto al que mira la cámara [mm] (centro entre motores, hacia el área de trabajo)."""
    return (kin.GEOM["D"] / 2.0, 150.0, 40.0)


class Robot3DView(QWidget):
    def __init__(self, controller, parent=None, interactive=True, show_hud=True, show_trail=True):
        super().__init__(parent)
        self.ctrl = controller
        self.interactive = interactive
        self.show_hud = show_hud
        self.show_trail = show_trail
        self.az, self.el = VIEWS["3d"]
        self.zoom = 1.0
        self._drag = None
        self.setMinimumSize(360, 300)
        self.setMouseTracking(False)
        controller.pose_changed.connect(self.update)
        controller.status_changed.connect(self.update)
        controller.geometry_changed.connect(self.update)
        self._spin = QTimer(self)
        self._spin.setInterval(30)
        self._spin.timeout.connect(self._spin_step)

    # ------------------------------------------------------------ cámara
    def set_view(self, name):
        self.az, self.el = VIEWS.get(name, VIEWS["3d"])
        self.update()

    def auto_rotate(self, on=True):
        if on:
            self._spin.start()
        else:
            self._spin.stop()

    def _spin_step(self):
        self.az = (self.az + 0.35) % 360
        self.update()

    def _scale(self):
        return min(self.width() / 640.0, self.height() / 460.0) * self.zoom

    def proj(self, x, y, z):
        """Punto 3D [mm] -> (QPointF en pantalla, profundidad)."""
        tx, ty, tz = _target()
        x -= tx
        y -= ty
        z -= tz
        a, e = math.radians(self.az), math.radians(self.el)
        xr = x * math.cos(a) - y * math.sin(a)
        yr = x * math.sin(a) + y * math.cos(a)
        up = yr * math.sin(e) + z * math.cos(e)
        depth = yr * math.cos(e) - z * math.sin(e)
        s = self._scale()
        return QPointF(self.width() / 2 + s * xr, self.height() / 2 + 20 - s * up), depth

    def p(self, x, y, z):
        return self.proj(x, y, z)[0]

    # ------------------------------------------------------------ ratón
    def mousePressEvent(self, ev):
        if self.interactive and ev.button() == Qt.MouseButton.LeftButton:
            self._drag = ev.position()

    def mouseMoveEvent(self, ev):
        if self._drag is not None:
            d = ev.position() - self._drag
            self._drag = ev.position()
            self.az = (self.az - d.x() * 0.4) % 360
            self.el = max(3.0, min(90.0, self.el + d.y() * 0.3))
            self.update()

    def mouseReleaseEvent(self, _ev):
        self._drag = None

    def mouseDoubleClickEvent(self, _ev):
        if self.interactive:
            self.zoom = 1.0
            self.set_view("3d")

    def wheelEvent(self, ev):
        if self.interactive:
            f = 1.1 if ev.angleDelta().y() > 0 else 1 / 1.1
            self.zoom = max(0.5, min(3.0, self.zoom * f))
            self.update()

    # ------------------------------------------------------------ dibujo
    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.fillRect(self.rect(), QColor("#FAFBFC"))
        s = self._scale()
        self._floor(qp)
        self._paths(qp, s)

        g = self.ctrl.points()
        A, B, Cc, Dd, P = g["A"], g["B"], g["C"], g["D"], g["P"]
        hp, hd = H_PROXIMAL, H_DISTAL

        # sombras de los eslabones sobre la mesa
        shadow = QPen(QColor(22, 32, 42, 28), max(2.0, 15 * s))
        shadow.setCapStyle(Qt.PenCapStyle.RoundCap)
        shadow.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        qp.setPen(shadow)
        path = QPainterPath(self.p(A[0], A[1], 0))
        for pt in (Cc, P, Dd, B):
            path.lineTo(self.p(pt[0], pt[1], 0))
        qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawPath(path)

        # elementos con profundidad: se dibujan del más lejano al más cercano
        items = []
        for m in (A, B):
            items.append((self.proj(m[0], m[1], hp / 2)[1], lambda m=m: self._cylinder(qp, m, 0, hp - 6, 22,
                                                                                     C["motor"], C["motor_top"],
                                                                                     C["motor_bottom"])))
        links = [(A, Cc, hp), (B, Dd, hp), (Cc, P, hd), (Dd, P, hd)]
        for a, b, h in links:
            depth = (self.proj(a[0], a[1], h)[1] + self.proj(b[0], b[1], h)[1]) / 2
            items.append((depth, lambda a=a, b=b, h=h: self._bar(qp, a, b, h, s)))
        for j, h in ((A, hp), (B, hp), (Cc, (hp + hd) / 2), (Dd, (hp + hd) / 2)):
            items.append((self.proj(j[0], j[1], h)[1] - 0.5, lambda j=j, h=h: self._pin(qp, j, h, s)))
        items.append((self.proj(P[0], P[1], hd)[1] - 1, lambda: self._effector(qp, P, s)))
        for _, draw in sorted(items, key=lambda it: -it[0]):
            draw()

        if self.show_hud:
            self._hud(qp, A, B)
        qp.end()

    def _floor(self, qp):
        pen = QPen(QColor(C["grid"]), 1)
        qp.setPen(pen)
        for x in range(-250, 401, 50):
            qp.drawLine(self.p(x, -100, 0), self.p(x, 400, 0))
        for y in range(-100, 401, 50):
            qp.drawLine(self.p(-250, y, 0), self.p(400, y, 0))
        # placa base
        d = kin.GEOM["D"]
        poly = QPolygonF([self.p(-70, -45, 0), self.p(d + 70, -45, 0), self.p(d + 70, 35, 0), self.p(-70, 35, 0)])
        qp.setPen(QPen(QColor(C["border_strong"]), 1.2))
        qp.setBrush(QColor("#E7ECF0"))
        qp.drawPolygon(poly)
        # ejes
        ax = QPen(QColor("#B7C0CA"), 1.2)
        qp.setPen(ax)
        qp.drawLine(self.p(-250, 0, 0), self.p(400, 0, 0))
        qp.drawLine(self.p(0, -100, 0), self.p(0, 400, 0))

    def _paths(self, qp, s):
        ref = self.ctrl.preview
        if len(ref) > 1:
            pen = QPen(QColor("#8794A1"), 1.4, Qt.PenStyle.DashLine)
            qp.setPen(pen)
            qp.setBrush(Qt.BrushStyle.NoBrush)
            path = QPainterPath(self.p(ref[0][0], ref[0][1], 0.5))
            for x, y in ref[1:]:
                path.lineTo(self.p(x, y, 0.5))
            qp.drawPath(path)
        tr = self.ctrl.trail
        if self.show_trail and len(tr) > 1:
            pen = QPen(QColor(C["accent"]), max(1.6, 3 * s))
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            qp.setPen(pen)
            path = QPainterPath(self.p(tr[0][0], tr[0][1], 1))
            for x, y in tr[1:]:
                path.lineTo(self.p(x, y, 1))
            qp.drawPath(path)

    def _cylinder(self, qp, c, z0, z1, r, body, top, bottom):
        s = self._scale()
        t = self.p(c[0], c[1], z1)
        b = self.p(c[0], c[1], z0)
        rx = r * s
        ry = max(0.5, r * s * math.sin(math.radians(self.el)))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(bottom))
        qp.drawEllipse(b, rx, ry)
        qp.setBrush(QColor(body))
        qp.drawRect(QRectF(t.x() - rx, min(t.y(), b.y()), 2 * rx, abs(b.y() - t.y())))
        qp.setBrush(QColor(top))
        qp.drawEllipse(t, rx, ry)

    def _bar(self, qp, a, b, h, s):
        p1, p2 = self.p(a[0], a[1], h), self.p(b[0], b[1], h)
        dark = QPen(QColor(C["link_dark"]), max(3.0, 16 * s))
        dark.setCapStyle(Qt.PenCapStyle.RoundCap)
        qp.setPen(dark)
        qp.drawLine(p1, p2)
        light = QPen(QColor(C["link"]), max(1.5, 9 * s))
        light.setCapStyle(Qt.PenCapStyle.RoundCap)
        qp.setPen(light)
        off = QPointF(0, -max(0.5, 2 * s))
        qp.drawLine(p1 + off, p2 + off)

    def _pin(self, qp, j, h, s):
        qp.setPen(QPen(QColor(C["link_dark"]), max(1.5, 3 * s)))
        qp.setBrush(QColor("white"))
        r = max(3.0, 7 * s)
        qp.drawEllipse(self.p(j[0], j[1], h), r, r)

    def _effector(self, qp, P, s):
        top = self.p(P[0], P[1], H_DISTAL)
        bottom = self.p(P[0], P[1], H_CUP)
        rod = QPen(QColor(C["motor"]), max(2.0, 8 * s))
        rod.setCapStyle(Qt.PenCapStyle.RoundCap)
        qp.setPen(rod)
        qp.drawLine(top, bottom)
        cup = C["accent"] if self.ctrl.vacuum else C["motor_top"]
        self._cylinder(qp, P, 2, H_CUP, 13, cup, cup, C["motor_bottom"])
        qp.setPen(QPen(QColor("white"), max(1.5, 3 * s)))
        qp.setBrush(QColor(C["accent"]))
        r = max(4.0, 10 * s)
        qp.drawEllipse(top, r, r)

    def _hud(self, qp, A, B):
        qp.setFont(theme.mono(9))
        qp.setPen(QColor(C["muted"]))
        a = self.p(A[0], A[1], 0)
        b = self.p(B[0], B[1], 0)
        qp.drawText(a + QPointF(-60, 34), "θ₁ %.1f°" % self.ctrl.t1)
        qp.drawText(b + QPointF(18, 34), "θ₂ %.1f°" % self.ctrl.t2)
        if self.interactive:
            qp.setFont(theme.sans(8.5))
            qp.setPen(QColor(C["muted2"]))
            qp.drawText(QRectF(12, 10, self.width() - 24, 20), Qt.AlignmentFlag.AlignLeft,
                        "Arrastre para rotar · rueda para acercar · doble clic: vista inicial")
