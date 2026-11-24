"""Iconos de línea dibujados con QPainter (sin archivos de imagen)."""
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap


def _path(name):
    """Trazo del icono en una cuadrícula de 24 x 24."""
    p = QPainterPath()
    if name == "monitor":                       # pulso
        p.moveTo(3, 12); p.lineTo(7, 12); p.lineTo(10, 4); p.lineTo(14, 20); p.lineTo(17, 12); p.lineTo(21, 12)
    elif name == "path":                        # recorrido entre dos puntos
        p.addEllipse(QPointF(5, 19), 2, 2)
        p.addEllipse(QPointF(19, 5), 2, 2)
        p.moveTo(7, 18); p.cubicTo(13, 18, 10, 6, 17, 6)
    elif name == "control":                     # bloques del lazo
        p.addRoundedRect(QRectF(3, 8, 6, 8), 1, 1)
        p.addRoundedRect(QRectF(15, 8, 6, 8), 1, 1)
        p.moveTo(9, 12); p.lineTo(15, 12)
        p.moveTo(18, 16); p.lineTo(18, 20); p.lineTo(6, 20); p.lineTo(6, 16)
    elif name == "results":                     # barras
        p.moveTo(4, 20); p.lineTo(4, 10)
        p.moveTo(10, 20); p.lineTo(10, 4)
        p.moveTo(16, 20); p.lineTo(16, 13)
        p.moveTo(2, 20); p.lineTo(22, 20)
    elif name == "refresh":
        p.arcMoveTo(QRectF(4, 4, 16, 16), 0)
        p.arcTo(QRectF(4, 4, 16, 16), 0, 300)
        p.moveTo(20, 4); p.lineTo(20, 9); p.lineTo(15, 9)
    elif name == "arrow":
        p.moveTo(5, 12); p.lineTo(19, 12)
        p.moveTo(13, 6); p.lineTo(19, 12); p.lineTo(13, 18)
    elif name == "check":
        p.addEllipse(QPointF(12, 12), 9, 9)
        p.moveTo(8, 12); p.lineTo(11, 15); p.lineTo(16, 9)
    elif name == "plug":
        p.moveTo(9, 3); p.lineTo(9, 8)
        p.moveTo(15, 3); p.lineTo(15, 8)
        p.addRoundedRect(QRectF(6, 8, 12, 6), 2, 2)
        p.moveTo(12, 14); p.lineTo(12, 21)
    elif name == "settings":
        p.addEllipse(QPointF(12, 12), 3, 3)
        p.addEllipse(QPointF(12, 12), 8, 8)
    elif name == "stop_octagon":
        pts = [(8, 3), (16, 3), (21, 8), (21, 16), (16, 21), (8, 21), (3, 16), (3, 8)]
        p.moveTo(*pts[0])
        for x, y in pts[1:]:
            p.lineTo(x, y)
        p.closeSubpath()
        p.moveTo(8.5, 12); p.lineTo(15.5, 12)
    elif name == "logo":                        # robot de 5 barras
        p.addEllipse(QPointF(7.5, 19), 2, 2)
        p.addEllipse(QPointF(16.5, 19), 2, 2)
        p.moveTo(7.5, 19); p.lineTo(3.5, 10); p.lineTo(12, 4); p.lineTo(20.5, 10); p.lineTo(16.5, 19)
    return p


def icon(name, color="#56636F", size=18, width=2.0):
    """QIcon de línea con el color dado."""
    return QIcon(pixmap(name, color, size, width))


def pixmap(name, color="#56636F", size=18, width=2.0, ratio=2.0):
    pm = QPixmap(int(size * ratio), int(size * ratio))
    pm.fill(Qt.GlobalColor.transparent)
    pm.setDevicePixelRatio(ratio)
    qp = QPainter(pm)
    qp.setRenderHint(QPainter.RenderHint.Antialiasing)
    qp.scale(size / 24.0, size / 24.0)
    pen = QPen(QColor(color))
    pen.setWidthF(width * 24.0 / size)   # grosor en píxeles reales, aunque la cuadrícula esté escalada
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    qp.setPen(pen)
    qp.setBrush(Qt.BrushStyle.NoBrush)
    qp.drawPath(_path(name))
    qp.end()
    return pm


def figure_pixmap(kind, color="#56636F", size=30):
    """Miniatura de una figura de trayectoria."""
    from .paths import figure_points
    pts = figure_points(kind, 0.0, 0.0, 20.0)
    ratio = 2.0
    pm = QPixmap(int(size * ratio), int(size * ratio))
    pm.fill(Qt.GlobalColor.transparent)
    pm.setDevicePixelRatio(ratio)
    qp = QPainter(pm)
    qp.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen = QPen(QColor(color), 2.0)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    qp.setPen(pen)
    s = size / 26.0
    path = QPainterPath()
    for i, (x, y) in enumerate(pts):
        pt = QPointF(size / 2 + x * s, size / 2 - y * s)
        if i == 0:
            path.moveTo(pt)
        else:
            path.lineTo(pt)
    qp.drawPath(path)
    qp.end()
    return pm
