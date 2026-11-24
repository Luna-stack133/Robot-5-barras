"""Componentes reutilizables de la interfaz."""
from PySide6.QtCore import QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (QAbstractButton, QButtonGroup, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel,
                               QPushButton, QSizePolicy, QVBoxLayout, QWidget)

from .config import C
from .icons import icon


def set_prop(widget, name, value):
    """Cambia una propiedad usada por la hoja de estilos y refresca el estilo."""
    widget.setProperty(name, value)
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def label(text, name=None, wrap=False):
    lab = QLabel(text)
    if name:
        lab.setObjectName(name)
    lab.setWordWrap(wrap)
    return lab


def button(text, variant=None, on_click=None, icon_name=None, icon_color=None):
    b = QPushButton(text)
    if variant:
        b.setProperty("variant", variant)
    if icon_name:
        b.setIcon(icon(icon_name, icon_color or C["text"], 16))
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    if on_click:
        b.clicked.connect(lambda *_: on_click())   # la función se llama sin argumentos
    return b


class Card(QFrame):
    """Tarjeta blanca con borde redondeado. `body` es su QVBoxLayout."""

    def __init__(self, parent=None, margins=(18, 16, 18, 16), spacing=12, soft=False):
        super().__init__(parent)
        self.setObjectName("CardSoft" if soft else "Card")
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(*margins)
        self.body.setSpacing(spacing)


def divider():
    d = QFrame()
    d.setObjectName("Divider")
    d.setFrameShape(QFrame.Shape.NoFrame)
    return d


class Field(QWidget):
    """Etiqueta pequeña arriba y control abajo."""

    def __init__(self, text, widget, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(5)
        lab = QLabel(text)
        lab.setObjectName("Muted")
        lay.addWidget(lab)
        lay.addWidget(widget)
        self.widget = widget


def spin(value, lo, hi, suffix="", decimals=1, step=1.0):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(value)
    if suffix:
        s.setSuffix(suffix)
    s.setKeyboardTracking(False)
    return s


class ToggleSwitch(QAbstractButton):
    """Interruptor tipo switch (encendido/apagado)."""

    def __init__(self, parent=None, checked=False):
        super().__init__(parent)
        self.setCheckable(True)
        self.setChecked(checked)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedSize(52, 30)

    def sizeHint(self):
        return QSize(52, 30)

    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        on = self.isChecked()
        track = QColor(C["accent"] if on else "#B7C0CA")
        if not self.isEnabled():
            track = QColor("#DDE2E7")
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(track)
        qp.drawRoundedRect(QRectF(0, 0, 52, 30), 15, 15)
        qp.setBrush(QColor("white"))
        x = 25 if on else 3
        qp.drawEllipse(QRectF(x, 3, 24, 24))
        qp.end()


class StatusDot(QWidget):
    def __init__(self, color="#8794A1", size=10, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self.setFixedSize(size, size)

    def set_color(self, color):
        self._color = QColor(color)
        self.update()

    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(self._color)
        qp.drawEllipse(self.rect())
        qp.end()


class Segmented(QFrame):
    """Grupo de botones exclusivos (como un interruptor de varias posiciones)."""

    changed = Signal(str)

    def __init__(self, options, current=None, variant="seg", parent=None):
        """options: lista de (clave, texto)."""
        super().__init__(parent)
        if variant == "seg":
            self.setObjectName("Segmented")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3) if variant == "seg" else lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2 if variant == "seg" else 6)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        self.buttons = {}
        for key, text in options:
            b = QPushButton(text)
            b.setProperty("variant", variant)
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _=False, k=key: self.changed.emit(k))
            self.group.addButton(b)
            lay.addWidget(b)
            self.buttons[key] = b
        self.set_current(current or options[0][0])

    def set_current(self, key):
        if key in self.buttons:
            self.buttons[key].setChecked(True)

    def current(self):
        for k, b in self.buttons.items():
            if b.isChecked():
                return k
        return None


class EStopButton(QPushButton):
    def __init__(self, on_click, parent=None):
        super().__init__("  PARADA DE EMERGENCIA", parent)
        self.setProperty("variant", "danger")
        self.setIcon(icon("stop_octagon", "white", 16))
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Detiene el robot y deshabilita los motores (tecla Esc)")
        self.clicked.connect(on_click)


class PageHeader(QWidget):
    """Título y subtítulo de la página, con acciones a la derecha."""

    def __init__(self, title, subtitle, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        txt = QVBoxLayout()
        txt.setSpacing(2)
        txt.addWidget(label(title, "H1"))
        txt.addWidget(label(subtitle, "Muted"))
        lay.addLayout(txt)
        lay.addStretch(1)
        self.actions = lay

    def add(self, widget):
        self.actions.addWidget(widget)


class MessageLabel(QLabel):
    """Mensaje de estado con color según el tipo (normal, ok, error, warn)."""

    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self.setObjectName("Msg")
        self.setWordWrap(True)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)

    def show_message(self, text, state=""):
        self.setText(text)
        set_prop(self, "state", state)


class Readout(QWidget):
    """Lectura numérica: rótulo pequeño y valor grande en fuente monoespaciada."""

    def __init__(self, caption, value="—", big=False, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        lay.addWidget(label(caption, "Small"))
        self.value = label(value, "ReadoutBig" if big else "Readout")
        lay.addWidget(self.value)

    def set(self, text):
        self.value.setText(text)
