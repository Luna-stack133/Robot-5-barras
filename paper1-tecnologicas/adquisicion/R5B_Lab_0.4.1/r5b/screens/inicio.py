"""Pantalla de inicio: presentación del robot y conexión con la ESP32."""
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QComboBox, QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .. import theme
from ..config import APP_NAME, BAUD_RATES, C, DEFAULT_BAUD
from ..icons import icon, pixmap
from ..robot3d import Robot3DView
from ..robot_link import available_ports
from ..widgets import Card, Field, StatusDot, button, label


class StartScreen(QWidget):
    start = Signal()

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setObjectName("Page")
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ---------------- izquierda: presentación
        left = QFrame()
        left.setStyleSheet("QFrame#StartLeft { background: white; border-right: 1px solid %s; }" % C["border"])
        left.setObjectName("StartLeft")
        lv = QVBoxLayout(left)
        lv.setContentsMargins(48, 40, 48, 40)
        lv.setSpacing(18)
        brand = QHBoxLayout()
        brand.setSpacing(12)
        logo = QLabel()
        logo.setPixmap(pixmap("logo", C["accent"], 40))
        brand.addWidget(logo)
        names = QVBoxLayout()
        names.setSpacing(0)
        names.addWidget(label(APP_NAME, "H2"))
        names.addWidget(label("Técnicas de Control · Ingeniería Mecatrónica", "Muted"))
        brand.addLayout(names)
        brand.addStretch(1)
        lv.addLayout(brand)

        self.view = Robot3DView(controller, interactive=True, show_hud=False, show_trail=False)
        self.view.zoom = 1.15
        self.view.auto_rotate(True)
        lv.addWidget(self.view, 1)

        title = QLabel("Robot paralelo de cinco barras")
        title.setFont(theme.sans(24, QFont.Weight.DemiBold))
        title.setWordWrap(True)
        lv.addWidget(title)
        lv.addWidget(label("Control de trayectoria con realimentación IMU + EKF, movimiento manual "
                           "y supervisión en vivo.", "Muted", wrap=True))
        root.addWidget(left, 3)

        # ---------------- derecha: conexión
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(40, 40, 40, 40)
        rv.addStretch(1)
        col = QWidget()
        col.setMaximumWidth(460)
        cv = QVBoxLayout(col)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(18)
        cv.addWidget(label("Conectar con el robot", "H1"))
        cv.addWidget(label("Elija el puerto donde está conectada la ESP32 que controla los servomotores.",
                           "Muted", wrap=True))

        card = Card(margins=(22, 20, 22, 20), spacing=14)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(240)
        row.addWidget(self.port_combo, 1)
        refresh = QPushButton()
        refresh.setIcon(icon("refresh", C["text"], 16))
        refresh.setToolTip("Buscar puertos")
        refresh.setFixedWidth(44)
        refresh.clicked.connect(lambda *_: self.refresh_ports())
        row.addWidget(refresh)
        holder = QWidget()
        holder.setLayout(row)
        row.setContentsMargins(0, 0, 0, 0)
        card.body.addWidget(Field("Puerto serial", holder))

        self.baud_combo = QComboBox()
        for b in BAUD_RATES:
            self.baud_combo.addItem(str(b), b)
        self.baud_combo.setCurrentIndex(max(0, self.baud_combo.findData(DEFAULT_BAUD)))
        card.body.addWidget(Field("Velocidad (baudios)", self.baud_combo))

        self.connect_btn = button("Conectar", "outline", self.toggle_connection)
        card.body.addWidget(self.connect_btn)

        self.status_box = QFrame()
        sb = QHBoxLayout(self.status_box)
        sb.setContentsMargins(14, 12, 14, 12)
        sb.setSpacing(12)
        self.dot = StatusDot("#8794A1", 10)
        sb.addWidget(self.dot, 0, Qt.AlignmentFlag.AlignVCenter)
        st = QVBoxLayout()
        st.setSpacing(1)
        self.status_title = label("Sin conexión", "Title")
        self.status_sub = label("Conecte el cable USB y pulse Conectar.", "Small", wrap=True)
        st.addWidget(self.status_title)
        st.addWidget(self.status_sub)
        sb.addLayout(st, 1)
        card.body.addWidget(self.status_box)
        cv.addWidget(card)

        self.start_btn = button("Comencemos", "primary", self.start.emit)
        self.start_btn.setIcon(icon("arrow", "white", 18))
        self.start_btn.setLayoutDirection(Qt.LayoutDirection.RightToLeft)
        self.start_btn.setMinimumHeight(54)
        self.start_btn.setFont(theme.sans(12.5, QFont.Weight.DemiBold))
        cv.addWidget(self.start_btn)
        self.sim_btn = button("Continuar sin robot (modo simulación)", "link", self._start_sim)
        cv.addWidget(self.sim_btn, 0, Qt.AlignmentFlag.AlignHCenter)

        rv.addWidget(col, 0, Qt.AlignmentFlag.AlignHCenter)
        rv.addStretch(1)
        root.addWidget(right, 2)

        controller.link.connection_changed.connect(lambda *_: self._update_status())
        controller.status_changed.connect(self._update_status)
        self.refresh_ports()
        self._update_status()

    # ------------------------------------------------------------ acciones
    def refresh_ports(self):
        self.port_combo.clear()
        ports = available_ports()
        for dev, desc in ports:
            self.port_combo.addItem("%s · %s" % (dev, desc), dev)
        if not ports:
            self.port_combo.addItem("No se encontraron puertos", None)
        self.port_combo.setEnabled(bool(ports))
        self._update_status()

    def toggle_connection(self):
        link = self.ctrl.link
        if link.connected:
            link.close()
            return
        port = self.port_combo.currentData()
        if not port:
            self._show_status(False, "Sin conexión", "No hay un puerto seleccionado. Conecte la ESP32 y "
                                                     "pulse el botón de buscar.", error=True)
            return
        ok, msg = link.open(port, self.baud_combo.currentData())
        if not ok:
            self._show_status(False, "No se pudo conectar", msg, error=True)

    def _start_sim(self):
        if self.ctrl.link.connected:
            self.ctrl.link.close()
        self.start.emit()

    def _update_status(self):
        link = self.ctrl.link
        if link.connected:
            title, _sub, _color = self.ctrl.status_text()
            if not self.ctrl.responding:
                self._show_status(True, "Puerto abierto · esperando a la ESP32...",
                                  "%s · %d baudios. Si no responde en unos segundos, pulse RESET en la placa."
                                  % (link.port_name, link.baud), warn=True)
            else:
                sub = "%s · %d baudios · modo %d" % (link.port_name, link.baud, self.ctrl.mode)
                if not self.ctrl.homed:
                    sub += " · falta fijar el cero"
                self._show_status(True, "Robot conectado · " + (self.ctrl.fw_state or "OK"), sub)
        else:
            self._show_status(False, "Sin conexión", "Conecte el cable USB y pulse Conectar.")

    def _show_status(self, on, title, sub, error=False, warn=False):
        self.status_title.setText(title)
        self.status_sub.setText(sub)
        self.connect_btn.setText("Desconectar" if on else "Conectar")
        self.start_btn.setEnabled(on)
        if on and warn:
            bg, border, dot = C["amber_tint"], "#F1D3A3", "#D9A04A"
        elif on:
            bg, border, dot = C["success_tint"], C["success_border"], C["success"]
        elif error:
            bg, border, dot = C["danger_tint"], "#F3C2C2", C["danger"]
        else:
            bg, border, dot = C["surface_alt"], C["border"], "#8794A1"
        self.status_box.setStyleSheet("QFrame { background: %s; border: 1px solid %s; border-radius: 10px; }"
                                      "QLabel { background: transparent; border: none; }" % (bg, border))
        self.dot.set_color(dot)

    def showEvent(self, ev):
        super().showEvent(ev)
        self.view.auto_rotate(True)

    def hideEvent(self, ev):
        super().hideEvent(ev)
        self.view.auto_rotate(False)
