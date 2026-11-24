"""Monitoreo en vivo: movimiento manual (JOG), cero por utillaje, modo de ensamblaje, IMU y gemelo 3D."""
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QButtonGroup, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPlainTextEdit,
                               QPushButton, QScrollArea, QStackedWidget, QVBoxLayout, QWidget)

from .. import kinematics as kin
from ..config import C, IMU_GRABAR_S, MODE_NAMES
from ..robot3d import Robot3DView
from ..widgets import (Card, EStopButton, Field, MessageLabel, PageHeader, Readout, Segmented, StatusDot,
                       ToggleSwitch, button, divider, label, spin)

JOINT_STEPS = [0.1, 1.0, 2.0, 5.0]      # grados
CART_STEPS = [0.5, 1.0, 5.0, 10.0]      # mm


class MonitorScreen(QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setObjectName("Page")
        self.mode = "joint"
        self.step_index = 1

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 22, 28, 18)
        root.setSpacing(16)

        # ---------------- encabezado
        header = PageHeader("Monitoreo en vivo", "Mueva el robot a mano, fije el cero y vea su postura en tiempo real")
        chip = QFrame()
        chip.setObjectName("Chip")
        cl = QHBoxLayout(chip)
        cl.setContentsMargins(14, 8, 14, 8)
        cl.setSpacing(8)
        self.state_dot = StatusDot("#8794A1", 10)
        cl.addWidget(self.state_dot)
        self.state_label = label("Simulación", "Title")
        cl.addWidget(self.state_label)
        chip.setStyleSheet("QFrame#Chip { background: white; border: 1px solid %s; border-radius: 10px; }"
                           % C["border"])
        header.add(chip)
        self.enable_btn = QPushButton()
        self.enable_btn.setProperty("variant", "enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.enable_btn.setToolTip("Bloqueo de la app: si está bloqueado, la app no envía movimientos.")
        self.enable_btn.clicked.connect(lambda on: self.ctrl.set_enabled(on))
        header.add(self.enable_btn)
        header.add(EStopButton(self.ctrl.emergency_stop))
        root.addWidget(header)

        body = QHBoxLayout()
        body.setSpacing(16)
        root.addLayout(body, 1)

        # ---------------- gemelo 3D
        vcard = Card(margins=(0, 0, 0, 0), spacing=0)
        top = QHBoxLayout()
        top.setContentsMargins(18, 12, 14, 12)
        titles = QVBoxLayout()
        titles.setSpacing(1)
        titles.addWidget(label("Gemelo 3D", "Title"))
        self.geom_label = label("", "Small")
        titles.addWidget(self.geom_label)
        top.addLayout(titles)
        top.addStretch(1)
        self.console_btn = button("Consola", None, self._toggle_console)
        self.console_btn.setCheckable(True)
        top.addWidget(self.console_btn)
        top.addWidget(button("Limpiar rastro", None, self.ctrl.clear_trail))
        views = Segmented([("3d", "3D"), ("top", "Superior"), ("front", "Frontal")], "3d")
        top.addWidget(views)
        vcard.body.addLayout(top)
        vcard.body.addWidget(divider())
        self.view = Robot3DView(controller)
        views.changed.connect(self.view.set_view)
        vcard.body.addWidget(self.view, 1)
        vcard.body.addWidget(divider())

        readouts = QHBoxLayout()
        readouts.setContentsMargins(18, 10, 18, 12)
        readouts.setSpacing(10)
        self.r_t1 = Readout("θ₁ · motor 1")
        self.r_t2 = Readout("θ₂ · motor 2")
        self.r_x = Readout("x efector")
        self.r_y = Readout("y efector")
        for r in (self.r_t1, self.r_t2, self.r_x, self.r_y):
            readouts.addWidget(r, 1)
        vcard.body.addLayout(readouts)

        self.console = QPlainTextEdit()
        self.console.setObjectName("Console")
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(800)
        self.console.setFixedHeight(150)
        self.console.setPlaceholderText("Mensajes enviados (→) y recibidos (←) por el puerto serial. "
                                        "La telemetría STATE no se muestra.")
        self.console.hide()
        cwrap = QVBoxLayout()
        cwrap.setContentsMargins(14, 0, 14, 14)
        cwrap.addWidget(self.console)
        vcard.body.addLayout(cwrap)
        controller.link.log.connect(self.console.appendPlainText)
        body.addWidget(vcard, 1)

        # ---------------- panel derecho
        side = QWidget()
        sv = QVBoxLayout(side)
        sv.setContentsMargins(0, 0, 4, 0)
        sv.setSpacing(14)
        sv.addWidget(self._jog_card())
        sv.addWidget(self._zero_card())
        sv.addWidget(self._mode_card())
        sv.addWidget(self._imu_card())
        sv.addWidget(self._vacuum_card())
        sv.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(side)
        scroll.setFixedWidth(400)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        body.addWidget(scroll)

        # ---------------- señales y atajos
        controller.pose_changed.connect(self._update_pose)
        controller.status_changed.connect(self._update_status)
        controller.geometry_changed.connect(self._update_status)
        controller.imu_changed.connect(self._update_imu)
        self._imu_timer = QTimer(self)                 # lectura de la IMU en pantalla a 5 Hz
        self._imu_timer.setInterval(200)
        self._imu_timer.timeout.connect(self._update_imu)
        self._imu_timer.start()
        controller.message.connect(lambda text, err: self.msg.show_message(text, "error" if err else ""))
        for key, dx, dy in (("Left", -1, 0), ("Right", 1, 0), ("Up", 0, 1), ("Down", 0, -1)):
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            sc.activated.connect(lambda dx=dx, dy=dy: self._jog_xy(dx, dy))
        self._update_pose()
        self._update_status()

    # ------------------------------------------------------------ tarjetas
    def _jog_card(self):
        card = Card(spacing=14)
        top = QHBoxLayout()
        top.addWidget(label("Movimiento manual", "Title"))
        top.addStretch(1)
        mode = Segmented([("joint", "Articular"), ("cart", "Cartesiano")], "joint")
        mode.changed.connect(self._set_mode)
        top.addWidget(mode)
        card.body.addLayout(top)

        self.jog_pages = QStackedWidget()
        jp = QWidget()
        jl = QGridLayout(jp)
        jl.setContentsMargins(0, 0, 0, 0)
        jl.setHorizontalSpacing(10)
        jl.setVerticalSpacing(10)
        self.joint_vals = []
        for row, name in enumerate(("Motor 1", "Motor 2")):
            jl.addWidget(label(name, "Muted"), row, 0)
            minus = button("−", "jog", lambda _=False, r=row: self._jog_joint(r, -1))
            plus = button("+", "jog", lambda _=False, r=row: self._jog_joint(r, +1))
            for b in (minus, plus):
                b.setAutoRepeat(True)
                b.setAutoRepeatDelay(400)
                b.setAutoRepeatInterval(150)
            val = label("—", "Readout")
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.joint_vals.append(val)
            jl.addWidget(minus, row, 1)
            jl.addWidget(val, row, 2)
            jl.addWidget(plus, row, 3)
        jl.setColumnStretch(2, 1)
        self.jog_pages.addWidget(jp)

        cp = QWidget()
        cl = QGridLayout(cp)
        cl.setContentsMargins(40, 0, 40, 0)
        cl.setSpacing(8)
        for text, r, c, dx, dy in (("+Y", 0, 1, 0, 1), ("−X", 1, 0, -1, 0), ("+X", 1, 2, 1, 0), ("−Y", 2, 1, 0, -1)):
            b = button(text, "jogxy", lambda _=False, dx=dx, dy=dy: self._jog_xy(dx, dy))
            b.setAutoRepeat(True)
            b.setAutoRepeatDelay(400)
            b.setAutoRepeatInterval(150)
            cl.addWidget(b, r, c)
        center = QLabel("●")
        center.setAlignment(Qt.AlignmentFlag.AlignCenter)
        center.setStyleSheet("color: %s; font-size: 14pt;" % C["accent"])
        cl.addWidget(center, 1, 1)
        self.jog_pages.addWidget(cp)
        card.body.addWidget(self.jog_pages)

        card.body.addWidget(label("Paso por clic", "Muted"))
        steps = QHBoxLayout()
        steps.setSpacing(6)
        self.step_group = QButtonGroup(self)
        self.step_buttons = []
        for i in range(4):
            b = QPushButton()
            b.setProperty("variant", "chip")
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _=False, i=i: self._set_step(i))
            self.step_group.addButton(b)
            self.step_buttons.append(b)
            steps.addWidget(b)
        card.body.addLayout(steps)
        self.jog_hint = label("", "Small", wrap=True)
        card.body.addWidget(self.jog_hint)
        self.msg = MessageLabel("Listo.")
        card.body.addWidget(self.msg)
        self._set_step(1)
        return card

    def _zero_card(self):
        card = Card(spacing=12)
        card.body.addWidget(label("Cero del robot (utillaje)", "Title"))
        row = QHBoxLayout()
        row.setSpacing(10)
        self.z1 = spin(self.ctrl.zero[0], -180, 360, "°", 2)
        self.z2 = spin(self.ctrl.zero[1], -180, 360, "°", 2)
        row.addWidget(Field("θ₁ del utillaje", self.z1))
        row.addWidget(Field("θ₂ del utillaje", self.z2))
        card.body.addLayout(row)
        row2 = QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(button("Fijar cero (utillaje puesto)", "primary", self._set_zero), 2)
        row2.addWidget(button("Ir al cero", None, self.ctrl.go_zero), 1)
        card.body.addLayout(row2)
        card.body.addWidget(label("1) Acerque los brazos con JOG articular · 2) encaje el utillaje · "
                                  "3) Fijar cero · 4) retire el utillaje.", "Small", wrap=True))
        return card

    def _mode_card(self):
        card = Card(spacing=10)
        card.body.addWidget(label("Modo de ensamblaje (codos)", "Title"))
        self.mode_combo = QComboBox()
        for m, name in MODE_NAMES.items():
            self.mode_combo.addItem(name, m)
        self.mode_combo.activated.connect(lambda *_: self._mode_selected())
        card.body.addWidget(self.mode_combo)
        card.body.addWidget(label("Con el cero fijado, el robot mueve los brazos al nuevo modo manteniendo el "
                                  "efector en su punto (el firmware verifica que sea seguro).", "Small", wrap=True))
        return card

    def _imu_card(self):
        card = Card(spacing=10)
        card.body.addWidget(label("IMU · grabar (prueba de golpes)", "Title"))
        self.imu_live = label("IMU: sin datos", "Small", wrap=True)
        self.imu_live.setStyleSheet("font-family: Consolas, 'IBM Plex Mono', monospace;")
        card.body.addWidget(self.imu_live)
        row = QHBoxLayout()
        row.setSpacing(10)
        self.rec_combo = QComboBox()
        for s in IMU_GRABAR_S:
            self.rec_combo.addItem("%d s" % s, s)
        self.rec_combo.setCurrentIndex(min(1, len(IMU_GRABAR_S) - 1))
        row.addWidget(self.rec_combo, 1)
        self.rec_btn = button("Grabar IMU", "primary", self._record)
        row.addWidget(self.rec_btn, 2)
        card.body.addLayout(row)
        card.body.addWidget(label("Con el robot quieto y los servos energizados: pulse Grabar IMU y dé golpecitos "
                                  "suaves al efector, en el plano del robot, separados unos 2 s. Deje el brazo "
                                  "quieto 3 s antes del primer golpe. La grabación se guarda sola como una corrida.",
                                  "Small", wrap=True))
        return card

    def _vacuum_card(self):
        card = Card(margins=(18, 12, 18, 12))
        row = QHBoxLayout()
        txt = QVBoxLayout()
        txt.setSpacing(1)
        txt.addWidget(label("Ventosa", "Title"))
        self.vac_label = label("Liberada", "Small", wrap=True)
        txt.addWidget(self.vac_label)
        row.addLayout(txt, 1)
        self.vac_switch = ToggleSwitch()
        self.vac_switch.toggled.connect(self.ctrl.set_vacuum)
        row.addWidget(self.vac_switch)
        card.body.addLayout(row)
        return card

    # ------------------------------------------------------------ acciones
    def _set_mode(self, mode):
        self.mode = mode
        self.jog_pages.setCurrentIndex(0 if mode == "joint" else 1)
        self._set_step(self.step_index)
        self._update_status()

    def _steps(self):
        return JOINT_STEPS if self.mode == "joint" else CART_STEPS

    def _set_step(self, i):
        self.step_index = i
        unit = "°" if self.mode == "joint" else " mm"
        for k, b in enumerate(self.step_buttons):
            b.setText(("%g" % self._steps()[k]) + unit)
        self.step_buttons[i].setChecked(True)

    def _jog_joint(self, motor, sign):
        d = sign * JOINT_STEPS[self.step_index]
        self.ctrl.jog_joint(d if motor == 0 else 0.0, d if motor == 1 else 0.0)

    def _jog_xy(self, dx, dy):
        st = CART_STEPS[self.step_index]
        self.ctrl.jog_xy(dx * st, dy * st)

    def _set_zero(self):
        self.ctrl.set_zero(self.z1.value(), self.z2.value())

    def _mode_selected(self):
        m = self.mode_combo.currentData()
        self.mode_combo.setCurrentIndex(self.mode_combo.findData(kin.mode()))   # vuelve hasta que se confirme
        self.ctrl.request_mode(m)

    def _record(self):
        self.ctrl.record_imu(self.rec_combo.currentData())
        self._update_imu()

    def _update_imu(self):
        c = self.ctrl
        rec = c.recording
        self.rec_btn.setEnabled(c.real and bool(c.imu_ok) and (rec or not c.running))
        self.rec_btn.setText("Grabando… %.0f s" % c.recording_left() if rec else "Grabar IMU")
        if rec:
            self.rec_btn.setEnabled(False)
        self.rec_combo.setEnabled(not rec)
        if not c.real:
            self.imu_live.setText("IMU: solo con el robot conectado.")
            return
        if c.imu_ok is False:
            self.imu_live.setText("IMU: el firmware no tiene IMU disponible.")
            return
        s = c.imu_last
        if not c.imu_ok or s is None:
            self.imu_live.setText("IMU: esperando datos…")
            return
        # s = (t_host, t_us, q1, q2, ax, ay, az, gx, gy, gz, rumbo, roll, pitch, cal)
        cal = s[13]
        amag = (s[4] ** 2 + s[5] ** 2 + s[6] ** 2) ** 0.5
        self.imu_live.setText("gz %+7.2f °/s   |a| %.2f m/s²\nroll %+5.1f°  pitch %+5.1f°  rumbo %5.1f°\n"
                              "calibración gir %d · acel %d"
                              % (s[9], amag, s[11], s[12], s[10], (cal >> 4) & 3, (cal >> 2) & 3))

    def _toggle_console(self):
        self.console.setVisible(self.console_btn.isChecked())

    # ------------------------------------------------------------ actualización
    def _update_pose(self):
        c = self.ctrl
        x, y = c.effector()
        self.r_t1.set("%.2f°" % c.t1)
        self.r_t2.set("%.2f°" % c.t2)
        self.r_x.set("%.1f mm" % x)
        self.r_y.set("%.1f mm" % y)
        self.joint_vals[0].setText("%.1f°" % c.t1)
        self.joint_vals[1].setText("%.1f°" % c.t2)

    def _update_status(self):
        c = self.ctrl
        title, _sub, color = c.status_text()
        self.state_label.setText(title)
        self.state_dot.set_color(color)
        self.enable_btn.setChecked(c.enabled)
        self.enable_btn.setText("●  Movimiento habilitado" if c.enabled else "○  Movimiento bloqueado")
        g = kin.GEOM
        self.geom_label.setText("Proximales %.0f / %.0f mm · distales %.0f / %.0f mm · motores a %.0f mm · modo %d"
                                % (g["L1"], g["L4"], g["L2"], g["L3"], g["D"], kin.mode()))
        self.mode_combo.setCurrentIndex(self.mode_combo.findData(kin.mode()))
        if c.real and not c.homed:
            self.jog_hint.setText("Sin cero fijado solo funciona el movimiento articular (JOG). "
                                  "Fije el cero para usar el cartesiano.")
        elif c.real and self.mode == "cart":
            self.jog_hint.setText("Cada clic envía GOTO al punto vecino. Flechas del teclado: mover en X / Y.")
        elif c.real:
            self.jog_hint.setText("Cada clic envía JOG al motor (giro relativo, velocidad baja).")
        else:
            self.jog_hint.setText("Simulación: los movimientos se aplican al gemelo. Flechas del teclado: X / Y.")
        self.vac_switch.blockSignals(True)
        self.vac_switch.setChecked(c.vacuum and not c.real)
        self.vac_switch.blockSignals(False)
        self.vac_switch.setEnabled(not c.real)
        self.vac_switch.update()
        if c.real:
            self.vac_label.setText("Pendiente: el firmware aún no tiene comando para la bomba de vacío.")
        else:
            self.vac_label.setText("Succión activa: objeto sujeto" if c.vacuum else "Liberada")
