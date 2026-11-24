"""Control y estimación: corrección de la trayectoria con IMU (firmware 1.3).

El firmware 1.3 corre un EKF en la ESP32 (100 Hz, en la misma tarea que lee la BNO055) que estima
los ángulos, velocidades y pares de perturbación de los brazos. Una ley de corrección calcula Δq
y la suma a los pulsos que van a los drivers. Desde aquí se elige el modo:

  * Apagada: el firmware se comporta igual que el 1.x (corridas "sin IMU").
  * Sombra:  el EKF y la ley corren y se registran en cada corrida, pero no se aplican.
  * Activa:  Δq se suma a los pulsos (corridas "con IMU").

y los parámetros de la ley. Cada corrida guarda en info.json la configuración que estaba vigente.

Firmware 1.4: prealimentación por modelo (se enciende y apaga al elegirla, igual que el modo) y
el campo "Carga en el efector", que solo se anota en cada corrida y en el nombre de la carpeta.
"""
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QVBoxLayout, QWidget

from .. import theme
from ..config import C
from ..widgets import Card, Field, MessageLabel, PageHeader, Readout, Segmented, button, label, spin

BLOCKS = [
    ("Trayectoria", "IK + perfil · q_plan", "plain"),
    ("Σ  q_plan + ff + Δq", "pulsos cada 4 ms", "accent"),
    ("Drivers iHSV57", "lazos internos P + PI", "plain"),
    ("Mecanismo 5 barras", "θ₁, θ₂ → P(x, y)", "plain"),
    ("IMU BNO055", "ω_z, a · distal der.", "plain"),
    ("EKF + ley", "q̂, ω̂, d̂ → Δq", "amber"),
]

# valores recomendados (validados en simulación y con las corridas del 13 nov)
DEFAULTS = {"ley": "ALFA", "alfa": 0.30, "zona": 1.0, "dmax": 1.0, "lavado": 1.0, "ffgan": 1.0}

MODOS = {0: "Apagada", 1: "Sombra", 2: "Activa"}
AYUDA_MODO = {
    "0": "Apagada: el firmware no corre el EKF y se comporta igual que el 1.x. Úsela para las corridas "
         "\"sin IMU\".",
    "1": "Sombra: el EKF y la ley corren y se registran en cada corrida (columnas *_ekf y dq*_ley en imu.csv), "
         "pero la corrección NO se aplica. Sirve para verificar el EKF antes de cerrar el lazo.",
    "2": "Activa: la corrección Δq se suma a los pulsos. Si la IMU falla o el EKF deja de coincidir con el "
         "giroscopio, el firmware vuelve solo a Sombra y lo avisa.",
}


class LoopDiagram(QWidget):
    """Diagrama de bloques del lazo con el retorno del EKF a la suma de la referencia."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(150)

    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        n = len(BLOCKS)
        gap = 26
        w = (self.width() - 20 - gap * (n - 1)) / n
        h = 64
        y = 16
        rects = []
        for i, (title, sub, kind) in enumerate(BLOCKS):
            r = QRectF(10 + i * (w + gap), y, w, h)
            rects.append(r)
            if kind == "accent":
                border, fill = C["accent"], C["accent_tint"]
            elif kind == "amber":
                border, fill = "#D9A04A", C["amber_tint"]
            else:
                border, fill = C["border_strong"], C["surface_alt"]
            qp.setPen(QPen(QColor(border), 1.4))
            qp.setBrush(QColor(fill))
            qp.drawRoundedRect(r, 9, 9)
            qp.setPen(QColor(C["text"]))
            qp.setFont(theme.sans(9.5, QFont.Weight.DemiBold))
            qp.drawText(r.adjusted(6, 8, -6, -h / 2), Qt.AlignmentFlag.AlignCenter, title)
            qp.setPen(QColor(C["muted"]))
            qp.setFont(theme.sans(8.5))
            qp.drawText(r.adjusted(6, h / 2, -6, -8), Qt.AlignmentFlag.AlignCenter, sub)
        pen = QPen(QColor("#7D8995"), 1.6)
        qp.setPen(pen)
        qp.setBrush(QColor("#7D8995"))
        for a, b in zip(rects[:-1], rects[1:]):
            self._arrow(qp, QPointF(a.right() + 2, a.center().y()), QPointF(b.left() - 2, b.center().y()))
        last, suma = rects[-1], rects[1]
        yb = y + h + 34
        qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawLine(QPointF(last.center().x(), last.bottom()), QPointF(last.center().x(), yb))
        qp.drawLine(QPointF(last.center().x(), yb), QPointF(suma.center().x(), yb))
        qp.setBrush(QColor("#7D8995"))
        self._arrow(qp, QPointF(suma.center().x(), yb), QPointF(suma.center().x(), suma.bottom() + 2))
        qp.setPen(QColor(C["amber_text"]))
        qp.setFont(theme.sans(8.5))
        qp.drawText(QRectF(suma.center().x(), yb + 2, last.center().x() - suma.center().x(), 18),
                    Qt.AlignmentFlag.AlignCenter,
                    "Δq = −K⁻¹·D·zona(ω̂ − q̇_plan), con lavado y recorte · 100 Hz")
        qp.end()

    @staticmethod
    def _arrow(qp, p1, p2):
        qp.drawLine(p1, p2)
        d = p2 - p1
        length = max(1e-6, (d.x() ** 2 + d.y() ** 2) ** 0.5)
        ux, uy = d.x() / length, d.y() / length
        tip = p2
        left = QPointF(tip.x() - 8 * ux + 4 * uy, tip.y() - 8 * uy - 4 * ux)
        right = QPointF(tip.x() - 8 * ux - 4 * uy, tip.y() - 8 * uy + 4 * ux)
        qp.drawPolygon(QPolygonF([tip, left, right]))


class ControlScreen(QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setObjectName("Page")
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 22, 28, 18)
        root.setSpacing(16)
        header = PageHeader("Control y estimación",
                            "Corrección de la trayectoria con IMU: EKF en la ESP32 y realimentación del estado "
                            "estimado, y prealimentación por modelo (firmware 1.3 / 1.4)")
        header.add(button("Valores recomendados", None, self.restore))
        header.add(button("Aplicar", "primary", self.apply))
        root.addWidget(header)

        loop = Card(spacing=8)
        loop.body.addWidget(label("Lazo de control", "Title"))
        loop.body.addWidget(LoopDiagram())
        root.addWidget(loop)

        grid = QGridLayout()
        grid.setSpacing(16)

        # ---------------------------------------------------------------- configuración
        c1 = Card(spacing=12)
        c1.body.addWidget(label("Corrección con IMU", "Title"))
        self.seg = Segmented([("0", "Apagada"), ("1", "Sombra"), ("2", "Activa")], "0")
        self.seg.changed.connect(self._mode_clicked)
        c1.body.addWidget(Field("Modo (se envía al robot al elegirlo)", self.seg))
        self.mode_help = label(AYUDA_MODO["0"], "Muted", wrap=True)
        c1.body.addWidget(self.mode_help)
        self.ley = QComboBox()
        self.ley.addItem("Inyección de amortiguamiento α (recomendada)", "ALFA")
        self.ley.addItem("LQR con retardo (experimental)", "LQR")
        c1.body.addWidget(Field("Ley de corrección", self.ley))
        g = QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(10)
        self.alfa = spin(DEFAULTS["alfa"], 0.0, 1.0, "", 2, 0.05)
        self.alfa.setToolTip("Amortiguamiento agregado, como fracción del crítico de cada articulación.")
        self.zona = spin(DEFAULTS["zona"], 0.0, 20.0, " °/s", 1, 0.5)
        self.zona.setToolTip("Errores de velocidad menores que esto no se corrigen (ruido del EKF).")
        self.dmax = spin(DEFAULTS["dmax"], 0.0, 3.0, " °", 2, 0.25)
        self.dmax.setToolTip("Recorte de la corrección en cada motor.")
        self.lavado = spin(DEFAULTS["lavado"], 0.0, 10.0, " s", 1, 0.5)
        self.lavado.setToolTip("Filtro de lavado: la corrección no puede sostener un valor constante, así un "
                               "sesgo del acelerómetro no deja error de posición en reposo. 0 = sin filtro.")
        g.addWidget(Field("α · amortiguamiento", self.alfa), 0, 0)
        g.addWidget(Field("Zona muerta", self.zona), 0, 1)
        g.addWidget(Field("Δq máximo", self.dmax), 1, 0)
        g.addWidget(Field("Lavado", self.lavado), 1, 1)
        c1.body.addLayout(g)

        # prealimentación y carga (firmware 1.4)
        c1.body.addWidget(label("Prealimentación por modelo (firmware 1.4)", "Title"))
        self.seg_ff = Segmented([("0", "Apagada"), ("1", "Encendida")], "0")
        self.seg_ff.changed.connect(self._ff_clicked)
        self.ffgan = spin(DEFAULTS["ffgan"], 0.0, 1.5, "", 2, 0.25)
        self.ffgan.setToolTip("Escala de la prealimentación: 1 = el modelo identificado. Se envía con Aplicar.")
        self.carga = spin(0, 0, 1000, " g", 0, 50)
        self.carga.setToolTip("Masa agregada en el efector. El firmware no la usa: se anota en cada corrida "
                              "(info.json y nombre de la carpeta).")
        self.carga.valueChanged.connect(self._carga_changed)
        g4 = QGridLayout()
        g4.setHorizontalSpacing(12)
        g4.setVerticalSpacing(10)
        g4.addWidget(Field("Prealimentación (se envía al elegirla)", self.seg_ff), 0, 0, 1, 2)
        g4.addWidget(Field("Ganancia", self.ffgan), 1, 0)
        g4.addWidget(Field("Carga en el efector", self.carga), 1, 1)
        c1.body.addLayout(g4)
        c1.body.addStretch(1)

        # ---------------------------------------------------------------- estado en el robot
        c2 = Card(spacing=10)
        c2.body.addWidget(label("Estado en el robot", "Title"))
        g2 = QGridLayout()
        g2.setHorizontalSpacing(14)
        g2.setVerticalSpacing(10)
        self.r_modo = Readout("Modo en el firmware")
        self.r_ekf = Readout("EKF")
        self.r_apl = Readout("Corrección")
        self.r_t = Readout("Cálculo EKF + ley")
        self.r_t.setToolTip("Tiempo que tarda el EKF + la ley en la tarea de la IMU (último / máximo). "
                            "Debe quedar bien por debajo de los 10 ms del periodo.")
        self.r_dq = Readout("q̂ − q emitido [°]")
        self.r_w = Readout("ω̂₁, ω̂₂ [°/s]")
        self.r_corr = Readout("Δq de la ley [°]")
        self.r_inov = Readout("Innovación gz [°/s]")
        self.r_ff = Readout("Prealimentación sumada [°]")
        self.r_da = Readout("Δq sumada [°]")
        for i, r in enumerate((self.r_modo, self.r_ekf, self.r_apl, self.r_t,
                               self.r_dq, self.r_w, self.r_corr, self.r_inov, self.r_ff, self.r_da)):
            g2.addWidget(r, i // 2, i % 2)
        c2.body.addLayout(g2)
        self.fw_note = label("", "Muted", wrap=True)
        c2.body.addWidget(self.fw_note)
        c2.body.addStretch(1)

        # ---------------------------------------------------------------- modelo
        c3 = Card(spacing=8)
        c3.body.addWidget(label("Modelo del EKF (fijo en el firmware)", "Title"))
        filas = [
            ("Estados", "q₁, q₂, ω₁, ω₂, d₁, d₂, sesgo del giróscopo"),
            ("Mediciones", "gz, ax, ay de la BNO055 (distal derecho)"),
            ("Servo + driver", "K = 1.335 / 0.865 N·m/rad · B = 0.036 / 0.066 N·m·s/rad"),
            ("", "ff = 0.609 · τ = 23.4 ms (identificado el 13 nov)"),
            ("Prealimentación", "q_c = q_plan(t+τ) + K⁻¹·[M·q̈ + C·q̇ + B·(1−ff)·q̇]"),
            ("Perturbación", "Gauss-Markov, τ_d = 0.3 s, σ_d = 0.03 N·m"),
            ("Ruido de medición", "gz 0.8 °/s · acelerómetro 0.35 m/s²"),
            ("Montaje del chip", "ψ = −177° · R = 33.5 mm · K_giro = 1.031"),
            ("Retardos", "IMU + planificador ≈ 20 ms · servo 23 ms"),
        ]
        g3 = QGridLayout()
        g3.setHorizontalSpacing(12)
        g3.setVerticalSpacing(6)
        self._modelo_labels = []
        for i, (k, v) in enumerate(filas):
            g3.addWidget(label(k, "Small"), i, 0, Qt.AlignmentFlag.AlignTop)
            lv = label(v, None, wrap=True)
            lv.setStyleSheet("font-family: '%s'; font-size: 9pt;" % theme.mono_family())
            g3.addWidget(lv, i, 1)
            self._modelo_labels.append(lv)
        g3.setColumnStretch(1, 1)
        c3.body.addLayout(g3)
        c3.body.addStretch(1)

        for i, c in enumerate((c1, c2, c3)):
            grid.addWidget(c, 0, i)
            grid.setColumnStretch(i, 1)
        root.addLayout(grid, 1)

        bottom = QHBoxLayout()
        self.msg = MessageLabel("Conecte el robot con el firmware 1.3 para usar la corrección con IMU.")
        bottom.addWidget(self.msg, 1)
        root.addLayout(bottom)

        controller.ctl_changed.connect(self._refresh_state)
        controller.status_changed.connect(self._refresh_state)
        controller.imu_changed.connect(self._refresh_state)
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._refresh_live)
        self._timer.start()
        self._refresh_state()

    # ------------------------------------------------------------ valores
    def values(self):
        v = {"ley": self.ley.currentData(), "alfa": self.alfa.value(), "zona": self.zona.value(),
             "dmax": self.dmax.value(), "lavado": self.lavado.value()}
        if self.ctrl.ff_available:
            v["ffgan"] = self.ffgan.value()
        return v

    def restore(self):
        i = self.ley.findData(DEFAULTS["ley"])
        self.ley.setCurrentIndex(max(0, i))
        self.alfa.setValue(DEFAULTS["alfa"])
        self.zona.setValue(DEFAULTS["zona"])
        self.dmax.setValue(DEFAULTS["dmax"])
        self.lavado.setValue(DEFAULTS["lavado"])
        self.ffgan.setValue(DEFAULTS["ffgan"])
        self.msg.show_message("Valores recomendados cargados. Pulse Aplicar para enviarlos al robot.", "")

    def _send(self, modo):
        v = self.values()
        ok = self.ctrl.set_control(modo=modo, **v)
        if not ok:
            self._refresh_state()
            return False
        txt = "%s · %s" % (MODOS.get(modo, "?") if modo is not None else "Parámetros",
                           "α = %.2f, zona %.1f °/s, Δq máx %.2f°, lavado %.1f s"
                           % (v["alfa"], v["zona"], v["dmax"], v["lavado"]) if v["ley"] == "ALFA"
                           else "LQR (experimental), Δq máx %.2f°, lavado %.1f s" % (v["dmax"], v["lavado"]))
        warn = v["ley"] == "LQR" and modo == 2
        self.msg.show_message("Enviado al robot: %s.%s" % (
            txt, " Aviso: el LQR realimenta la posición estimada y, con un sesgo del acelerómetro, puede dejar "
                 "1–2 mm de error en reposo; úselo solo para comparar." if warn else ""), "warn" if warn else "ok")
        return True

    def apply(self):
        cur = self.ctrl.ctl.get("modo") if isinstance(self.ctrl.ctl, dict) else None
        self._send(cur)

    def _ff_clicked(self, key):
        on = key == "1"
        if not self.ctrl.set_control(ff=on, ffgan=self.ffgan.value()):
            st = self.ctrl.ctl if isinstance(self.ctrl.ctl, dict) else {}
            self.seg_ff.set_current("1" if st.get("ff") else "0")
            return
        self.msg.show_message("Prealimentación %s (ganancia %.2f)." % ("encendida" if on else "apagada",
                                                                      self.ffgan.value()), "ok")

    def _carga_changed(self, v):
        self.ctrl.carga_g = float(v)
        self.ctrl.status_changed.emit()

    def _mode_clicked(self, key):
        self.mode_help.setText(AYUDA_MODO.get(key, ""))
        if not self._send(int(key)):
            cur = self.ctrl.ctl.get("modo") if isinstance(self.ctrl.ctl, dict) else 0
            self.seg.set_current(str(cur))
            self.mode_help.setText(AYUDA_MODO.get(str(cur), ""))

    # ------------------------------------------------------------ estado
    def _refresh_state(self):
        c = self.ctrl
        st = c.ctl if isinstance(c.ctl, dict) else None
        disponible = c.real and st is not None
        for w in (self.seg, self.ley, self.alfa, self.zona, self.dmax, self.lavado):
            w.setEnabled(disponible)
        ff_ok = c.ff_available
        for w in (self.seg_ff, self.ffgan):
            w.setEnabled(ff_ok)
        if isinstance(c.ctl_modelo, dict):
            m = c.ctl_modelo
            try:
                self._modelo_labels[2].setText("K = %.3f / %.3f N·m/rad · B = %.3f / %.3f N·m·s/rad"
                                               % (m["K"][0], m["K"][1], m["B"][0], m["B"][1]))
                self._modelo_labels[3].setText("ff = %.3f · τ = %.1f ms (vigente en el firmware)"
                                               % (m["ff"], m["tau"] * 1000))
            except (KeyError, IndexError, TypeError):
                pass
        if not c.real:
            self.fw_note.setText("Sin robot conectado (simulación): la corrección vive en el firmware.")
        elif c.ctl is False:
            self.fw_note.setText("El firmware conectado no tiene la corrección con IMU (versión 1.x). "
                                 "Cargue el firmware 1.3 para usarla.")
        elif st is None:
            self.fw_note.setText("Consultando el firmware…")
        elif not c.imu_ok:
            self.fw_note.setText("El firmware tiene la corrección, pero no llegan datos de la IMU.")
        else:
            self.fw_note.setText("Firmware con corrección con IMU%s%s." % (
                " " + c.fw_version if c.fw_version else "",
                " y prealimentación" if ff_ok else " (sin prealimentación: cargue el 1.4 para usarla)"))
        if st is None:
            for r in (self.r_modo, self.r_ekf, self.r_apl, self.r_t):
                r.set("—")
            return
        modo = st.get("modo", 0)
        self.seg.set_current(str(modo))
        if ff_ok:
            self.seg_ff.set_current("1" if st.get("ff") else "0")
        self.mode_help.setText(AYUDA_MODO.get(str(modo), ""))
        ley = "α %.2f" % st.get("alfa", 0) if st.get("ley") == "ALFA" else "LQR"
        self.r_modo.set("%s · %s" % (MODOS.get(modo, "?"), ley))
        if "t_ekf_us" in st:
            self.r_t.set("%d / %d µs" % (st["t_ekf_us"], st.get("t_ekf_max_us", 0)))

    def _refresh_live(self):
        if not self.isVisible():
            return
        c = self.ctrl
        s = c.imu_last
        if not c.real or s is None or len(s) < 24:
            for r in (self.r_dq, self.r_w, self.r_corr, self.r_inov, self.r_ff, self.r_da):
                r.set("—")
            if c.real and s is not None and isinstance(c.ctl, dict):
                self.r_ekf.set("—")
            return
        q1, q2 = s[2], s[3]
        q1h, q2h, w1h, w2h, _d1, _d2, dl1, dl2, inov, est = s[14:24]
        if len(s) >= 30:
            self.r_ff.set("%+.2f  %+.2f" % (s[26], s[27]))
            self.r_da.set("%+.3f  %+.3f" % (s[28], s[29]))
        else:
            self.r_ff.set("—")
            self.r_da.set("—")
        modo, valido, aplicando = est & 3, (est >> 2) & 1, (est >> 3) & 1
        self.r_ekf.set("válido" if valido else ("apagado" if modo == 0 else "iniciando"))
        self.r_apl.set("aplicando" if aplicando else ("calculada, sin aplicar" if modo == 1 else "—"))
        if modo == 0:
            for r in (self.r_dq, self.r_w, self.r_corr, self.r_inov):
                r.set("—")
            return
        self.r_dq.set("%+.3f  %+.3f" % (q1h - q1, q2h - q2))
        self.r_w.set("%+.1f  %+.1f" % (w1h, w2h))
        self.r_corr.set("%+.3f  %+.3f" % (dl1, dl2))
        self.r_inov.set("%+.2f" % inov)
