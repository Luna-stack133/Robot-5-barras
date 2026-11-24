"""Trayectorias: figuras, ir a un punto y secuencias pick & place con la garra."""
from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (QAbstractItemView, QButtonGroup, QComboBox, QGridLayout, QHBoxLayout, QHeaderView,
                               QSizePolicy, QSlider, QStackedWidget, QTableWidget, QTableWidgetItem, QToolButton, QVBoxLayout,
                               QWidget)

from .. import config
from .. import kinematics as kin
from ..config import C
from ..icons import figure_pixmap
from ..paths import FIGURES, check_path, corner_indices, densify, figure_points, limit_points
from ..widgets import Card, EStopButton, Field, MessageLabel, PageHeader, Segmented, button, divider, label, spin
from ..workspace_map import GOOD_QUALITY, WorkspaceMap

ACTIONS = ["Mover", "Tomar", "Soltar"]


class TrajectoryScreen(QWidget):
    go_monitor = Signal()

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setObjectName("Page")
        self.mode = "fig"
        self._plan = None

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 22, 28, 18)
        root.setSpacing(16)
        header = PageHeader("Trayectorias",
                            "Dibuje figuras, envíe el robot a un punto o programe una secuencia con la garra")
        header.add(EStopButton(self.ctrl.emergency_stop))
        root.addWidget(header)

        tabs_card = Card(margins=(5, 5, 5, 5))
        self.tabs = Segmented([("fig", "Figuras"), ("point", "Ir a un punto"), ("seq", "Secuencia (pick & place)")],
                              "fig", variant="tab")
        self.tabs.changed.connect(self._set_mode)
        tabs_card.body.addWidget(self.tabs)
        tabs_row = QHBoxLayout()
        tabs_row.addWidget(tabs_card)
        tabs_row.addStretch(1)
        root.addLayout(tabs_row)

        body = QHBoxLayout()
        body.setSpacing(16)
        root.addLayout(body, 1)

        # ---------------- configuración
        left = Card(spacing=16)
        left.setFixedWidth(410)
        self.pages = QStackedWidget()
        self.pages.addWidget(self._figure_page())
        self.pages.addWidget(self._point_page())
        self.pages.addWidget(self._sequence_page())
        left.body.addWidget(self.pages)

        self.speed_label = label("", "Muted")
        left.body.addWidget(self.speed_label)
        self.speed = QSlider(Qt.Orientation.Horizontal)
        self.speed.setRange(20, int(config.VMAX_SLIDER_MM_S))
        self.speed.setValue(int(self.ctrl.vmax))
        self.speed.valueChanged.connect(lambda *_: self._speed_changed())
        left.body.addWidget(self.speed)
        self.acc_label = label("", "Muted")
        left.body.addWidget(self.acc_label)
        self.acc = QSlider(Qt.Orientation.Horizontal)
        self.acc.setRange(50, int(config.AMAX_SLIDER_MM_S2))
        self.acc.setSingleStep(10)
        self.acc.setPageStep(100)
        self.acc.setValue(int(self.ctrl.amax))
        self.acc.valueChanged.connect(lambda *_: self._speed_changed())
        left.body.addWidget(self.acc)
        left.body.addStretch(1)
        self.msg = MessageLabel()
        left.body.addWidget(self.msg)
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(button("Ver en el gemelo 3D", None, self.go_monitor.emit), 1)
        self.run_btn = button("Dibujar figura", "primary", self._run_or_stop)
        self.run_btn.setMinimumHeight(44)
        row.addWidget(self.run_btn, 1)
        left.body.addLayout(row)
        body.addWidget(left)

        # ---------------- mapa
        right = Card(margins=(0, 0, 0, 0), spacing=0)
        top = QHBoxLayout()
        top.setContentsMargins(18, 12, 18, 12)
        top.addWidget(label("Vista superior del área de trabajo", "Title"))
        top.addStretch(1)
        for color, text in (("#CFE7E9", "Zona recomendada"), ("#E9EDF1", "Alcanzable"),
                            ("#F6D3D3", "Zona singular")):
            sw = label("", None)
            sw.setFixedSize(14, 10)
            sw.setStyleSheet("background: %s; border: 1px solid %s; border-radius: 2px;" % (color, C["border_strong"]))
            top.addWidget(sw)
            top.addWidget(label(text, "Small"))
            top.addSpacing(8)
        right.body.addLayout(top)
        right.body.addWidget(divider())
        self.map = WorkspaceMap(controller)
        self.map.clicked.connect(self._map_clicked)
        self.map.hovered.connect(self._map_hovered)
        right.body.addWidget(self.map, 1)
        right.body.addWidget(divider())
        self.hover = label("Pase el ratón sobre el mapa para leer x, y. Clic para elegir centro, destino o puntos.",
                           "Small")
        self.hover.setContentsMargins(18, 10, 18, 10)
        right.body.addWidget(self.hover)
        body.addWidget(right, 1)

        controller.status_changed.connect(self._status_changed)
        controller.pose_changed.connect(self._pose_changed)
        controller.geometry_changed.connect(self._refresh)
        self._refresh()

    # ------------------------------------------------------------ páginas
    def _figure_page(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(14)
        v.addWidget(label("Figura", "Title"))
        grid = QGridLayout()
        grid.setSpacing(8)
        self.fig_group = QButtonGroup(self)
        self.fig_buttons = {}
        for i, (key, name) in enumerate(FIGURES):
            b = QToolButton()
            b.setObjectName("FigureButton")
            b.setText(name)
            b.setIcon(QIcon(figure_pixmap(key, C["muted"], 30)))
            b.setIconSize(QSize(30, 30))
            b.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            b.setCheckable(True)
            b.setMinimumHeight(78)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda *_: self._refresh())
            self.fig_group.addButton(b)
            self.fig_buttons[key] = b
            grid.addWidget(b, i // 3, i % 3)
        self.fig_buttons["circle"].setChecked(True)
        v.addLayout(grid)
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(12)
        self.f_cx = spin(75, -250, 400, " mm")
        self.f_cy = spin(190, 0, 380, " mm")
        self.f_size = spin(100, 5, 400, " mm")
        self.f_reps = spin(1, 1, 20, "", 0)
        for i, (text, s) in enumerate((("Centro x", self.f_cx), ("Centro y", self.f_cy), ("Tamaño", self.f_size),
                                       ("Repeticiones", self.f_reps))):
            s.valueChanged.connect(lambda *_: self._refresh())
            form.addWidget(Field(text, s), i // 2, i % 2)
        v.addLayout(form)
        return w

    def _point_page(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(14)
        v.addWidget(label("Ir a un punto", "Title"))
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        self.p_x = spin(150, -250, 400, " mm")
        self.p_y = spin(230, 0, 380, " mm")
        for i, (text, s) in enumerate((("Destino x", self.p_x), ("Destino y", self.p_y))):
            s.valueChanged.connect(lambda *_: self._refresh())
            form.addWidget(Field(text, s), 0, i)
        v.addLayout(form)
        self.p_kind = QComboBox()
        self.p_kind.addItem("Línea recta (cartesiano)", "line")
        self.p_kind.addItem("Movimiento articular", "joint")
        self.p_kind.currentIndexChanged.connect(lambda *_: self._refresh())
        v.addWidget(Field("Tipo de recorrido", self.p_kind))
        v.addWidget(label("También puede hacer clic sobre el mapa para elegir el destino.", "Small", wrap=True))
        return w

    def _sequence_page(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)
        v.addWidget(label("Secuencia de puntos", "Title"))
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["#", "x [mm]", "y [mm]", "Acción"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumHeight(220)
        self.table.setStyleSheet("QTableWidget { border: 1px solid %s; border-radius: 8px; }" % C["border"])
        self.table.itemChanged.connect(lambda *_: self._refresh())
        v.addWidget(self.table)
        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(button("+ Punto actual", None, self._add_current))
        row.addWidget(button("Quitar", None, self._remove_selected))
        row.addWidget(button("Limpiar", None, self._clear_points))
        v.addLayout(row)
        row2 = QHBoxLayout()
        row2.setSpacing(8)
        row2.addWidget(label("Pausa en cada punto", "Small"))
        self.pause = spin(0.0, 0.0, 10.0, " s", 1, 0.5)
        self.pause.valueChanged.connect(lambda *_: self._refresh())
        row2.addWidget(self.pause)
        row2.addWidget(button("Puntos de calibración", None, self._load_calibration), 1)
        row2.addWidget(button("Ciclo pick and place", None, self._load_pick_place), 1)
        v.addLayout(row2)
        # Ensayo de excitación comandada (Paper 1): ida y vuelta corta desde la postura actual
        row3 = QHBoxLayout()
        row3.setSpacing(8)
        row3.addWidget(label("Excitación · Δ", "Small"))
        self.exc_d = spin(15, 2, 60, " mm", 0, 1)
        row3.addWidget(self.exc_d)
        self.exc_dir = QComboBox()
        self.exc_dir.addItems(["+x", "-x", "+y", "-y"])
        row3.addWidget(self.exc_dir)
        row3.addWidget(label("ciclos", "Small"))
        self.exc_n = spin(5, 1, 20, "", 0, 1)
        row3.addWidget(self.exc_n)
        row3.addWidget(button("Cargar excitación aquí", None, self._load_excitacion), 1)
        v.addLayout(row3)
        v.addWidget(label("Lleve el robot con el movimiento manual y guarde cada punto, o haga clic en el mapa. "
                          "Con el robot real cada punto se envía como GOTO y la app espera a que llegue "
                          "(la garra aún no está en el firmware).", "Small", wrap=True))
        for x, y, a in ((-20, 220, "Tomar"), (-20, 150, "Mover"), (170, 150, "Mover"), (170, 220, "Soltar")):
            self._add_point(x, y, a, refresh=False)
        return w

    # ------------------------------------------------------------ secuencia
    def _add_point(self, x, y, action="Mover", refresh=True):
        self.table.blockSignals(True)
        r = self.table.rowCount()
        self.table.insertRow(r)
        n = QTableWidgetItem(str(r + 1))
        n.setFlags(Qt.ItemFlag.ItemIsEnabled)
        n.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table.setItem(r, 0, n)
        for c, val in ((1, x), (2, y)):
            it = QTableWidgetItem("%.1f" % val)
            it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(r, c, it)
        combo = QComboBox()
        combo.addItems(ACTIONS)
        combo.setCurrentText(action)
        combo.currentIndexChanged.connect(lambda *_: self._refresh())
        self.table.setCellWidget(r, 3, combo)
        self.table.blockSignals(False)
        if refresh:
            self._refresh()

    def _renumber(self):
        self.table.blockSignals(True)
        for r in range(self.table.rowCount()):
            self.table.item(r, 0).setText(str(r + 1))
        self.table.blockSignals(False)

    def _add_current(self):
        x, y = self.ctrl.effector()
        self._add_point(x, y)

    def _remove_selected(self):
        rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
        if not rows and self.table.rowCount():
            rows = [self.table.rowCount() - 1]
        for r in rows:
            self.table.removeRow(r)
        self._renumber()
        self._refresh()

    def _load_calibration(self):
        """Cuadrícula para la calibración cinemática con la IMU: puntos repartidos por el espacio de
        trabajo (el distal derecho barre ~60°), en zigzag, volviendo al primero. Pausa de 2 s, lento."""
        xs = (-30, 25, 80, 135, 180)
        ys = (150, 190, 230, 270)
        pts = []
        for j, y in enumerate(ys):
            fila = [(x, y) for x in xs if (kin.quality_xy(x, y) or 0) >= 0.40]
            pts += fila[::-1] if j % 2 else fila
        ok = [pts[0]] if pts else []
        for p in pts[1:]:
            if check_path([ok[-1], p])["ok"]:
                ok.append(p)
        if len(ok) > 1 and check_path([ok[-1], ok[0]])["ok"]:
            ok.append(ok[0])
        self.table.setRowCount(0)
        for x, y in ok:
            self._add_point(x, y, "Mover", refresh=False)
        self.pause.setValue(2.0)
        self.speed.setValue(50)
        self.acc.setValue(200)
        self._refresh()
        self.msg.show_message("Calibración: %d puntos, pausa de 2 s, 50 mm/s y 200 mm/s². Revise que la garra y el "
                              "eje Z no choquen en ningún punto antes de ejecutar." % len(ok), "ok")

    def _load_pick_place(self, ciclos=10):
        """Ciclo de pick and place en el plano para el bloque 2 del paper: tomar en (0, 180), soltar en
        (150, 250), 10 ciclos, pausa de 2 s en cada punto (tiempo para cerrar/abrir la garra y para medir
        el asentamiento), 100 mm/s y 1000 mm/s². La garra todavía no se maneja desde la ESP32: la columna
        "Acción" queda como registro."""
        tomar, soltar = (0.0, 180.0), (150.0, 250.0)
        if not (check_path([tomar, soltar])["ok"] and check_path([soltar, tomar])["ok"]):
            self.msg.show_message("Los puntos de pick and place no son alcanzables con esta geometría.", "error")
            return
        self.table.setRowCount(0)
        for _ in range(ciclos):
            self._add_point(*tomar, "Tomar", refresh=False)
            self._add_point(*soltar, "Soltar", refresh=False)
        self._pp_ciclos = ciclos
        self._exc = None
        self.pause.setValue(2.0)
        self.speed.setValue(100)
        self.acc.setValue(1000)
        self._refresh()
        self.msg.show_message("Pick and place: %d ciclos entre (0, 180) y (150, 250) mm, pausa de 2 s, "
                              "100 mm/s y 1000 mm/s². Puede cambiar los puntos, la pausa y la velocidad. "
                              "Grabe el video desde antes de ejecutar hasta 5 s después del último punto."
                              % ciclos, "ok")

    def _load_excitacion(self):
        """Ensayo de excitación comandada del Paper 1: desde la postura actual P va a P + Δ y vuelve,
        con pausa de 2 s en cada punto. Cada llegada a P es una respuesta libre con el servo sujetando.
        P se marca "Tomar" y P + Δ "Soltar" para que pickplace_analizar.py la analice igual que un
        pick and place. La velocidad y la aceleración quedan las elegidas en los controles."""
        x0, y0 = self.ctrl.effector()
        x0, y0 = round(x0, 1), round(y0, 1)
        d = float(self.exc_d.value())
        ux, uy = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}[self.exc_dir.currentText()]
        p, q = (x0, y0), (x0 + d * ux, y0 + d * uy)
        if not (check_path([p, q])["ok"] and check_path([q, p])["ok"]):
            self.msg.show_message("El punto P + Δ no es alcanzable desde esta postura.", "error")
            return
        n = int(self.exc_n.value())
        self.table.setRowCount(0)
        for _ in range(n):
            self._add_point(*q, "Soltar", refresh=False)
            self._add_point(*p, "Tomar", refresh=False)
        self._pp_ciclos = 0
        self._exc = (x0, y0, d, self.exc_dir.currentText(), n)
        self.pause.setValue(2.0)
        self._refresh()
        self.msg.show_message("Excitación: %d ciclos entre (%.0f, %.0f) y (%.0f, %.0f) mm, pausa de 2 s, a %d mm/s "
                              "y %d mm/s². Grabe el video desde antes de ejecutar." %
                              (n, p[0], p[1], q[0], q[1], self.speed.value(), self.acc.value()), "ok")

    def _clear_points(self):
        self._exc = None
        self._pp_ciclos = 0
        self.table.setRowCount(0)
        self._refresh()

    def _sequence(self):
        """Lista de (x, y, acción). Las filas con números inválidos se ignoran."""
        out = []
        for r in range(self.table.rowCount()):
            try:
                x = float(self.table.item(r, 1).text().replace(",", "."))
                y = float(self.table.item(r, 2).text().replace(",", "."))
            except (AttributeError, ValueError):
                continue
            out.append((x, y, self.table.cellWidget(r, 3).currentText()))
        return out

    # ------------------------------------------------------------ plan del movimiento
    def _figure_key(self):
        for k, b in self.fig_buttons.items():
            if b.isChecked():
                return k
        return "circle"

    def _build_plan(self):
        """Plan para RobotController.run(), con la vista previa (`preview`) y datos para el mapa."""
        cur = self.ctrl.effector()
        if self.mode == "fig":
            key = self._figure_key()
            pts = figure_points(key, self.f_cx.value(), self.f_cy.value(), self.f_size.value(),
                                int(self.f_reps.value()))
            sent = limit_points(pts)            # lo que realmente recibe el firmware (máx. 130)
            # la vista previa incluye el tramo de aproximación desde la posición actual (el firmware lo recorre)
            return {"kind": "figure", "points": sent, "preview": [cur] + sent, "check": [cur] + sent,
                    "name": "Figura: %s" % dict(FIGURES)[key], "marks": [], "target": None,
                    "corners": [sent[i] for i in corner_indices(sent)], "run_text": "Dibujar figura"}
        if self.mode == "point":
            target = (self.p_x.value(), self.p_y.value())
            if self.p_kind.currentData() == "joint":
                q = kin.ik(*target)
                path = [cur, target]
                if q is not None:
                    path = []
                    ref = cur
                    for k in range(61):
                        t = k / 60.0
                        p = kin.fk_near(self.ctrl.t1 + (q[0] - self.ctrl.t1) * t,
                                        self.ctrl.t2 + (q[1] - self.ctrl.t2) * t, ref)
                        if p:
                            path.append(p)
                            ref = p
                return {"kind": "joint", "target": q, "preview": path, "check": [target],
                        "name": "Articular a (%.0f, %.0f) mm" % target, "marks": [], "target": target,
                        "corners": [], "run_text": "Mover al punto", "valid_extra": q is not None}
            path = [cur, target]
            return {"kind": "goto", "target": target, "preview": path, "check": path,
                    "name": "Ir a (%.0f, %.0f) mm" % target, "marks": [], "target": target, "corners": [],
                    "run_text": "Mover al punto"}
        seq = self._sequence()
        pts = [(x, y) for x, y, _ in seq]
        path = [cur] + pts if pts else []
        pausa = self.pause.value() if hasattr(self, "pause") else 0.0
        nombre = ("Secuencia de %d puntos con pausa" if pausa > 0 else "Secuencia de %d puntos") % len(pts)
        pp = getattr(self, "_pp_ciclos", 0)
        if pp and len(pts) == 2 * pp and all(a == ("Tomar" if i % 2 == 0 else "Soltar")
                                             for i, (_, _, a) in enumerate(seq)):
            nombre = "Pick and place %d ciclos" % pp
        exc = getattr(self, "_exc", None)
        if exc and len(pts) == 2 * exc[4] and pts[-1] == (exc[0], exc[1]):
            nombre = "Excitacion %d ciclos (%.0f, %.0f) d%.0f %s" % (exc[4], exc[0], exc[1], exc[2],
                                                                    exc[3].replace("+", "mas").replace("-", "menos"))
        return {"kind": "sequence", "items": seq, "preview": path, "check": path, "pause_s": pausa,
                "name": nombre,
                "marks": pts, "target": None, "corners": [],
                "run_text": "Ejecutar secuencia"}

    def _speed_changed(self):
        self.ctrl.set_speed(self.speed.value(), self.acc.value(), send=False)
        self._refresh()

    def _refresh(self):
        speed = self.speed.value()
        self.speed_label.setText("Velocidad máxima del efector · %d mm/s" % speed)
        self.acc_label.setText("Aceleración máxima · %d mm/s²" % self.acc.value())
        real = self.ctrl.real
        joint_item = self.p_kind.model().item(1) if hasattr(self.p_kind.model(), "item") else None
        if joint_item is not None:
            joint_item.setEnabled(not real)
        if real and self.p_kind.currentData() == "joint":
            self.p_kind.setCurrentIndex(0)
        plan = self._build_plan()
        self._plan = plan
        path = plan["preview"]
        if self.mode == "seq" and not plan["marks"]:
            self.map.set_overlay([], [], None)
            self.msg.show_message("Agregue al menos un punto a la secuencia.", "warn")
            self._plan_ok = False
        else:
            chk = check_path(plan["check"])
            ok = chk["ok"] and plan.get("valid_extra", True)
            self.map.set_overlay(path, plan["marks"], plan["target"], ok, chk["bad_point"], plan["corners"])
            if not ok:
                bp = chk["bad_point"] or plan["target"]
                self.msg.show_message("Fuera de alcance o muy cerca de una singularidad en (%.1f, %.1f) mm. "
                                      "Ajuste el recorrido." % bp, "error")
            else:
                length = check_path(path)["length"] if path else 0.0
                text = "Recorrido válido · %.0f mm · ≈ %.1f s" % (length, length / speed)
                if plan["kind"] == "figure":
                    text += " · %d puntos, frena en %d" % (len(plan["points"]), max(1, len(plan["corners"]) - 1))
                if chk.get("warning"):
                    self.msg.show_message(text + ". Pasa cerca de una singularidad: reduzca la velocidad.", "warn")
                else:
                    self.msg.show_message(text, "ok")
            self._plan_ok = ok
        self._update_run_button()

    def _update_run_button(self):
        if self.ctrl.running:
            self.run_btn.setText("Detener")
            self.run_btn.setEnabled(True)
        else:
            self.run_btn.setText(self._plan["run_text"] if self._plan else "Ejecutar")
            self.run_btn.setEnabled(bool(getattr(self, "_plan_ok", False)))

    def _run_or_stop(self):
        if self.ctrl.running:
            self.ctrl.stop_run()
            return
        self._refresh()
        if not self._plan_ok:
            return
        self.ctrl.set_speed(self.speed.value(), self.acc.value(), send=False)
        self.ctrl.run(self._plan)

    def _status_changed(self):
        if self.isVisible() and not self.ctrl.running:
            self._refresh()
        else:
            self._update_run_button()

    def _pose_changed(self):
        # el tramo desde la posición actual cambia cuando el robot se mueve
        if not self.ctrl.running and self.mode in ("point", "seq") and self.isVisible():
            self._refresh()

    # ------------------------------------------------------------ eventos
    def _set_mode(self, mode):
        self.mode = mode
        self.pages.setCurrentIndex({"fig": 0, "point": 1, "seq": 2}[mode])
        self._refresh()

    def _map_clicked(self, x, y):
        if self.ctrl.running:
            return
        if self.mode == "fig":
            self.f_cx.setValue(round(x, 1))
            self.f_cy.setValue(round(y, 1))
        elif self.mode == "point":
            self.p_x.setValue(round(x, 1))
            self.p_y.setValue(round(y, 1))
        else:
            self._add_point(round(x, 1), round(y, 1))

    def _map_hovered(self, x, y):
        q = kin.quality_xy(x, y)
        if q is None:
            zone = "fuera de alcance"
        elif q >= GOOD_QUALITY:
            zone = "zona recomendada"
        else:
            zone = "alcanzable, cerca de una singularidad"
        self.hover.setText("Cursor: x = %.1f mm · y = %.1f mm · %s" % (x, y, zone))

    def showEvent(self, ev):
        super().showEvent(ev)
        self._refresh()
