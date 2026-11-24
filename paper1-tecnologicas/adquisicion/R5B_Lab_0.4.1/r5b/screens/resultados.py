"""Resultados: registro de las corridas, gráficas y exportación a CSV."""
import csv
import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QHeaderView, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from .. import kinematics as kin
from .. import theme
from ..config import C
from ..registro import guardar_corrida
from ..widgets import Card, MessageLabel, PageHeader, Readout, Segmented, button, divider, label


def _muestras(run):
    """Muestras (t, x, y, θ1, θ2) de la corrida con la mejor resolución disponible.

    Con el robot real y la IMU activa, cada muestra de la IMU (100 Hz) trae los ángulos de los motores
    según los pulsos emitidos en ese instante: el recorrido se reconstruye con la cinemática directa.
    La telemetría STATE llega solo a 20–50 Hz y, dibujada como polilínea, "corta" las esquinas y parece
    que el robot no pasa por los puntos. Sin IMU (o en simulación) se usa la telemetría."""
    cache = run.get("_muestras")
    if cache is not None:
        return cache
    out = []
    imu = run.get("imu") or []
    if len(imu) >= 5 and run.get("tipo") != "grabacion":
        ref = None
        fin = run.get("duration", imu[-1][0])
        for s in imu:
            if s[0] < 0.0 or s[0] > fin:
                continue
            q1, q2 = s[1], s[2]
            p = kin.fk_near(q1, q2, ref) if ref is not None else kin.fk_mode(q1, q2)
            if p is None:
                continue
            ref = p
            out.append((s[0], p[0], p[1], q1, q2))
    if len(out) < 5:
        out = list(run.get("samples") or [])
    run["_muestras"] = out
    return out


def _muestras_ekf(run):
    """Recorrido estimado por el EKF del firmware 1.3 (t, x, y, θ̂1, θ̂2), solo donde el EKF era válido."""
    cache = run.get("_muestras_ekf")
    if cache is not None:
        return cache
    out = []
    imu = run.get("imu") or []
    if len(imu) >= 5 and len(imu[0]) >= 23 and run.get("tipo") != "grabacion":
        ref = None
        fin = run.get("duration", imu[-1][0])
        for s in imu:
            if len(s) < 23 or s[0] < 0.0 or s[0] > fin or not (s[22] >> 2) & 1:
                continue
            q1, q2 = s[13], s[14]
            p = kin.fk_near(q1, q2, ref) if ref is not None else kin.fk_mode(q1, q2)
            if p is None:
                continue
            ref = p
            out.append((s[0], p[0], p[1], q1, q2))
    run["_muestras_ekf"] = out
    return out


def _texto_corrida(run):
    """Línea de estado al seleccionar una corrida: nombre y modo de la corrección con IMU."""
    info = run.get("info") or {}
    ctl = info.get("correccion_imu")
    txt = run.get("name", "Corrida")
    if not ctl:
        return txt + (" · simulación" if not run.get("imu") else "")
    modo = ctl.get("modo")
    if modo in (1, 2):
        ley = "α %.2f" % ctl.get("alfa", 0) if ctl.get("ley") == "ALFA" else "LQR"
        txt += " · corrección con IMU: %s (%s, Δq máx %.2f°, lavado %.1f s)" % (
            ctl.get("modo_texto", "?"), ley, ctl.get("dmax", 0), ctl.get("lavado", 0))
        ev = ctl.get("eventos") or []
        if ev:
            txt += " · se desactivó sola: " + ", ".join("%s a los %.1f s" % (m, t) for t, m in ev)
    else:
        txt += " · corrección con IMU: %s" % ctl.get("modo_texto", "apagada")
    if ctl.get("prealimentacion"):
        txt += " · prealimentación (ganancia %.2f)" % (ctl.get("prealimentacion_ganancia") or 1.0)
    if info.get("carga_g"):
        txt += " · carga %g g" % info["carga_g"]
    return txt


def _deviation(run, smp=None):
    """RMS de la distancia de cada muestra al recorrido de referencia (mm), o None."""
    ref = run.get("reference") or []
    smp = _muestras(run) if smp is None else smp
    if len(ref) < 2 or not smp:
        return None
    dense = []
    for (x0, y0), (x1, y1) in zip(ref[:-1], ref[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 1.0))
        dense += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    dense.append(ref[-1])
    acc = 0.0
    for s in smp:
        acc += min((s[1] - x) ** 2 + (s[2] - y) ** 2 for x, y in dense)
    return math.sqrt(acc / len(smp))


class SimpleChart(QWidget):
    """Gráfica de líneas sencilla dibujada con QPainter."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.series = []         # (xs, ys, color, discontinua, nombre)
        self.xlabel = ""
        self.ylabel = ""
        self.equal = False
        self.empty_text = "Aún no hay corridas. Ejecute una trayectoria para ver sus resultados aquí."
        self.setMinimumHeight(300)

    def set_data(self, series, xlabel="", ylabel="", equal=False):
        self.series, self.xlabel, self.ylabel, self.equal = series, xlabel, ylabel, equal
        self.update()

    def paintEvent(self, _event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.fillRect(self.rect(), QColor("white"))
        pts = [(x, y) for xs, ys, *_ in self.series for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
        if not pts:
            qp.setPen(QColor(C["muted2"]))
            qp.setFont(theme.sans(10))
            qp.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.empty_text)
            qp.end()
            return
        x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
        y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
        if x1 - x0 < 1e-9:
            x0, x1 = x0 - 1, x1 + 1
        if y1 - y0 < 1e-9:
            y0, y1 = y0 - 1, y1 + 1
        padx, pady = (x1 - x0) * 0.05, (y1 - y0) * 0.08
        x0, x1, y0, y1 = x0 - padx, x1 + padx, y0 - pady, y1 + pady
        plot = QRectF(62, 18, self.width() - 80, self.height() - 60)
        sx, sy = plot.width() / (x1 - x0), plot.height() / (y1 - y0)
        if self.equal:
            s = min(sx, sy)
            sx = sy = s
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            x0, x1 = cx - plot.width() / s / 2, cx + plot.width() / s / 2
            y0, y1 = cy - plot.height() / s / 2, cy + plot.height() / s / 2

        def tp(x, y):
            return QPointF(plot.left() + (x - x0) * sx, plot.bottom() - (y - y0) * sy)

        # rejilla y marcas
        qp.setFont(theme.mono(8))
        for k in range(6):
            fx = x0 + (x1 - x0) * k / 5
            fy = y0 + (y1 - y0) * k / 5
            qp.setPen(QPen(QColor(C["border"]), 1))
            qp.drawLine(tp(fx, y0), tp(fx, y1))
            qp.drawLine(tp(x0, fy), tp(x1, fy))
            qp.setPen(QColor(C["muted2"]))
            qp.drawText(tp(fx, y0) + QPointF(-18, 16), "%.4g" % fx)
            qp.drawText(QRectF(4, tp(x0, fy).y() - 8, 54, 16), Qt.AlignmentFlag.AlignRight, "%.4g" % fy)
        qp.setFont(theme.sans(8.5))
        qp.drawText(QRectF(plot.left(), self.height() - 20, plot.width(), 18), Qt.AlignmentFlag.AlignCenter,
                    self.xlabel)
        qp.drawText(QRectF(plot.left() + 6, plot.top() - 2, 300, 16), Qt.AlignmentFlag.AlignLeft, self.ylabel)

        # series
        qp.setClipRect(plot)
        for xs, ys, color, dashed, _name in self.series:
            pen = QPen(QColor(color), 2.0)
            if dashed:
                pen.setStyle(Qt.PenStyle.DashLine)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            qp.setPen(pen)
            path = QPainterPath()
            first = True
            for x, y in zip(xs, ys):
                if not (math.isfinite(x) and math.isfinite(y)):
                    first = True
                    continue
                if first:
                    path.moveTo(tp(x, y))
                    first = False
                else:
                    path.lineTo(tp(x, y))
            qp.drawPath(path)
        qp.setClipping(False)

        # leyenda
        lx = plot.right() - 10
        qp.setFont(theme.sans(8.5))
        for xs, ys, color, dashed, name in reversed(self.series):
            w = qp.fontMetrics().horizontalAdvance(name)
            lx -= w + 34
            pen = QPen(QColor(color), 2.0, Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine)
            qp.setPen(pen)
            qp.drawLine(QPointF(lx, plot.top() + 10), QPointF(lx + 18, plot.top() + 10))
            qp.setPen(QColor(C["muted"]))
            qp.drawText(QPointF(lx + 24, plot.top() + 14), name)
        qp.end()


def _gz_predicho(run):
    """Velocidad de giro de los dos eslabones distales calculada con la cinemática a partir de los
    ángulos de los motores que llegan con cada muestra de la IMU. Devuelve (t, izquierdo, derecho) en °/s."""
    imu = run.get("imu") or []
    if len(imu) < 5:
        return [], [], []
    t, a_izq, a_der = [], [], []
    ref = None
    for s in imu:
        q1, q2 = s[1], s[2]
        p = kin.fk_near(q1, q2, ref) if ref is not None else kin.fk_mode(q1, q2)
        if p is None:
            continue
        ref = p
        pts = kin.points(q1, q2, p)
        (cx, cy), (dx, dy), (px, py) = pts["C"], pts["D"], pts["P"]
        t.append(s[0])
        a_izq.append(math.degrees(math.atan2(py - cy, px - cx)))
        a_der.append(math.degrees(math.atan2(py - dy, px - dx)))

    def deriv(a):
        out = [float("nan")] * len(a)
        un = list(a)
        for i in range(1, len(un)):                    # quitar saltos de 360°
            d = un[i] - un[i - 1]
            un[i] -= 360.0 * round(d / 360.0)
        for i in range(2, len(un) - 2):                # derivada central sobre ±2 muestras (suaviza)
            dt = t[i + 2] - t[i - 2]
            if dt > 1e-6:
                out[i] = (un[i + 2] - un[i - 2]) / dt
        return out
    return t, deriv(a_izq), deriv(a_der)


class ResultsScreen(QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.setObjectName("Page")
        self.runs = []
        self.chart_kind = "path"

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 22, 28, 18)
        root.setSpacing(16)
        self.header = PageHeader("Resultados", "Corridas de esta sesión: recorrido, ángulos y exportación de datos")
        self.header.add(button("Exportar CSV", "primary", self.export_csv))
        root.addWidget(self.header)

        tiles = QHBoxLayout()
        tiles.setSpacing(14)
        self.t_dur = Readout("Duración", "—", big=True)
        self.t_len = Readout("Longitud recorrida", "—", big=True)
        self.t_n = Readout("Muestras registradas", "—", big=True)
        self.t_err = Readout("Desviación de la referencia (RMS)", "—", big=True)
        self.t_err.setToolTip("Distancia de cada muestra al recorrido pedido: con los pulsos emitidos y, si la "
                              "corrida tiene el EKF del firmware 1.3, con la posición estimada por el EKF.")
        for t in (self.t_dur, self.t_len, self.t_n, self.t_err):
            c = Card(spacing=4)
            c.body.addWidget(t)
            tiles.addWidget(c, 1)
        root.addLayout(tiles)

        body = QHBoxLayout()
        body.setSpacing(16)
        chart_card = Card(margins=(0, 0, 0, 0), spacing=0)
        top = QHBoxLayout()
        top.setContentsMargins(12, 10, 12, 10)
        tabs = Segmented([("path", "Trayectoria x–y"), ("joints", "Ángulos θ₁, θ₂"), ("speed", "Velocidad"),
                          ("imu", "IMU · gz"), ("ekf", "EKF · Δq")], "path")
        tabs.changed.connect(self._set_chart)
        top.addWidget(tabs)
        top.addStretch(1)
        chart_card.body.addLayout(top)
        chart_card.body.addWidget(divider())
        self.chart = SimpleChart()
        chart_card.body.addWidget(self.chart, 1)
        body.addWidget(chart_card, 3)

        list_card = Card(margins=(0, 0, 0, 0), spacing=0)
        lt = QHBoxLayout()
        lt.setContentsMargins(18, 14, 18, 12)
        lt.addWidget(label("Corridas de la sesión", "Title"))
        list_card.body.addLayout(lt)
        list_card.body.addWidget(divider())
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["#", "Movimiento", "Duración", "Estado"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        self.table.itemSelectionChanged.connect(self._show_selected)
        list_card.body.addWidget(self.table, 1)
        self.msg = MessageLabel("Cada recorrido ejecutado en Trayectorias se registra aquí.")
        wrap = QVBoxLayout()
        wrap.setContentsMargins(14, 10, 14, 14)
        wrap.addWidget(self.msg)
        list_card.body.addLayout(wrap)
        list_card.setMinimumWidth(360)
        body.addWidget(list_card, 2)
        root.addLayout(body, 1)

        controller.run_finished.connect(self.add_run)

    # ------------------------------------------------------------ corridas
    def add_run(self, run):
        self.runs.append(run)
        if run.get("imu"):
            try:
                carpeta = guardar_corrida(run)
                run["carpeta"] = carpeta
                self.msg.show_message("Corrida con IMU guardada en %s" % carpeta, "ok")
            except OSError as exc:
                self.msg.show_message("No se pudo guardar la corrida con IMU: %s" % exc, "error")
        r = self.table.rowCount()
        self.table.insertRow(r)
        state = "Interrumpida" if run.get("aborted") else "Completa"
        name = run["name"] + ("  · real" if run.get("mode") == "Robot real" else "  · sim.")
        for c, text in enumerate((str(r + 1), name, "%.1f s" % run["duration"], state)):
            it = QTableWidgetItem(text)
            if c in (0, 2):
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(r, c, it)
        self.table.selectRow(r)

    def _selected(self):
        rows = {i.row() for i in self.table.selectedIndexes()}
        if not rows:
            return None
        return self.runs[min(rows)]

    def _show_selected(self):
        run = self._selected()
        if run is None:
            return
        self.t_dur.set("%.2f s" % run["duration"])
        m_ = _muestras(run)
        largo = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(m_[:-1], m_[1:]))
        self.t_len.set("%.0f mm" % largo)
        n_imu = len(run.get("imu") or [])
        self.t_n.set(str(len(_muestras(run))) + (" · IMU %d" % n_imu if n_imu else ""))
        dev = _deviation(run)
        ekf = _muestras_ekf(run)
        dev_e = _deviation(run, ekf) if ekf else None
        if dev is None:
            self.t_err.set("—")
        elif dev_e is None:
            self.t_err.set("%.2f mm" % dev)
        else:
            self.t_err.set("%.2f · EKF %.2f mm" % (dev, dev_e))
        self.msg.show_message(_texto_corrida(run), "")
        self._draw(run)

    def _set_chart(self, kind):
        self.chart_kind = kind
        run = self._selected()
        if run is not None:
            self._draw(run)

    def _draw(self, run):
        smp = _muestras(run)
        t = [s[0] for s in smp]
        if self.chart_kind == "path":
            ref = run.get("reference") or []
            series = []
            if ref:
                series.append(([p[0] for p in ref], [p[1] for p in ref], "#8794A1", True, "Referencia"))
            series.append(([s[1] for s in smp], [s[2] for s in smp], C["accent"], False, "Pulsos emitidos (comando)"))
            ekf = _muestras_ekf(run)
            if ekf:
                series.append(([s[1] for s in ekf], [s[2] for s in ekf], "#D97A1E", False, "Estimado por el EKF"))
            self.chart.set_data(series, "x [mm]", "y [mm]", equal=True)
        elif self.chart_kind == "joints":
            self.chart.set_data([(t, [s[3] for s in smp], C["accent"], False, "θ₁"),
                                 (t, [s[4] for s in smp], "#D97A1E", False, "θ₂")], "tiempo [s]", "ángulo [°]")
        elif self.chart_kind == "imu":
            imu = run.get("imu") or []
            if not imu:
                self.chart.set_data([], "tiempo [s]", "gz [°/s]")
                self.msg.show_message("Esta corrida no tiene datos de la IMU (requiere el firmware con BNO055).", "warn")
                return
            tp, izq, der = _gz_predicho(run)
            series = [([s[0] for s in imu], [s[8] for s in imu], C["accent"], False, "gz medido (IMU)")]
            if tp:
                series.append((tp, izq, "#D97A1E", True, "predicho · distal izquierdo"))
                series.append((tp, der, "#8794A1", True, "predicho · distal derecho"))
            self.chart.set_data(series, "tiempo [s]", "velocidad de giro [°/s]")
        elif self.chart_kind == "ekf":
            imu = run.get("imu") or []
            imu = [s for s in imu if len(s) >= 23]
            if not imu:
                self.chart.set_data([], "tiempo [s]", "Δq [°]")
                self.msg.show_message("Esta corrida no tiene datos del EKF (requiere el firmware 1.3 con la "
                                      "corrección en Sombra o Activa).", "warn")
                return
            ti = [s[0] for s in imu]
            series = [(ti, [s[19] for s in imu], C["accent"], False, "Δq₁ de la ley"),
                      (ti, [s[20] for s in imu], "#D97A1E", False, "Δq₂ de la ley"),
                      (ti, [(s[13] - s[1]) for s in imu], "#8794A1", True, "q̂₁ − q₁ emitido"),
                      (ti, [(s[14] - s[2]) for s in imu], "#B7C0CA", True, "q̂₂ − q₂ emitido")]
            if all(len(s) >= 29 for s in imu) and any(s[25] or s[26] for s in imu):
                series += [(ti, [s[25] for s in imu], "#2E8B57", True, "prealimentación ff₁"),
                           (ti, [s[26] for s in imu], "#7FBF9A", True, "prealimentación ff₂")]
            aplic = sum(1 for s in imu if (s[22] >> 3) & 1)
            self.msg.show_message("Corrección aplicada en %d de %d muestras (%s)." % (
                aplic, len(imu), (run.get("info") or {}).get("correccion_imu", {}).get("modo_texto", "?")), "")
            self.chart.set_data(series, "tiempo [s]", "ángulo [°]")
        else:
            v = [float("nan")]
            for a, b in zip(smp[:-1], smp[1:]):
                dt = b[0] - a[0]
                v.append(math.hypot(b[1] - a[1], b[2] - a[2]) / dt if dt > 1e-6 else float("nan"))
            self.chart.set_data([(t, v, C["accent"], False, "‖v‖ del efector")], "tiempo [s]", "velocidad [mm/s]")

    def export_csv(self):
        run = self._selected()
        if run is None:
            self.msg.show_message("Seleccione una corrida para exportar.", "warn")
            return
        name = "corrida_%d.csv" % (self.runs.index(run) + 1)
        path, _ = QFileDialog.getSaveFileName(self, "Exportar corrida", name, "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_mm", "y_mm", "theta1_deg", "theta2_deg"])
            for s in _muestras(run):
                w.writerow(["%.4f" % s[0], "%.3f" % s[1], "%.3f" % s[2], "%.4f" % s[3], "%.4f" % s[4]])
        self.msg.show_message("Corrida exportada en %s" % path, "ok")
