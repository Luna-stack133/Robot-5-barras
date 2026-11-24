"""Estado del robot y órdenes de movimiento.

`RobotController` es el único punto por donde pasan los movimientos. Tiene dos modos:

  * ROBOT REAL (puerto abierto): traduce cada acción al protocolo del firmware
    robot_5bar_esp32.ino (JOG, SETHOME, GOTO, PBEGIN/PADD/PRUN, SETV, MODE, STOP) y toma
    la postura de la telemetría "STATE ..." que la ESP32 envía a 20 Hz.
  * SIMULACIÓN (sin puerto): mueve un gemelo local con la misma cinemática.

IMU (etapa 1): si el firmware tiene la BNO055, al conectar se pide "IMU 1" y la ESP32 envía
una línea "I ..." a 100 Hz con la IMU y los ángulos de los motores del mismo instante. Cada
corrida del robot real guarda esas muestras (desde 0.5 s antes hasta 2.5 s después, para ver
cómo se asienta el robot) y se graban solas en Descargas/Resultados_BNO055/etapa1.

Grabación libre de la IMU (prueba de golpes): `record_imu(segundos)` graba la IMU con el robot
quieto durante el tiempo pedido, sin mover nada, y la guarda como una corrida más con
"tipo": "grabacion" en info.json.

Corrección con IMU (firmware 1.3): el firmware corre un EKF en la ESP32 y puede sumar una
corrección Δq a los pulsos. La app la configura con los comandos "CTL ..." (`set_control`), lee su
estado de las líneas "CTL modo=..." y guarda la configuración vigente en info.json de cada corrida.
Con el firmware 1.3 cada línea "I" trae además la estimación del EKF y la corrección calculada.

Unidades: la app usa mm y grados; el firmware m y grados (STATE) / m (GOTO, PADD, SETV).
"""
import collections
import math
import time

from PySide6.QtCore import QObject, QTimer, Signal

from . import config
from . import kinematics as kin
from .paths import densify, limit_points
from .robot_link import RobotLink

MM = 1000.0


class RobotController(QObject):
    pose_changed = Signal()            # cambió la postura
    status_changed = Signal()          # conexión, estado del firmware, ejecución, bloqueo
    message = Signal(str, bool)        # texto, es_error
    run_finished = Signal(dict)        # registro de la corrida terminada
    imu_changed = Signal()             # cambió el estado de la IMU (conectada, sin IMU)
    geometry_changed = Signal()        # cambió la geometría o el modo de ensamblaje
    ctl_changed = Signal()             # cambió el estado de la corrección con IMU (firmware 1.3)

    TICK_MS = 20

    def __init__(self, parent=None):
        super().__init__(parent)
        self.link = RobotLink(self)
        self.link.line.connect(self._on_line)
        self.link.connection_changed.connect(self._on_connection)

        start = kin.ik(75.0, 220.0) or kin.ik(*kin.home_point())
        self.t1, self.t2 = start
        self.x, self.y = 75.0, 220.0
        self.zero = config.HOME_DEG          # ángulos del utillaje
        self.enabled = True                  # bloqueo de movimiento en la app
        self.vacuum = False
        self.vmax = config.VMAX_MM_S
        self.amax = config.AMAX_MM_S2
        self.trail = []
        self.preview = []

        # robot real
        self.fw_state = None                 # INIT / IDLE / JOGGING / MOVING / HALTED
        self.last_rx = None
        self.responding = False
        self.geom_ok = False
        self._geom_tries = 0
        self._conn_t0 = 0.0
        self._upload = None
        self._upload_seq = 0
        self._seq = None                     # cola de GOTO de una secuencia
        self._seq_pausa = 0.0                # s de pausa en cada punto de la secuencia
        self._seq_token = 0                  # invalida pausas pendientes de una secuencia anterior
        self._run = None
        self._rec_seq = 0                    # identifica cada grabación libre de la IMU

        # IMU (BNO055 en el firmware)
        self.imu_ok = None                   # None: sin preguntar / True: enviando / False: no hay IMU
        self.imu_buf = collections.deque(maxlen=config.IMU_BUFFER)
        self.imu_last = None                 # última muestra (para mostrarla en pantalla)

        # corrección con IMU (firmware 1.3) y prealimentación (1.4)
        self.ctl = None                      # None: sin preguntar / False: firmware sin CTL / dict: estado
        self.ctl_modelo = None               # parámetros del modelo del servo que usa el firmware (1.4)
        self.fw_version = ""
        self.carga_g = 0.0                   # masa agregada en el efector (solo se anota en cada corrida)

        # simulación
        self._queue = []
        self._cur = None
        self._timer = QTimer(self)
        self._timer.setInterval(self.TICK_MS)
        self._timer.timeout.connect(self._tick)

        self._watch = QTimer(self)
        self._watch.setInterval(1000)
        self._watch.timeout.connect(self._watchdog)

    # =================================================================== consultas
    @property
    def real(self):
        return self.link.connected

    @property
    def homed(self):
        return (not self.real) or (self.fw_state not in (None, "INIT"))

    @property
    def running(self):
        if self.real:
            return self._run is not None or self._upload is not None or self.fw_state == "MOVING"
        return self._timer.isActive()

    @property
    def mode(self):
        return kin.mode()

    def effector(self):
        return self.x, self.y

    def points(self):
        return kin.points(self.t1, self.t2, (self.x, self.y))

    @property
    def recording(self):
        """True mientras hay una grabación libre de la IMU (prueba de golpes)."""
        return self._run is not None and self._run.get("tipo") == "grabacion"

    def recording_left(self):
        """Segundos que faltan de la grabación libre (0 si no hay)."""
        if not self.recording:
            return 0.0
        return max(0.0, self._run["dur_obj"] - (time.perf_counter() - self._run["t0"]))

    def status_text(self):
        if not self.real:
            return "Simulación", "Gemelo digital local", "#8794A1"
        port = "%s · %d baud" % (self.link.port_name, self.link.baud)
        if self.imu_ok:
            port += " · IMU"
        if isinstance(self.ctl, dict):
            port += {1: " · EKF en sombra", 2: " · corrección IMU activa"}.get(self.ctl.get("modo"), "")
            if self.ctl.get("ff"):
                port += " · prealimentación"
            if self.carga_g:
                port += " · carga %g g" % self.carga_g
        if not self.responding:
            return "Sin respuesta", port, config.C["danger"]
        st = self.fw_state or "conectado"
        if self.recording:
            return "Grabando IMU", port, config.C["accent"]
        if st == "INIT":
            return "Falta fijar el cero", port, "#D9A04A"
        if st == "HALTED":
            return "Detenido (HALTED)", port, "#D9A04A"
        return "Conectado · %s" % st, port, config.C["success"]

    # =================================================================== conexión
    def _on_connection(self, on):
        self._upload = None
        self._seq = None
        if self._run is not None:
            self._finish_run(aborted=True)
        if on:
            self._timer.stop()
            self.fw_state = None
            self.last_rx = None
            self.responding = False
            self.geom_ok = False
            self._geom_tries = 0
            self._conn_t0 = time.monotonic()
            self.trail = []
            # al abrir el puerto la ESP32 puede reiniciarse: se pregunta varias veces
            QTimer.singleShot(300, lambda: self._send("PING", quiet=True))
            QTimer.singleShot(600, lambda: self._send("GEOM", quiet=True))
            QTimer.singleShot(800, lambda: self._send("MODE", quiet=True))
            QTimer.singleShot(1000, lambda: self._send("STATUS", quiet=True))
            QTimer.singleShot(1200, lambda: self._send("IMU 1", quiet=True))
            QTimer.singleShot(1400, lambda: self._send("CTL", quiet=True))
            self.imu_ok = None
            self.ctl = None
            self.imu_buf.clear()
            self._watch.start()
        else:
            self._watch.stop()
            self.fw_state = None
            self.responding = False
            self.imu_ok = None
            self.ctl = None
        self.ctl_changed.emit()
        self.imu_changed.emit()
        self.status_changed.emit()
        self.pose_changed.emit()

    def _watchdog(self):
        if not self.real:
            return
        now = time.monotonic()
        if not self.geom_ok and self._geom_tries < 5 and self.last_rx is not None:
            self._geom_tries += 1
            self._send("GEOM", quiet=True)
        ref = self.last_rx if self.last_rx is not None else self._conn_t0
        ok = (now - ref) <= config.SILENCE_S
        if ok != self.responding:
            self.responding = ok
            if not ok:
                self.message.emit("La ESP32 no responde. Pulse RESET en la placa, cierre el Monitor Serie de "
                                  "Arduino y verifique que sea el puerto correcto.", True)
            self.status_changed.emit()

    def _send(self, text, quiet=False):
        return self.link.send(text, quiet=quiet)

    # =================================================================== recepción
    def _on_line(self, line):
        self.last_rx = time.monotonic()
        if not self.responding:
            self.responding = True
            self.status_changed.emit()
        parts = line.split()
        head = parts[0] if parts else ""

        if head == "I" and len(parts) >= 14:
            self._on_imu(parts)
            return

        if head == "CTL" and len(parts) >= 2 and ("=" in parts[1] or parts[1] == "modelo"):
            self._on_ctl(parts[1:])
            return

        if head == "STATE" and len(parts) >= 7:
            try:
                st = parts[1]
                x, y = float(parts[2]) * MM, float(parts[3]) * MM
                t1, t2 = float(parts[4]), float(parts[5])
            except ValueError:
                return
            changed = st != self.fw_state
            self.fw_state = st
            self.t1, self.t2, self.x, self.y = t1, t2, x, y
            if st == "MOVING" or st == "JOGGING":
                self._add_trail((x, y))
            if self._run is not None:
                self._run["samples"].append((time.perf_counter() - self._run["t0"], x, y, t1, t2))
            self.pose_changed.emit()
            if changed:
                self.status_changed.emit()
            return

        if head == "MODE" and len(parts) >= 2 and parts[1].isdigit():
            if kin.set_mode(int(parts[1])):
                self.message.emit("Modo de ensamblaje %s activo en el robot." % parts[1], False)
                self.geometry_changed.emit()
            self.status_changed.emit()
            return

        if head == "GEOM" and len(parts) >= 6:
            try:
                vals = [float(v) * MM for v in parts[1:6]]
            except ValueError:
                return
            self.geom_ok = True
            if kin.set_geometry(*vals):
                self.message.emit("Geometría del firmware aplicada: L1=%.0f L2=%.0f L3=%.0f L4=%.0f D=%.0f mm"
                                  % tuple(vals), False)
                self.geometry_changed.emit()
            return

        # carga de figura por partes
        if self._upload is not None:
            if line.startswith("OK pbegin") or line.startswith("OK padd"):
                self._upload_next()
                return
            if head == "ERR":
                self._upload = None
                self._abort_run("Carga de la figura cancelada: " + line[4:])
                return

        if head == "EVT" and len(parts) >= 2:
            evt = parts[1]
            if evt == "DONE":
                if self._seq and self._seq_pausa > 0:
                    tok = self._seq_token
                    QTimer.singleShot(int(self._seq_pausa * 1000), lambda: self._seq_after_pause(tok))
                elif self._seq:
                    self._seq_next()
                elif self._run is not None:
                    self._finish_run()
                    self.message.emit("Movimiento terminado.", False)
            elif evt == "HOMED":
                self.message.emit("Cero fijado. Ya puede retirar el utillaje.", False)
            elif evt == "LIMIT":
                self._abort_run("El robot se detuvo: un punto quedó fuera de límites o cerca de una singularidad.")
            elif evt == "CTL_OFF":
                motivo = parts[2] if len(parts) > 2 else "?"
                textos = {"IMU_INVALIDA": "la IMU entregó lecturas inválidas",
                          "SIN_LECTURA": "la IMU dejó de responder",
                          "INNOVACION": "el EKF dejó de coincidir con el giroscopio",
                          "EKF": "el EKF falló"}
                self.message.emit("La corrección con IMU se desactivó sola (%s): pasó a modo sombra."
                                  % textos.get(motivo, motivo), True)
                if self._run is not None:
                    self._run.setdefault("ctl_eventos", []).append(
                        [round(time.perf_counter() - self._run["t0"], 3), motivo])
            self.status_changed.emit()
            return

        # IMU: confirmación, ausencia y reinicio de la ESP32
        if line.startswith("OK imu"):
            on = line.strip().endswith("1")
            if on and self.imu_ok is not True:
                self.message.emit("IMU conectada: BNO055 a 100 Hz, se graba con cada corrida.", False)
            self.imu_ok = True if on else self.imu_ok
            self.imu_changed.emit()
            return
        if line.startswith("ERR imu"):
            if self.imu_ok is not False:
                self.message.emit("El firmware no tiene IMU disponible: las corridas se graban sin IMU.", False)
            self.imu_ok = False
            self.imu_changed.emit()
            return
        if line.startswith("OK ready"):
            # la ESP32 se reinició: volver a pedir geometría, modo, IMU y corrección
            self.message.emit("Robot: " + line[3:], False)
            m = [p for p in parts if p.startswith("v") and p[1:2].isdigit()]
            self.fw_version = m[0][1:] if m else "1.x"
            QTimer.singleShot(200, lambda: self._send("GEOM", quiet=True))
            QTimer.singleShot(400, lambda: self._send("MODE", quiet=True))
            QTimer.singleShot(600, lambda: self._send("IMU 1", quiet=True))
            QTimer.singleShot(800, lambda: self._send("CTL", quiet=True))
            if self.ctl:
                self.ctl = None
                self.ctl_changed.emit()
            return

        # firmware 1.x: no conoce CTL (sin corrección con IMU)
        if head == "ERR" and "unknown cmd" in line and line.rstrip().endswith("CTL"):
            if self.ctl is not False:
                self.ctl = False
                self.ctl_changed.emit()
            return
        if head == "OK" and len(parts) >= 2 and parts[1] == "ctl":
            return                              # la confirmación llega también como línea CTL
        if head == "ERR" and len(parts) >= 2 and parts[1].startswith("ctl"):
            self.message.emit("Corrección con IMU: " + line[4:], True)
            return

        if head == "ERR":
            self.message.emit("Robot: " + line[4:], True)
            if self._run is not None and self.fw_state != "MOVING":
                self._abort_run(None)
            return
        if head in ("WARN",):
            self.message.emit("Robot: " + line[5:], True)
            return
        if head == "OK":
            text = line[3:]
            if text not in ("PONG",) and not text.startswith(("setv", "jog")):
                self.message.emit("Robot: " + text, False)
            return

    def _on_imu(self, parts):
        """I t_us q1_mdeg q2_mdeg ax ay az gx gy gz rumbo roll pitch cal  (enteros del firmware).
        Firmware 1.3 agrega: q1h q2h w1h w2h d1 d2 dl1 dl2 inov estado  (EKF y corrección).
        Firmware 1.4 agrega: q1p q2p ff1 ff2 da1 da2  (plan y lo sumado a los pulsos, milésimas de grado)."""
        try:
            v = [int(p) for p in parts[1:14]]
        except ValueError:
            return
        s = (time.perf_counter(), v[0], v[1] / 1000.0, v[2] / 1000.0,
             v[3] / 100.0, v[4] / 100.0, v[5] / 100.0,
             v[6] / 16.0, v[7] / 16.0, v[8] / 16.0,
             v[9] / 16.0, v[10] / 16.0, v[11] / 16.0, v[12])
        if len(parts) >= 24:
            try:
                e = [int(p) for p in parts[14:24]]
            except ValueError:
                e = None
            if e is not None:
                # q̂ [°], ω̂ [°/s], d̂ [mN·m], Δq de la ley [°], innovación del giroscopio [°/s], estado
                s = s + (e[0] / 1000.0, e[1] / 1000.0, e[2] / 100.0, e[3] / 100.0, e[4] / 10.0, e[5] / 10.0,
                         e[6] / 1000.0, e[7] / 1000.0, e[8] / 100.0, e[9])
                if len(parts) >= 30:
                    try:
                        f = [int(p) for p in parts[24:30]]
                    except ValueError:
                        f = None
                    if f is not None:
                        # plan [°], prealimentación sumada [°], corrección realmente sumada [°]
                        s = s + tuple(x / 1000.0 for x in f)
        self.imu_buf.append(s)
        self.imu_last = s
        if self.imu_ok is not True:
            self.imu_ok = True
            self.imu_changed.emit()

    def _on_ctl(self, kv):
        """CTL modo=2 ley=ALFA alfa=0.300 zona=1.00 dmax=1.000 lavado=1.00 ekf=1 t_ekf=812/1350 [ff=0 ffgan=1.00 ffmax=10.0]
        CTL modelo K=1.3346/0.8651 B=0.03569/0.06636 ff=0.609 tau=0.0234   (firmware 1.4)"""
        if kv and kv[0] == "modelo":
            m = {}
            for item in kv[1:]:
                k, _, v = item.partition("=")
                try:
                    m[k] = [float(x) for x in v.split("/")] if "/" in v else float(v)
                except ValueError:
                    pass
            self.ctl_modelo = m
            self.ctl_changed.emit()
            return
        st = {}
        for item in kv:
            if "=" not in item:
                continue
            k, v = item.split("=", 1)
            if k == "t_ekf":
                a, _, b = v.partition("/")
                try:
                    st["t_ekf_us"], st["t_ekf_max_us"] = int(a), int(b or 0)
                except ValueError:
                    pass
                continue
            try:
                st[k] = int(v) if k in ("modo", "ekf", "ff") else (float(v) if k != "ley" else v)
            except ValueError:
                st[k] = v
        if "modo" not in st:
            return
        if not self.fw_version:
            self.fw_version = "1.3"           # solo el firmware 1.3 (o posterior) responde CTL
        self.ctl = st
        self.ctl_changed.emit()

    @property
    def ctl_available(self):
        return self.real and isinstance(self.ctl, dict)

    @property
    def ff_available(self):
        """El firmware conectado tiene la prealimentación por modelo (1.4 o posterior)."""
        return self.ctl_available and "ff" in self.ctl

    def set_control(self, modo=None, ley=None, alfa=None, zona=None, dmax=None, lavado=None,
                    ff=None, ffgan=None):
        """Configura la corrección con IMU del firmware 1.3 (comandos CTL). Devuelve False si no se pudo."""
        if not self.real or not self.responding:
            self.message.emit("La corrección con IMU necesita el robot conectado.", True)
            return False
        if self.ctl is False:
            self.message.emit("El firmware conectado no tiene la corrección con IMU: cargue el firmware 1.3.", True)
            return False
        if modo and not self.imu_ok:
            self.message.emit("No llegan datos de la IMU: la corrección no se puede encender.", True)
            return False
        cmds = []
        if ley is not None:
            cmds.append("CTL LEY %s" % ("LQR" if str(ley).upper() == "LQR" else "ALFA"))
        if alfa is not None:
            cmds.append("CTL ALFA %.3f" % alfa)
        if zona is not None:
            cmds.append("CTL ZONA %.3f" % zona)
        if dmax is not None:
            cmds.append("CTL DMAX %.3f" % dmax)
        if lavado is not None:
            cmds.append("CTL LAVADO %.3f" % lavado)
        if (ff is not None or ffgan is not None) and not self.ff_available:
            if ff:
                self.message.emit("El firmware conectado no tiene la prealimentación: cargue el firmware 1.4.", True)
                return False
            ff = ffgan = None
        if ffgan is not None:
            cmds.append("CTL FFGAN %.3f" % ffgan)
        if ff is not None:
            cmds.append("CTL FF %d" % (1 if ff else 0))
        if modo is not None:
            cmds.append("CTL MODO %d" % int(modo))     # el modo al final: ya con los parámetros nuevos
        for k, c in enumerate(cmds):
            QTimer.singleShot(40 * k, lambda c=c: self._send(c, quiet=True))
        return True

    def _add_trail(self, p):
        self.trail.append(p)
        if len(self.trail) > 6000:
            del self.trail[:1500]

    # =================================================================== órdenes
    def _ready(self, need_home=True):
        if not self.enabled:
            self.message.emit("Movimiento bloqueado: habilítelo para mover el robot.", True)
            return False
        if self.real:
            if not self.responding:
                self.message.emit("La ESP32 no está respondiendo.", True)
                return False
            if need_home and not self.homed:
                self.message.emit("Primero fije el cero con el utillaje (tarjeta 'Cero del robot').", True)
                return False
        return True

    def jog_joint(self, d1, d2):
        """Giro relativo de cada motor (grados). En el robot real usa JOG (funciona sin cero)."""
        if not self._ready(need_home=False):
            return
        if self.running and not (self.real and self.fw_state == "JOGGING"):
            self.message.emit("Hay un movimiento en curso. Espere o pulse Parada.", True)
            return
        if self.real:
            if d1:
                self._send("JOG 1 %.3f" % d1)
            if d2:
                self._send("JOG 2 %.3f" % d2)
            return
        t1, t2 = self.t1 + d1, self.t2 + d2
        p = kin.fk_near(t1, t2, (self.x, self.y))
        if p is None or not kin.joint_limits_ok(t1, t2):
            self.message.emit("Postura no válida: la cadena no cierra o se exceden los límites.", True)
            return
        self._set_pose(t1, t2, p)
        self.message.emit("Movimiento enviado.", False)

    def jog_xy(self, dx, dy):
        self.move_to(self.x + dx, self.y + dy, quiet=True)

    def move_to(self, x, y, quiet=False):
        """Mover el efector a (x, y) mm en línea recta (GOTO en el firmware)."""
        if not self._ready():
            return False
        if kin.quality_xy(x, y) is None:
            self.message.emit("(%.1f, %.1f) mm está fuera de alcance o muy cerca de una singularidad." % (x, y), True)
            return False
        if self.real:
            if self._run is not None or self._upload is not None:
                self.message.emit("Hay un recorrido en ejecución. Deténgalo primero.", True)
                return False
            return self._send("GOTO %.4f %.4f" % (x / MM, y / MM), quiet=quiet)
        if self.running:
            self.message.emit("Hay un recorrido en ejecución. Deténgalo primero.", True)
            return False
        q = kin.ik(x, y)
        self._set_pose(q[0], q[1], (x, y))
        if not quiet:
            self.message.emit("Movimiento enviado.", False)
        return True

    def set_zero(self, t1, t2):
        """Cero por utillaje: los brazos están en (t1, t2) grados. No mueve el robot."""
        self.zero = (t1, t2)
        if self.real:
            if not self.responding:
                self.message.emit("La ESP32 no está respondiendo.", True)
                return
            self._send("SETHOME %.3f %.3f" % (t1, t2))
            return
        p = kin.home_point(t1, t2)
        if p is None:
            self.message.emit("Con esos ángulos los brazos no pueden cerrar la cadena.", True)
            return
        m = kin.detect_mode(p[0], p[1], t1, t2)
        if m and kin.set_mode(m):
            self.geometry_changed.emit()
        self._set_pose(t1, t2, p)
        self.message.emit("Cero fijado en θ₁ = %.1f°, θ₂ = %.1f° (modo %d)." % (t1, t2, kin.mode()), False)
        self.status_changed.emit()

    def go_zero(self):
        p = kin.home_point(*self.zero)
        if p is None:
            self.message.emit("La pose de cero no es válida.", True)
            return
        if kin.quality_xy(*p) is None:
            self.message.emit("La pose del utillaje (%.0f, %.0f) mm está en el borde del área de trabajo: "
                              "el firmware no permite llegar ahí con GOTO. Use JOG si lo necesita." % p, True)
            return
        self.move_to(*p)

    def request_mode(self, m):
        """Cambiar el modo de ensamblaje (codos)."""
        if m == kin.mode():
            return
        if self.real:
            if self.running:
                self.message.emit("Espere a que termine el movimiento.", True)
                return
            self._send("MODE %d" % m)
            return
        q = kin.ik(self.x, self.y, m)
        if q is None or kin.sing_measure(self.x, self.y, q[0], q[1]) < config.SIN_MIN:
            self.message.emit("El punto actual no es alcanzable (o es singular) en el modo %d." % m, True)
            return
        kin.set_mode(m)
        self.geometry_changed.emit()
        self._start_sim([("joint", q)], "Cambio a modo %d" % m)

    def set_speed(self, vmax_mm_s, amax_mm_s2, send=True):
        self.vmax, self.amax = float(vmax_mm_s), float(amax_mm_s2)
        if send and self.real and self.responding:
            self._send("SETV %.4f %.4f" % (self.vmax / MM, self.amax / MM), quiet=True)

    def record_imu(self, seconds):
        """Graba la IMU con el robot quieto (prueba de golpes). No mueve el robot."""
        if not self.real:
            self.message.emit("La grabación de la IMU necesita el robot conectado.", True)
            return False
        if not self.responding:
            self.message.emit("La ESP32 no está respondiendo.", True)
            return False
        if not self.imu_ok:
            self.message.emit("No llegan datos de la IMU. Revise que el firmware sea el que tiene la BNO055 "
                              "y el cableado del I2C.", True)
            return False
        if self.running:
            self.message.emit("Hay un movimiento en curso. Espere a que termine.", True)
            return False
        seconds = max(2.0, float(seconds))
        self._rec_seq += 1
        seq = self._rec_seq
        name = "Golpes (%.0f, %.0f) mm" % (self.x, self.y)
        self._run = {"name": name, "t0": time.perf_counter(), "samples": [], "reference": [],
                     "tipo": "grabacion", "seq": seq, "dur_obj": seconds,
                     "pose0": (self.x, self.y, self.t1, self.t2),
                     "ctl0": dict(self.ctl) if isinstance(self.ctl, dict) else self.ctl,
                     "modelo0": dict(self.ctl_modelo) if isinstance(self.ctl_modelo, dict) else None,
                     "carga0": self.carga_g}
        self.message.emit("Grabando la IMU %.0f s: den golpecitos suaves al efector, en el plano del robot, "
                          "separados unos 2 s." % seconds, False)
        QTimer.singleShot(int(seconds * 1000), lambda: self._end_recording(seq))
        self.status_changed.emit()
        return True

    def _end_recording(self, seq):
        run = self._run
        if run is None or run.get("seq") != seq:
            return                            # ya se detuvo o es otra grabación
        self._finish_run()
        self.message.emit("Grabación terminada: quedó en Resultados y en la carpeta etapa1.", False)

    def set_enabled(self, on):
        self.enabled = bool(on)
        if not self.enabled:
            self.stop_run(silent=True)
        self.message.emit("Movimiento habilitado." if on else "Movimiento bloqueado.", False)
        self.status_changed.emit()

    def set_vacuum(self, on):
        if self.real:
            self.message.emit("El firmware todavía no tiene un comando para la ventosa.", True)
            self.status_changed.emit()
            return
        self.vacuum = bool(on)
        self.status_changed.emit()
        self.pose_changed.emit()

    def emergency_stop(self):
        self._upload = None
        self._seq = None
        if self.real:
            self._send("STOP")
        self._timer.stop()
        if self._run is not None:
            self._finish_run(aborted=True)
        self.enabled = False
        self.message.emit("PARADA: movimiento detenido y bloqueado. Habilítelo para continuar.", True)
        self.status_changed.emit()

    def stop_run(self, silent=False):
        self._upload = None
        self._seq = None
        if self.real:
            self._send("STOP")
        if self._timer.isActive():
            self._timer.stop()
        if self._run is not None:
            self._finish_run(aborted=True)
        if not silent:
            self.message.emit("Movimiento detenido.", False)
        self.status_changed.emit()

    def clear_trail(self):
        self.trail = []
        self.pose_changed.emit()

    # =================================================================== recorridos
    def run(self, plan):
        """Ejecuta un plan de Trayectorias:
            {"kind": "figure", "points": [...], "name": str}
            {"kind": "goto", "target": (x, y), "name": str}
            {"kind": "joint", "target": (t1, t2), "name": str}        (solo simulación)
            {"kind": "sequence", "items": [(x, y, acción)], "name": str}
        """
        if not self._ready():
            return False
        if self.running:
            self.message.emit("Ya hay un movimiento en ejecución.", True)
            return False
        kind, name = plan["kind"], plan["name"]
        preview = plan.get("preview") or []
        if self.real:
            self.set_speed(self.vmax, self.amax)
            self.preview = list(preview)
            self.trail = []
            self._run = {"name": name, "t0": time.perf_counter(), "samples": [], "reference": list(preview),
                         "pose0": (self.x, self.y, self.t1, self.t2),
                         "ctl0": dict(self.ctl) if isinstance(self.ctl, dict) else self.ctl,
                     "modelo0": dict(self.ctl_modelo) if isinstance(self.ctl_modelo, dict) else None,
                     "carga0": self.carga_g}
            if kind == "figure":
                self._upload_path(limit_points(plan["points"]))
            elif kind == "goto":
                self._send("GOTO %.4f %.4f" % (plan["target"][0] / MM, plan["target"][1] / MM))
            elif kind == "sequence":
                items = list(plan["items"])
                if any(a != "Mover" for _, _, a in items):
                    self.message.emit("Aviso: el firmware aún no controla la ventosa; se recorren los puntos sin "
                                      "tomar ni soltar.", True)
                self._seq = [(x, y) for x, y, _ in items]
                self._seq_pausa = max(0.0, float(plan.get("pause_s", 0) or 0))
                self._seq_token += 1
                self._run["pausa_s"] = self._seq_pausa
                self._seq_next()
            else:
                self._run = None
                self.message.emit("Este tipo de movimiento solo está disponible en simulación.", True)
                return False
            self.status_changed.emit()
            return True

        # simulación
        if kind == "figure":
            steps = [("path", plan["points"])]
        elif kind == "goto":
            steps = [("path", densify([(self.x, self.y), plan["target"]], step=1.0))]
        elif kind == "joint":
            steps = [("joint", plan["target"])]
        else:
            steps = []
            for x, y, action in plan["items"]:
                steps.append(("path", [(x, y)]))
                if action == "Tomar":
                    steps += [("vacuum", True), ("wait", 0.4)]
                elif action == "Soltar":
                    steps += [("vacuum", False), ("wait", 0.4)]
        return self._start_sim(steps, name, preview)

    # ---------------- robot real: figura por partes (PBEGIN / PADD / PRUN)
    def _upload_path(self, pts):
        chunks = []
        for i in range(0, len(pts), config.CHUNK_PTS):
            chunks.append("PADD " + " ".join("%.4f %.4f" % (x / MM, y / MM) for x, y in pts[i:i + config.CHUNK_PTS]))
        self._upload_seq += 1
        self._upload = {"chunks": chunks, "i": 0, "seq": self._upload_seq}
        self.message.emit("Enviando figura: %d puntos en %d partes..." % (len(pts), len(chunks)), False)
        self._send("PBEGIN %d" % len(pts), quiet=True)
        self._arm_upload_timeout()

    def _upload_next(self):
        up = self._upload
        if up is None:
            return
        if up["i"] < len(up["chunks"]):
            self._send(up["chunks"][up["i"]], quiet=True)
            up["i"] += 1
            self._arm_upload_timeout()
        else:
            self._upload = None
            self._send("PRUN")

    def _arm_upload_timeout(self):
        up = self._upload
        if up is None:
            return
        key = (up["seq"], up["i"])

        def check():
            cur = self._upload
            if cur is not None and (cur["seq"], cur["i"]) == key:
                self._upload = None
                self._abort_run("El robot no confirmó la carga de la figura.")
        QTimer.singleShot(config.UPLOAD_TIMEOUT_MS, check)

    # ---------------- robot real: secuencia de GOTO
    def _seq_next(self):
        if not self._seq:
            self._seq = None
            if self._run is not None:
                self._finish_run()
                self.message.emit("Secuencia terminada.", False)
            return
        x, y = self._seq.pop(0)
        self._send("GOTO %.4f %.4f" % (x / MM, y / MM))

    def _seq_after_pause(self, tok):
        if tok == self._seq_token and self._seq and self._run is not None:
            self._seq_next()

    def _abort_run(self, text):
        self._seq = None
        if self._run is not None:
            self._finish_run(aborted=True)
        if text:
            self.message.emit(text, True)
        self.status_changed.emit()

    def _finish_run(self, aborted=False):
        run, self._run = self._run, None
        if run is None:
            return
        run["duration"] = time.perf_counter() - run["t0"]
        run["aborted"] = aborted
        run["mode"] = "Robot real" if self.real else "Simulación"
        pts = [(s[1], s[2]) for s in run["samples"]]
        run["length"] = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
        run["info"] = {
            "nombre": run["name"], "fecha": time.strftime("%Y-%m-%d %H:%M:%S"),
            "geometria_mm": dict(kin.GEOM), "modo_ensamblaje": kin.mode(),
            "vmax_mm_s": self.vmax, "amax_mm_s2": self.amax, "interrumpida": aborted,
            "duracion_s": run["duration"], "tipo": run.get("tipo", "corrida"),
        }
        if run.get("pausa_s"):
            run["info"]["pausa_s"] = run["pausa_s"]
        run["info"]["carga_g"] = run.get("carga0", self.carga_g)
        if self.real:
            c0 = run.get("ctl0")
            if isinstance(c0, dict):
                ctl = {k: c0.get(k) for k in ("modo", "ley", "alfa", "zona", "dmax", "lavado")}
                if "ff" in c0:
                    ctl["prealimentacion"] = c0.get("ff")
                    ctl["prealimentacion_ganancia"] = c0.get("ffgan")
                if isinstance(run.get("modelo0"), dict):
                    ctl["modelo_servo"] = run["modelo0"]
                ctl["modo_texto"] = {0: "apagada", 1: "sombra", 2: "activa"}.get(c0.get("modo"), "?")
                ctl["eventos"] = run.get("ctl_eventos", [])
                run["info"]["correccion_imu"] = ctl
            else:
                run["info"]["correccion_imu"] = {
                    "modo": 0, "modo_texto": "no disponible (firmware 1.x)" if c0 is False or self.ctl is False
                    else "desconocido (el firmware no respondió a CTL)"}
            if self.fw_version or self.ctl is False:
                run["info"]["firmware"] = self.fw_version or "1.x"
        if run.get("pose0"):
            x0, y0, a1, a2 = run["pose0"]
            run["info"].update({"x_mm": round(x0, 2), "y_mm": round(y0, 2),
                                "theta1_deg": round(a1, 3), "theta2_deg": round(a2, 3)})
        if self.real and self.imu_ok and run.get("tipo") == "grabacion":
            self._emit_run_with_imu(run)      # la grabación ya cubre el tiempo pedido: sin cola
        elif self.real and self.imu_ok:
            # se sigue grabando la IMU un momento para ver cómo se asienta el robot
            QTimer.singleShot(int(config.IMU_COLA_S * 1000), lambda r=run: self._emit_run_with_imu(r))
        else:
            self.run_finished.emit(run)
        self.status_changed.emit()

    def _emit_run_with_imu(self, run):
        t_ini = run["t0"] - config.IMU_PREVIO_S
        muestras = [s for s in self.imu_buf if s[0] >= t_ini]
        if muestras:
            # tiempo del reloj de la ESP32 (exacto entre muestras), referido al inicio de la corrida
            host0, us0 = muestras[0][0], muestras[0][1]
            base = host0 - run["t0"]
            imu = []
            for s in muestras:
                dus = (s[1] - us0) % (1 << 32)
                imu.append((base + dus * 1e-6,) + s[2:])
            run["imu"] = imu
        self.run_finished.emit(run)

    # =================================================================== simulación
    def _set_pose(self, t1, t2, p):
        self.t1, self.t2 = t1, t2
        self.x, self.y = p
        self._add_trail(p)
        if self._run is not None:
            self._run["samples"].append((time.perf_counter() - self._run["t0"], p[0], p[1], t1, t2))
        self.pose_changed.emit()

    def _start_sim(self, steps, name, preview=None):
        self._queue = list(steps)
        self._cur = None
        self.preview = list(preview or [])
        self.trail = []
        self._run = {"name": name, "t0": time.perf_counter(), "samples": [], "reference": list(preview or [])}
        self._timer.start()
        self.message.emit("Ejecutando: %s" % name, False)
        self.status_changed.emit()
        return True

    def _next_step(self):
        while self._queue:
            kind, arg = self._queue.pop(0)
            if kind == "vacuum":
                self.vacuum = bool(arg)
                self.status_changed.emit()
                continue
            if kind == "wait":
                self._cur = {"kind": "wait", "left": float(arg)}
                return True
            if kind == "joint":
                self._cur = {"kind": "joint", "start": (self.t1, self.t2), "goal": tuple(arg), "s": 0.0,
                             "ref": (self.x, self.y)}
                return True
            if kind == "path":
                pts = list(arg)
                if not pts:
                    continue
                x, y = self.x, self.y
                lead = densify([(x, y), pts[0]], step=1.0) if math.hypot(pts[0][0] - x, pts[0][1] - y) > 0.5 else []
                full = lead + pts
                cum = [0.0]
                for a, b in zip(full[:-1], full[1:]):
                    cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
                self._cur = {"kind": "path", "pts": full, "cum": cum, "s": 0.0, "i": 0}
                return True
        return False

    def _tick(self):
        dt = self.TICK_MS / 1000.0
        if self._cur is None and not self._next_step():
            self._timer.stop()
            self._finish_run()
            self.message.emit("Recorrido terminado.", False)
            return
        c = self._cur
        if c["kind"] == "wait":
            c["left"] -= dt
            if c["left"] <= 0:
                self._cur = None
            return
        if c["kind"] == "joint":
            (a1, a2), (b1, b2) = c["start"], c["goal"]
            span = max(abs(b1 - a1), abs(b2 - a2), 1e-6)
            c["s"] = min(1.0, c["s"] + 40.0 * dt / span)
            t1, t2 = a1 + (b1 - a1) * c["s"], a2 + (b2 - a2) * c["s"]
            p = kin.fk_near(t1, t2, (self.x, self.y))
            if p is None:
                self._timer.stop()
                self._abort_run("Movimiento interrumpido: la cadena no cierra en el camino.")
                return
            self._set_pose(t1, t2, p)
            if c["s"] >= 1.0:
                self._cur = None
            return
        c["s"] += self.vmax * dt
        cum, pts = c["cum"], c["pts"]
        if c["s"] >= cum[-1]:
            x, y = pts[-1]
            done = True
        else:
            i = c["i"]
            while cum[i + 1] < c["s"]:
                i += 1
            c["i"] = i
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 0 else (c["s"] - cum[i]) / seg
            x = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t
            y = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t
            done = False
        q = kin.ik(x, y)
        if q is None:
            self._timer.stop()
            self._abort_run("Recorrido interrumpido: el punto (%.1f, %.1f) mm está fuera de alcance." % (x, y))
            return
        self._set_pose(q[0], q[1], (x, y))
        if done:
            self._cur = None
