"""Puerto serial con la ESP32 (texto por líneas, 115200 bps).

Esta capa solo abre/cierra el puerto, envía líneas y entrega las líneas recibidas.
La interpretación del protocolo del firmware está en robot_state.py.
"""
from PySide6.QtCore import QObject, QTimer, Signal

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # la app sigue funcionando en modo simulación
    serial = None
    list_ports = None


def available_ports():
    """Lista de (dispositivo, descripción) de los puertos seriales del equipo.

    La descripción incluye VID:PID para distinguir el USB nativo de la ESP32-S3
    (VID 303A) del conversor USB-UART (CH340/CH343/CP210x).
    """
    if list_ports is None:
        return []
    out = []
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        desc = (p.description or "").replace(p.device, "").strip(" ()-") or p.device
        if p.vid is not None:
            desc += "  [%04X:%04X]" % (p.vid, p.pid or 0)
        out.append((p.device, desc))
    return out


class RobotLink(QObject):
    line = Signal(str)                # cada línea recibida (sin salto de línea)
    log = Signal(str)                 # texto para la consola (sin la telemetría STATE)
    connection_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port = None
        self.port_name = ""
        self.baud = 0
        self._buf = b""
        self._rx_timer = QTimer(self)
        self._rx_timer.setInterval(15)
        self._rx_timer.timeout.connect(self._read_incoming)

    @property
    def connected(self):
        return self.port is not None and self.port.is_open

    def open(self, port_name, baud):
        """Abre el puerto. Devuelve (ok, mensaje)."""
        if serial is None:
            return False, "Falta la librería pyserial (pip install pyserial)."
        self.close(quiet=True)
        try:
            self.port = serial.Serial(port_name, baud, timeout=0, write_timeout=0.5)
        except Exception as exc:  # noqa: BLE001
            self.port = None
            msg = str(exc)
            if "PermissionError" in msg or "Acceso denegado" in msg or "Access is denied" in msg:
                msg += "\n¿Está abierto el Monitor Serie de Arduino u otro programa con ese puerto?"
            return False, "No se pudo abrir %s: %s" % (port_name, msg)
        self.port_name, self.baud = port_name, baud
        self._buf = b""
        self._rx_timer.start()
        self.log.emit("Puerto %s abierto a %d baudios" % (port_name, baud))
        self.connection_changed.emit(True)
        return True, "Puerto %s abierto" % port_name

    def close(self, quiet=False):
        was = self.port is not None
        if self.port is not None:
            try:
                self.port.close()
            except Exception:  # noqa: BLE001
                pass
        self.port = None
        self._rx_timer.stop()
        if was:
            self.log.emit("Puerto %s cerrado" % self.port_name)
        if was or not quiet:
            self.connection_changed.emit(False)

    def send(self, text, quiet=False):
        """Envía una línea. Devuelve False si no hay conexión o falla la escritura."""
        if not self.connected:
            return False
        try:
            self.port.write((text + "\n").encode("ascii", errors="replace"))
        except Exception as exc:  # noqa: BLE001
            self.log.emit("Error al enviar: %s" % exc)
            self.close()
            return False
        if not quiet:
            self.log.emit("→ " + (text if len(text) < 90 else text[:87] + "..."))
        return True

    def _read_incoming(self):
        if not self.connected:
            return
        try:
            n = self.port.in_waiting
            data = self.port.read(n or 1)
        except Exception as exc:  # noqa: BLE001  (cable desconectado, placa reiniciada...)
            self.log.emit("Conexión perdida: %s" % exc)
            self.close()
            return
        if not data:
            return
        self._buf += data
        while b"\n" in self._buf:
            raw, self._buf = self._buf.split(b"\n", 1)
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if not text.startswith(("STATE", "I ")):      # telemetría y muestras de la IMU: no a la consola
                self.log.emit("← " + text)
            self.line.emit(text)
        if len(self._buf) > 8192:          # basura sin saltos de línea
            self._buf = b""
