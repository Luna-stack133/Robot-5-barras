"""Ventana principal: pantalla de inicio y, después de "Comencemos", el menú lateral con las secciones."""
import os

from PySide6.QtCore import Qt
from PySide6.QtGui import QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (QButtonGroup, QFrame, QHBoxLayout, QLabel, QMainWindow, QPushButton, QStackedWidget,
                               QVBoxLayout, QWidget)

from . import theme
from .config import APP_NAME, APP_SUBTITLE, APP_VERSION, C
from .icons import icon, pixmap
from .robot_state import RobotController
from .screens.control import ControlScreen
from .screens.inicio import StartScreen
from .screens.monitoreo import MonitorScreen
from .screens.resultados import ResultsScreen
from .screens.trayectorias import TrajectoryScreen
from .widgets import Card, StatusDot, button, label

SECTIONS = [
    ("monitor", "Monitoreo en vivo"),
    ("path", "Trayectorias"),
    ("control", "Control y estimación"),
    ("results", "Resultados"),
]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("%s %s · %s" % (APP_NAME, APP_VERSION, APP_SUBTITLE))
        ico = os.path.join(os.path.dirname(__file__), "assets", "r5b.ico")
        self.setWindowIcon(QIcon(ico) if os.path.exists(ico) else icon("logo", C["accent"], 64, 2.2))
        self.resize(1440, 900)
        self.setMinimumSize(1180, 740)

        self.ctrl = RobotController(self)
        self.root = QStackedWidget()
        self.root.setObjectName("Root")
        self.setCentralWidget(self.root)

        self.start = StartScreen(self.ctrl)
        self.start.start.connect(self.enter_app)
        self.root.addWidget(self.start)
        self.root.addWidget(self._build_shell())

        esc = QShortcut(QKeySequence("Esc"), self)
        esc.setContext(Qt.ShortcutContext.ApplicationShortcut)
        esc.activated.connect(self._escape)

        self.ctrl.status_changed.connect(self._update_connection)
        self.ctrl.imu_changed.connect(self._update_connection)
        self._update_connection()

    # ------------------------------------------------------------ estructura
    def _build_shell(self):
        shell = QWidget()
        shell.setObjectName("Shell")
        lay = QHBoxLayout(shell)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        side = QFrame()
        side.setObjectName("Sidebar")
        side.setFixedWidth(250)
        sv = QVBoxLayout(side)
        sv.setContentsMargins(14, 20, 14, 18)
        sv.setSpacing(4)
        brand = QHBoxLayout()
        brand.setContentsMargins(8, 0, 0, 18)
        brand.setSpacing(12)
        logo = QLabel()
        logo.setPixmap(pixmap("logo", C["accent"], 34))
        brand.addWidget(logo)
        names = QVBoxLayout()
        names.setSpacing(0)
        names.addWidget(label(APP_NAME, "H2"))
        names.addWidget(label(APP_SUBTITLE, "Small"))
        brand.addLayout(names)
        brand.addStretch(1)
        sv.addLayout(brand)

        self.pages = QStackedWidget()
        self.monitor = MonitorScreen(self.ctrl)
        self.traj = TrajectoryScreen(self.ctrl)
        self.traj.go_monitor.connect(lambda: self.show_section("monitor"))
        self.control = ControlScreen(self.ctrl)
        self.results = ResultsScreen(self.ctrl)
        self.page_by_key = {"monitor": self.monitor, "path": self.traj, "control": self.control,
                            "results": self.results}
        for w in self.page_by_key.values():
            self.pages.addWidget(w)

        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)
        self.nav = {}
        for key, text in SECTIONS:
            b = QPushButton("  " + text)
            b.setObjectName("NavButton")
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _=False, k=key: self.show_section(k))
            self.nav_group.addButton(b)
            self.nav[key] = b
            sv.addWidget(b)
        sv.addStretch(1)

        conn = Card(margins=(14, 12, 14, 12), spacing=6, soft=True)
        conn.body.addWidget(label("CONEXIÓN", "Caps"))
        row = QHBoxLayout()
        row.setSpacing(8)
        self.conn_dot = StatusDot("#8794A1", 8)
        row.addWidget(self.conn_dot)
        self.conn_mode = label("Simulación", "Title")
        self.conn_mode.setWordWrap(True)
        row.addWidget(self.conn_mode)
        row.addStretch(1)
        conn.body.addLayout(row)
        self.conn_port = label("Gemelo digital local", "Small")
        self.conn_port.setStyleSheet("font-family: '%s';" % theme.mono_family())
        conn.body.addWidget(self.conn_port)
        conn.body.addWidget(button("Cambiar conexión", "link", self.back_to_start), 0, Qt.AlignmentFlag.AlignLeft)
        sv.addWidget(conn)

        lay.addWidget(side)
        lay.addWidget(self.pages, 1)
        self.show_section("monitor")
        return shell

    # ------------------------------------------------------------ navegación
    def enter_app(self):
        self.root.setCurrentIndex(1)
        self._update_connection()

    def back_to_start(self):
        if self.ctrl.running:
            self.ctrl.stop_run()
        self.root.setCurrentIndex(0)

    def show_section(self, key):
        self.pages.setCurrentWidget(self.page_by_key[key])
        for k, b in self.nav.items():
            b.setChecked(k == key)
            b.setIcon(icon(k, C["accent"] if k == key else C["muted"], 18))

    def _escape(self):
        if self.root.currentIndex() == 1:
            self.ctrl.emergency_stop()

    def _update_connection(self):
        title, sub, color = self.ctrl.status_text()
        self.conn_dot.set_color(color)
        self.conn_mode.setText(title)
        self.conn_port.setText(sub)

    def closeEvent(self, ev):
        if self.ctrl.real:
            self.ctrl.link.send("STOP", quiet=True)
        self.ctrl.link.close()
        super().closeEvent(ev)
