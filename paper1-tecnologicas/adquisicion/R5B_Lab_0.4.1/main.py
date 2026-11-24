"""Punto de entrada de R5B Lab.

Ejecutar desde la carpeta del proyecto:
    python main.py
"""
import sys

from PySide6.QtWidgets import QApplication

from r5b import theme
from r5b.config import APP_NAME
from r5b.main_window import MainWindow


def main():
    if sys.platform.startswith("win"):
        # para que Windows muestre el icono propio en la barra de tareas
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("r5blab.robot5barras")
        except Exception:  # noqa: BLE001
            pass
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setStyle("Fusion")
    theme.load_fonts()
    app.setFont(theme.sans(10))
    app.setStyleSheet(theme.stylesheet())
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
