"""Tema visual claro de R5B Lab: fuentes y hoja de estilos (QSS)."""
import os

from PySide6.QtGui import QFont, QFontDatabase

from .config import C, FONT_MONO, FONT_MONO_FALLBACK, FONT_SANS, FONT_SANS_FALLBACK

_FONTS = {"sans": FONT_SANS_FALLBACK, "mono": FONT_MONO_FALLBACK}


def load_fonts():
    """Carga las fuentes de r5b/assets/fonts (si las hay) y elige las familias disponibles.

    Para el aspecto exacto del diseño, copie los .ttf de IBM Plex Sans e IBM Plex Mono
    (descarga gratuita en Google Fonts) en la carpeta r5b/assets/fonts.
    """
    folder = os.path.join(os.path.dirname(__file__), "assets", "fonts")
    if os.path.isdir(folder):
        for name in os.listdir(folder):
            if name.lower().endswith((".ttf", ".otf")):
                QFontDatabase.addApplicationFont(os.path.join(folder, name))
    families = set(QFontDatabase.families())
    _FONTS["sans"] = FONT_SANS if FONT_SANS in families else FONT_SANS_FALLBACK
    _FONTS["mono"] = FONT_MONO if FONT_MONO in families else FONT_MONO_FALLBACK


def sans(size=10, weight=QFont.Weight.Normal):
    f = QFont(_FONTS["sans"])
    f.setPointSizeF(size)
    f.setWeight(weight)
    return f


def mono(size=10, weight=QFont.Weight.Normal):
    f = QFont(_FONTS["mono"])
    f.setPointSizeF(size)
    f.setWeight(weight)
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def mono_family():
    return _FONTS["mono"]


def stylesheet():
    c = C
    mono_f = _FONTS["mono"]
    return f"""
QWidget {{ color: {c['text']}; }}
QMainWindow, QWidget#Root, QWidget#Shell, QWidget#Page {{ background: {c['bg']}; }}
QScrollArea, QScrollArea > QWidget > QWidget {{ background: transparent; border: none; }}
QToolTip {{ background: {c['text']}; color: white; border: none; padding: 6px 8px; }}

/* ---------- barra lateral ---------- */
QFrame#Sidebar {{ background: {c['surface']}; border: none; border-right: 1px solid {c['border']}; }}
QPushButton#NavButton {{
    text-align: left; padding: 11px 14px; border: none; border-radius: 8px;
    color: {c['muted']}; background: transparent; font-size: 10.5pt;
}}
QPushButton#NavButton:hover {{ background: {c['surface_alt']}; color: {c['text']}; }}
QPushButton#NavButton:checked {{ background: {c['accent_tint']}; color: {c['text']}; font-weight: 600; }}

/* ---------- tarjetas ---------- */
QFrame#Card {{ background: {c['surface']}; border: 1px solid {c['border']}; border-radius: 14px; }}
QFrame#CardSoft {{ background: {c['surface_alt']}; border: 1px solid {c['border']}; border-radius: 10px; }}
QFrame#Divider {{ background: {c['border']}; border: none; max-height: 1px; min-height: 1px; }}

/* ---------- textos ---------- */
QLabel#H1 {{ font-size: 17pt; font-weight: 600; }}
QLabel#H2 {{ font-size: 12pt; font-weight: 600; }}
QLabel#Title {{ font-size: 10.5pt; font-weight: 600; }}
QLabel#Muted {{ color: {c['muted2']}; }}
QLabel#Small {{ color: {c['muted2']}; font-size: 8.5pt; }}
QLabel#Caps {{ color: {c['muted2']}; font-size: 8.5pt; font-weight: 600; }}
QLabel#Readout {{ font-family: "{mono_f}"; font-size: 15pt; }}
QLabel#ReadoutBig {{ font-family: "{mono_f}"; font-size: 17pt; }}
QLabel#Mono {{ font-family: "{mono_f}"; }}

QLabel#Msg {{ padding: 9px 11px; border-radius: 8px; background: {c['surface_alt']};
              border: 1px solid {c['border']}; color: {c['muted']}; }}
QLabel#Msg[state="error"] {{ background: {c['danger_tint']}; border-color: #F3C2C2; color: {c['danger_text']}; }}
QLabel#Msg[state="ok"] {{ background: {c['success_tint']}; border-color: {c['success_border']}; color: {c['success_text']}; }}
QLabel#Msg[state="warn"] {{ background: {c['amber_tint']}; border-color: #F1D3A3; color: {c['amber_text']}; }}

/* ---------- botones ---------- */
QPushButton {{
    background: {c['surface']}; color: {c['text']}; border: 1px solid {c['border_strong']};
    border-radius: 8px; padding: 9px 16px; font-weight: 500; min-height: 20px;
}}
QPushButton:hover {{ background: {c['surface_alt']}; }}
QPushButton:pressed {{ background: #EEF1F4; }}
QPushButton:disabled {{ color: #9AA5B1; background: {c['surface_alt']}; border-color: {c['border']}; }}

QPushButton[variant="primary"] {{ background: {c['accent']}; color: white; border: 1px solid {c['accent']}; font-weight: 600; }}
QPushButton[variant="primary"]:hover {{ background: {c['accent_hover']}; border-color: {c['accent_hover']}; }}
QPushButton[variant="primary"]:disabled {{ background: #9CC9CD; border-color: #9CC9CD; color: white; }}

QPushButton[variant="outline"] {{ color: {c['accent']}; border: 1px solid {c['accent']}; font-weight: 600; }}
QPushButton[variant="outline"]:hover {{ background: {c['accent_tint']}; }}

QPushButton[variant="danger"] {{
    background: {c['danger']}; color: white; border: 1px solid {c['danger']};
    font-weight: 700; padding: 11px 20px;
}}
QPushButton[variant="danger"]:hover {{ background: {c['danger_hover']}; border-color: {c['danger_hover']}; }}

QPushButton[variant="link"] {{ background: transparent; border: none; color: {c['muted']}; text-decoration: underline; }}
QPushButton[variant="link"]:hover {{ color: {c['text']}; }}

QPushButton[variant="jog"] {{ font-size: 16pt; font-weight: 500; min-width: 48px; min-height: 36px; padding: 4px; border-radius: 10px; }}
QPushButton[variant="jogxy"] {{ font-weight: 600; min-height: 34px; border-radius: 10px; }}
QPushButton[variant="jog"]:pressed, QPushButton[variant="jogxy"]:pressed {{ background: {c['accent_tint']}; border-color: {c['accent']}; }}

QPushButton[variant="chip"] {{ border-radius: 8px; padding: 8px 6px; color: {c['muted']}; font-family: "{mono_f}"; }}
QPushButton[variant="chip"]:checked {{ border: 1px solid {c['accent']}; background: {c['accent_tint']}; color: {c['text']}; font-weight: 600; }}

QPushButton[variant="seg"] {{ border: none; border-radius: 6px; padding: 7px 14px; background: transparent; color: {c['muted']}; }}
QPushButton[variant="seg"]:checked {{ background: {c['surface']}; color: {c['text']}; font-weight: 600; border: 1px solid {c['border']}; }}
QFrame#Segmented {{ background: {c['bg']}; border: 1px solid {c['border']}; border-radius: 9px; }}

QPushButton[variant="tab"] {{ border: none; border-radius: 8px; padding: 10px 18px; background: transparent; color: {c['muted']}; }}
QPushButton[variant="tab"]:hover {{ background: {c['surface_alt']}; color: {c['text']}; }}
QPushButton[variant="tab"]:checked {{ background: {c['accent']}; color: white; font-weight: 600; }}

QPushButton[variant="enable"] {{ border-radius: 10px; padding: 11px 16px; }}
QPushButton[variant="enable"]:checked {{ background: {c['success_tint']}; border-color: {c['success_border']}; color: {c['success_text']}; }}

QToolButton#FigureButton {{
    background: {c['surface']}; border: 1px solid {c['border_strong']}; border-radius: 10px;
    padding: 8px 4px; color: {c['muted']};
}}
QToolButton#FigureButton:hover {{ background: {c['surface_alt']}; }}
QToolButton#FigureButton:checked {{ border: 2px solid {c['accent']}; background: {c['accent_tint']}; color: {c['accent_dark_text']}; font-weight: 600; }}

/* ---------- campos ---------- */
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {{
    background: {c['surface']}; border: 1px solid {c['border_strong']}; border-radius: 8px;
    padding: 7px 10px; min-height: 20px; selection-background-color: {c['accent']};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border: 1px solid {c['accent']}; }}
QDoubleSpinBox, QSpinBox {{ font-family: "{mono_f}"; }}
QComboBox QAbstractItemView {{ background: white; border: 1px solid {c['border_strong']};
    selection-background-color: {c['accent_tint']}; selection-color: {c['text']}; outline: none; }}

QSlider::groove:horizontal {{ height: 6px; background: {c['border']}; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {c['accent']}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: white; border: 2px solid {c['accent']}; width: 16px; height: 16px;
    margin: -7px 0; border-radius: 10px; }}

QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 18px; height: 18px; border-radius: 4px; border: 1px solid {c['border_strong']}; background: white; }}
QCheckBox::indicator:checked {{ background: {c['accent']}; border-color: {c['accent']}; }}

/* ---------- tablas y consola ---------- */
QTableWidget {{ background: white; border: none; gridline-color: {c['border']};
    selection-background-color: {c['accent_tint']}; selection-color: {c['text']}; }}
QHeaderView::section {{ background: {c['surface']}; color: {c['muted2']}; border: none;
    border-bottom: 1px solid {c['border']}; padding: 8px 10px; font-weight: 500; }}
QPlainTextEdit#Console {{ background: #FBFCFD; border: 1px solid {c['border']}; border-radius: 8px;
    font-family: "{mono_f}"; font-size: 9pt; color: {c['muted']}; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {c['border_strong']}; border-radius: 4px; min-height: 30px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {c['border_strong']}; border-radius: 4px; min-width: 30px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
"""
