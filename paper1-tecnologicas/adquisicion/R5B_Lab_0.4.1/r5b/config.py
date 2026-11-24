"""Configuración general de la aplicación R5B Lab.

En la app las medidas van en milímetros y los ángulos en grados.
El firmware (robot_5bar_esp32.ino / kinematics_5bar.h) trabaja en metros y radianes:
la conversión se hace en robot_state.py al enviar/recibir.

La GEOMETRÍA de abajo es solo el valor inicial: al conectar, la app pide "GEOM" a la
ESP32 y usa la del firmware (kinematics_5bar.h es la única fuente de verdad).
"""

APP_NAME = "R5B Lab"
APP_SUBTITLE = "Robot paralelo de cinco barras"
APP_VERSION = "0.4.1"

# --- Geometría (mm) = kinematics_5bar.h ------------------------------------
L1 = 120.0   # proximal izquierdo (motor O1)
L2 = 180.0   # distal izquierdo
L3 = 180.0   # distal derecho
L4 = 120.0   # proximal derecho (motor O2)
D = 150.0    # separación O1-O2 (centro a centro de ejes)

SIN_MIN = 0.15          # guarda de singularidad |sin(a1 - a2)| (= firmware)
DEFAULT_MODE = 1        # modo de ensamblaje inicial (1 = codos afuera)
MODE_SIGNS = {1: (+1, -1), 2: (-1, +1), 3: (+1, +1), 4: (-1, -1)}
MODE_NAMES = {1: "1 · codos afuera",
              2: "2 · codos adentro",
              3: "3 · izq. afuera / der. adentro",
              4: "4 · izq. adentro / der. afuera"}

# Cero por utillaje (= TH1_HOME_DEG / TH2_HOME_DEG del firmware)
HOME_DEG = (90.0, 90.0)

# Límites blandos de articulación (grados, = firmware)
TH1_LIMITS = (-170.0, 350.0)
TH2_LIMITS = (-170.0, 350.0)

# Trayectorias (= firmware)
MAX_WP_HOST = 130       # puntos máximos por figura
CORNER_DEG = 30.0       # giro a partir del cual el robot frena en un punto
CHUNK_PTS = 8           # puntos por línea PADD
UPLOAD_TIMEOUT_MS = 1500

# Velocidad por defecto del efector (= g_Vmax / g_Amax del firmware)
VMAX_MM_S = 100.0
AMAX_MM_S2 = 200.0
# Topes de los deslizadores (subir por pasos y vigilar alarmas del driver; el firmware puede limitar SETV)
VMAX_SLIDER_MM_S = 500.0
AMAX_SLIDER_MM_S2 = 3000.0

# Alturas usadas solo para dibujar el gemelo 3D [mm]
H_PROXIMAL = 70.0
H_DISTAL = 82.0
H_CUP = 20.0

# --- IMU BNO055 (etapa 1) ----------------------------------------------------
IMU_BUFFER = 36000      # muestras guardadas en memoria (6 min a 100 Hz)
IMU_PREVIO_S = 0.5      # se graba desde 0.5 s antes de cada corrida...
IMU_COLA_S = 2.5        # ...hasta 2.5 s después (para ver cómo se asienta el robot)
IMU_GRABAR_S = [10, 20, 30]   # duraciones de la grabación libre (prueba de golpes), en s
IMU_CARPETA = "Resultados_BNO055"   # dentro de Descargas; las corridas van en la subcarpeta etapa1

# --- Comunicación serial --------------------------------------------------
BAUD_RATES = [115200, 230400, 57600, 9600]
DEFAULT_BAUD = 115200
SILENCE_S = 2.5         # sin líneas del robot durante este tiempo -> "sin respuesta"

# --- Paleta del tema claro --------------------------------------------------
C = {
    "bg": "#F3F5F7",
    "surface": "#FFFFFF",
    "surface_alt": "#F7F9FA",
    "border": "#E1E6EB",
    "border_strong": "#C9D2DB",
    "text": "#16202A",
    "muted": "#56636F",
    "muted2": "#5F6B78",
    "accent": "#0E7C86",
    "accent_hover": "#0B6870",
    "accent_tint": "#E3F1F2",
    "accent_dark_text": "#0B5E66",
    "amber": "#F2A93B",
    "amber_text": "#8A4E00",
    "amber_tint": "#FDF1DE",
    "danger": "#C62828",
    "danger_hover": "#A82020",
    "danger_tint": "#FDECEC",
    "danger_text": "#9B1C1C",
    "success": "#1F8A5B",
    "success_tint": "#E6F4EC",
    "success_border": "#B7DEC8",
    "success_text": "#145C3D",
    "link_dark": "#2A323B",
    "link": "#4A5562",
    "motor": "#5B6875",
    "motor_top": "#8794A1",
    "motor_bottom": "#47525E",
    "grid": "#E1E6EB",
}

FONT_SANS = "IBM Plex Sans"      # si no está instalada, Qt usa Segoe UI
FONT_SANS_FALLBACK = "Segoe UI"
FONT_MONO = "IBM Plex Mono"
FONT_MONO_FALLBACK = "Consolas"
