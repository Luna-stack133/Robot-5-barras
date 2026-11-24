"""Rutas del repositorio (todo relativo a la carpeta paper1-tecnologicas)."""
import os, sys
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATOS = os.path.join(RAIZ, "data", "raw")          # S1..S4/Lecturas, video_tracking, bitacora.xlsx
RES = os.path.join(RAIZ, "results")                # salidas numéricas (csv/json)
FIGS = os.path.join(RAIZ, "figures")               # figuras (png a 300 dpi)
LIB = os.path.join(RAIZ, "scripts", "lib")
for p in (LIB,):
    if p not in sys.path: sys.path.insert(0, p)
os.makedirs(RES, exist_ok=True); os.makedirs(FIGS, exist_ok=True)
def ruta_res(n): return os.path.join(RES, n)
