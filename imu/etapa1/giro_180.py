"""
Prueba de escala del giroscopio: girar TODO el robot un ángulo conocido (180°) sobre la mesa.

El brazo no se mueve respecto a la base, así que la IMU gira exactamente lo mismo que el robot.
Se integra gz entre el reposo de antes y el de después y se compara con el ángulo real:
    k = ángulo medido / ángulo real
No depende de la cinemática ni de ninguna medida del robot.

Uso (desde la carpeta del script):
    python giro_180.py                      usa la grabación más reciente de "Grabar IMU"
    python giro_180.py "ruta\\a\\la\\carpeta"  una grabación concreta
    python giro_180.py --angulo 90          si giraron otro ángulo
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from etapa1_analizar import corridas, leer_imu, leer_info, raiz_etapa1  # noqa: E402


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description="Escala del giroscopio con un giro conocido del robot completo")
    ap.add_argument("carpeta", nargs="?")
    ap.add_argument("--raiz")
    ap.add_argument("--angulo", type=float, default=180.0, help="ángulo real girado, en grados (por defecto 180)")
    a = ap.parse_args()
    if a.carpeta:
        carpeta = os.path.abspath(a.carpeta)
    else:
        raiz = os.path.abspath(a.raiz) if a.raiz else raiz_etapa1()
        grab = [c for c in corridas(raiz) if leer_info(c).get("tipo") == "grabacion"]
        if not grab:
            sys.exit("No hay grabaciones de 'Grabar IMU' en %s" % raiz)
        carpeta = grab[-1]
    d, reparadas = leer_imu(carpeta)
    t, gz = d["t_s"], d["gz"].astype(float)
    fs = 1.0 / np.median(np.diff(t))
    # suavizado de 0.5 s para encontrar dónde está girando
    n = max(3, int(0.5 * fs))
    env = np.convolve(np.abs(gz), np.ones(n) / n, mode="same")
    girando = np.where(env > 2.0)[0]
    if len(girando) == 0:
        sys.exit("No se ve ningún giro en %s" % os.path.basename(carpeta))
    i0, i1 = girando[0], girando[-1]
    antes = slice(0, max(0, i0 - int(0.3 * fs)))
    despues = slice(min(len(gz), i1 + int(0.3 * fs)), len(gz))
    if (antes.stop or 0) < int(1.0 * fs) or len(gz) - despues.start < int(1.0 * fs):
        sys.exit("Hace falta al menos 1 s quieto antes y después del giro. Repitan la grabación.")
    # sesgo: promedio (no mediana: gz viene en pasos de 1/16 °/s) de todo el reposo
    sesgo = float(np.mean(np.concatenate([gz[antes], gz[despues]])))
    dt = np.diff(t, prepend=t[0] - 1.0 / fs)
    ang = np.cumsum((gz - sesgo) * dt)
    # el ángulo se toma promediando 1 s justo antes y justo después del giro
    w = int(1.0 * fs)
    medido = float(np.mean(ang[despues.start:despues.start + w]) - np.mean(ang[antes.stop - w:antes.stop]))
    k = abs(medido) / a.angulo
    print("\n=== %s ===" % os.path.basename(carpeta))
    print("  Giro entre t = %.1f s y t = %.1f s | sesgo %.3f °/s | muestras reparadas %d"
          % (t[i0], t[i1], sesgo, reparadas))
    print("  Ángulo medido por el giroscopio: %.2f° (real: %.1f°)" % (medido, a.angulo))
    print("  Escala del giroscopio k = %.4f  (error de %+.1f %%)" % (k, (k - 1) * 100))
    print("\n  Para la calibración: python calibrar_cinematica.py --solo-pausa --k %.4f "
          "--libres d1_deg,d2_deg,L1,L2,L3,L4" % k)


if __name__ == "__main__":
    main()
