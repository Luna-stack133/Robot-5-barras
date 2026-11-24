"""
Calibración cinemática con el video de la secuencia de puntos con pausa.

Para cada reposo del comando (últimos 0.8 s de cada pausa) se toma la posición real del marcador en el
video (en mm, rectificada con el propio cuadro de 20 mm). Luego se ajustan, por mínimos cuadrados:
  - la posición y el giro de la cámara respecto al robot (3 parámetros, siempre),
  - y, según el modelo, los ceros de los motores, las longitudes y D.
La escala la fija el marcador (no se ajusta), así que las longitudes sí son identificables.

Uso:  python calib_video.py <video_marcador.csv> <carpeta_secuencia_1> [<carpeta_secuencia_2> ...]
Las secuencias deben estar en el video en ese orden.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
import modelo5b as m  # noqa: E402
import video_real as vr  # noqa: E402


def leer(carpeta):
    d = np.genfromtxt(os.path.join(carpeta, "imu.csv"), delimiter=",", names=True)
    with open(os.path.join(carpeta, "info.json"), encoding="utf-8") as f:
        info = json.load(f)
    return d["t_s"].astype(float), np.radians(d["q1_deg"]), np.radians(d["q2_deg"]), info


def reposos_comando(t, q1, q2, minimo=1.2):
    """Intervalos en que el comando está quieto al menos `minimo` s."""
    mov = np.r_[True, (np.abs(np.diff(q1)) + np.abs(np.diff(q2))) > 1e-9]
    quieto = ~mov
    out, i0 = [], None
    for i in range(len(t)):
        if quieto[i] and i0 is None:
            i0 = i
        if (not quieto[i] or i == len(t) - 1) and i0 is not None:
            if t[i - 1] - t[i0] >= minimo:
                out.append((t[i0], t[i - 1], q1[i0], q2[i0]))
            i0 = None
    return out


def fk_g(q1, q2, p):
    g = {"L1": p["L1"], "L2": p["L2"], "L3": p["L3"], "L4": p["L4"], "D": p["D"]}
    return np.array(m.fk(q1 + p["d1"], q2 + p["d2"], g)[2]) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("marcador_csv")
    ap.add_argument("secuencias", nargs="+")
    ap.add_argument("--salida", default=".")
    a = ap.parse_args()

    v = np.genfromtxt(a.marcador_csv, delimiter=",", names=True)
    G, Pv, rms_cuadro = vr.rectificar(v)
    tv = v["t_s"]
    okv = np.all(np.isfinite(Pv), axis=1)
    print("Video: %d cuadros, marcador en %.0f %%, cuadro de 20 mm tras rectificar: error RMS %.2f mm"
          % (len(tv), 100 * okv.mean(), rms_cuadro))

    # rapidez del efector en el video (huecos rellenados por interpolación)
    Pi = np.column_stack([np.interp(tv, tv[okv], Pv[okv, k]) for k in range(2)])
    rap = np.hypot(np.gradient(Pi[:, 0], tv), np.gradient(Pi[:, 1], tv))
    rap = np.convolve(rap, np.ones(9) / 9, mode="same")
    rej = np.arange(tv[0], tv[-1], 0.01)
    sv = np.interp(rej, tv, rap)

    # sincronización de cada secuencia: correlación de la rapidez comandada con la del video.
    # Las secuencias van en orden: cada una debe dejar espacio para las siguientes.
    datos = [leer(c) for c in a.secuencias]
    durs = [d_[0][-1] - d_[0][0] for d_ in datos]
    puntos = []          # (q1, q2, x_v, y_v, sec, k, sx, sy)
    desde = tv[0] - 5
    for s, carpeta in enumerate(a.secuencias):
        t, q1, q2, info = datos[s]
        rep = reposos_comando(t, q1, q2)
        Pc = np.array([m.fk(x1, x2)[2] for x1, x2 in zip(q1, q2)]) * 1000.0
        sc = np.hypot(np.gradient(Pc[:, 0], t), np.gradient(Pc[:, 1], t))
        hasta = tv[-1] - sum(durs[s:]) + 6
        mejor = None
        for d in np.arange(desde, hasta, 0.02):
            mv = np.interp(t + d, rej, sv, left=np.nan, right=np.nan)
            ok = np.isfinite(mv)
            if ok.sum() < 0.9 * len(t):
                continue
            r = np.corrcoef(mv[ok], sc[ok])[0, 1]
            if mejor is None or r > mejor[0]:
                mejor = (r, d)
        r, d = mejor
        print("Secuencia %d (%s): %d reposos del comando | sincronizada con r = %.2f (inicio en el video: %.1f s)"
              % (s + 1, os.path.basename(carpeta), len(rep), r, d + t[0]))
        desde = d + durs[s] - 6
        for k, (t0, t1, a1, a2) in enumerate(rep):
            w = (tv >= t1 + d - 0.8) & (tv <= t1 + d) & okv
            if w.sum() < 20:
                print("   reposo %2d: marcador no visible (fuera del cuadro o perdido)" % k)
                continue
            puntos.append((a1, a2, np.median(Pv[w, 0]), np.median(Pv[w, 1]), s, k,
                           float(np.std(Pv[w, 0])), float(np.std(Pv[w, 1]))))
    P = np.array(puntos)
    print("Reposos útiles: %d" % len(P))

    nominal = {"L1": 120.0e-3, "L2": 180.0e-3, "L3": 180.0e-3, "L4": 120.0e-3, "D": 150.0e-3, "d1": 0.0, "d2": 0.0}
    modelos = {
        "nominal (solo cámara)": [],
        "ceros": ["d1", "d2"],
        "ceros + proximales (L1=L4)": ["d1", "d2", "Lp"],
        "ceros + proximales + distales (L2=L3)": ["d1", "d2", "Lp", "Ld"],
        "ceros + proximales + distales + D": ["d1", "d2", "Lp", "Ld", "D"],
        "todo libre (L1, L2, L3, L4, D)": ["d1", "d2", "L1", "L2", "L3", "L4", "D"],
    }

    def armar(x, libres):
        p = dict(nominal)
        for k, val in zip(libres, x[3:]):
            if k == "Lp":
                p["L1"] = p["L4"] = val
            elif k == "Ld":
                p["L2"] = p["L3"] = val
            else:
                p[k] = val
        return p

    def res(x, libres, P):
        th, tx, ty = x[:3]
        p = armar(x, libres)
        c, s_ = math.cos(th), math.sin(th)
        out = []
        for a1, a2, xv, yv, *_ in P:
            xm, ym = fk_g(a1, a2, p)
            out += [c * xv - s_ * yv + tx - xm, s_ * xv + c * yv + ty - ym]
        return np.array(out)

    # punto de partida del giro/desplazamiento: probar varios giros con el modelo nominal
    best = None
    for esp in (False, True):
        Q = P.copy()
        if esp:
            Q[:, 3] = -Q[:, 3]
        for th0 in np.radians(np.arange(0, 360, 20)):
            sol = least_squares(res, [th0, 0, 0], args=([], Q))
            if best is None or sol.cost < best[0]:
                best = (sol.cost, sol.x, esp)
    _, x0, esp = best
    if esp:
        P[:, 3] = -P[:, 3]
    print("Espejo de la imagen: %s" % esp)

    resultados = {}
    print("\n%-42s %9s %9s  parámetros" % ("Modelo", "RMS mm", "máx mm"))
    for nombre, libres in modelos.items():
        ini = list(x0) + [{"d1": 0, "d2": 0, "Lp": 0.12, "Ld": 0.18, "D": 0.15, "L1": 0.12, "L2": 0.18,
                           "L3": 0.18, "L4": 0.12}[k] for k in libres]
        sol = least_squares(res, ini, args=(libres, P), x_scale=[0.01, 1, 1] + [0.01] * len(libres))
        r = sol.fun.reshape(-1, 2)
        e = np.hypot(r[:, 0], r[:, 1])
        J = sol.jac
        dof = max(1, len(sol.fun) - len(sol.x))
        cov = np.linalg.pinv(J.T @ J) * (sol.fun @ sol.fun) / dof
        sd = np.sqrt(np.diag(cov))
        p = armar(sol.x, libres)
        txt = []
        for k, val, s in zip(libres, sol.x[3:], sd[3:]):
            if k in ("d1", "d2"):
                txt.append("%s=%.2f±%.2f°" % (k, math.degrees(val), math.degrees(s)))
            else:
                txt.append("%s=%.1f±%.1f mm" % (k, val * 1000, s * 1000))
        print("%-42s %9.2f %9.2f  %s" % (nombre, np.sqrt(np.mean(e ** 2)), e.max(), ", ".join(txt)))
        resultados[nombre] = {"rms_mm": float(np.sqrt(np.mean(e ** 2))), "max_mm": float(e.max()),
                              "parametros": {k: float(v) for k, v in zip(libres, sol.x[3:])},
                              "incertidumbre": {k: float(s) for k, s in zip(libres, sd[3:])},
                              "camara": [float(x) for x in sol.x[:3]], "errores_mm": e.tolist()}
        resultados[nombre]["_sol"] = sol.x

    # repetibilidad: el mismo punto comandado visitado varias veces (posición del video, sin modelo)
    print("\nRepetibilidad (mismo punto comandado, posición medida en el video):")
    claves = {}
    for i, (a1, a2, xv, yv, s, k, sx, sy) in enumerate(P):
        claves.setdefault((round(a1, 4), round(a2, 4)), []).append((xv, yv, s, k))
    difs = []
    for kk, lst in claves.items():
        if len(lst) >= 2:
            X = np.array([[l[0], l[1]] for l in lst])
            dmax = max(np.hypot(*(X[i] - X[j])) for i in range(len(X)) for j in range(i + 1, len(X)))
            difs.append(dmax)
    if difs:
        difs = np.array(difs)
        print("  %d puntos visitados 2 o más veces: diferencia media %.2f mm, máxima %.2f mm"
              % (len(difs), difs.mean(), difs.max()))
    ruido = np.hypot(P[:, 6], P[:, 7])
    print("  ruido del video en reposo (desviación en 0.8 s): mediana %.2f mm" % np.median(ruido))

    # gráfica
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(14, 6.5))
    for ax, nombre in zip(axs, ["nominal (solo cámara)", "ceros + proximales + distales + D"]):
        x = resultados[nombre]["_sol"]
        libres = modelos[nombre]
        th, tx, ty = x[:3]
        p = armar(x, libres)
        c, s_ = math.cos(th), math.sin(th)
        for a1, a2, xv, yv, s, k, *_ in P:
            xm, ym = fk_g(a1, a2, p)
            xr, yr = c * xv - s_ * yv + tx, s_ * xv + c * yv + ty
            ax.plot(xm, ym, "o", mfc="none", color="#8794A1", ms=7)
            ax.plot(xr, yr, ".", color=["#0E7C86", "#D9480F", "#7048E8"][int(s) % 3], ms=6)
            ax.annotate("", xy=(xm + 10 * (xr - xm), ym + 10 * (yr - ym)), xytext=(xm, ym),
                        arrowprops=dict(arrowstyle="->", color="#D9480F", lw=0.8))
        ax.set_aspect("equal")
        ax.set_title("%s · RMS %.2f mm\n(círculo = modelo, punto = video, flecha = error ×10)"
                     % (nombre, resultados[nombre]["rms_mm"]))
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm]")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(a.salida, exist_ok=True)
    fig.savefig(os.path.join(a.salida, "calibracion_video.png"), dpi=110)
    for r in resultados.values():
        r.pop("_sol")
    with open(os.path.join(a.salida, "calibracion_video.json"), "w", encoding="utf-8") as f:
        json.dump({"espejo": bool(esp), "rms_cuadro_mm": rms_cuadro, "modelos": resultados}, f, indent=2,
                  ensure_ascii=False)
    print("\nResultados en %s" % a.salida)


if __name__ == "__main__":
    main()
