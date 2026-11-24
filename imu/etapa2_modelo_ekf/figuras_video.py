"""
Métricas de seguimiento de figuras (cuadrado, círculo) medidas con video, sin modelo dinámico.

Un video puede contener varias corridas seguidas (se pasan en el orden en que se hicieron).
Para cada corrida:
  1. Sincronización: correlación de la rapidez comandada con la rapidez del marcador en el video.
  2. Alineación al marco del robot: giro + desplazamiento (y espejo) que mejor superponen el recorrido
     del video sobre el comando con el mejor retardo. La escala NO se ajusta: la fija el marcador.
  3. Métricas:
     - error de contorno: distancia del punto real a la figura comandada (RMS y máximo), en movimiento;
     - escala aparente de la figura (ajuste con escala libre, solo informativo);
     - retardo respecto al comando;
     - parada final: desviación máxima respecto al reposo final y tiempo hasta quedar dentro de
       ±1 mm y ±0.5 mm;
     - error de posición final respecto al punto comandado.

Uso:  python figuras_video.py <video_marcador.csv> <corrida1> [<corrida2> ...] [--salida carpeta]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402
import video_real as vr  # noqa: E402


def leer(carpeta):
    d = np.genfromtxt(os.path.join(carpeta, "imu.csv"), delimiter=",", names=True)
    with open(os.path.join(carpeta, "info.json"), encoding="utf-8") as f:
        info = json.load(f)
    t = d["t_s"].astype(float)
    q1d, q2d = d["q1_deg"].astype(float), d["q2_deg"].astype(float)
    if "q1_plan_deg" in d.dtype.names:
        # firmware 1.4: el plan viene registrado (los pulsos llevan la prealimentación y la corrección)
        q1d, q2d = d["q1_plan_deg"].astype(float), d["q2_plan_deg"].astype(float)
    elif "dq1_ley_deg" in d.dtype.names:
        # firmware 1.3: los pulsos emitidos llevan la corrección; el comando (plan) es pulsos − Δq aplicada
        apl = d["ctl_aplicando"].astype(float)
        q1d, q2d = q1d - apl * d["dq1_ley_deg"], q2d - apl * d["dq2_ley_deg"]
    q1, q2 = np.radians(q1d), np.radians(q2d)
    P = np.array([m.fk(a, b)[2] for a, b in zip(q1, q2)]) * 1000.0
    # inicio y fin del movimiento según el plan (telemetría STATE), si existe: con la corrección activa los
    # pulsos emitidos siguen cambiando después de que el plan terminó
    tel = os.path.join(carpeta, "telemetria.csv")
    if os.path.exists(tel):
        T = np.genfromtxt(tel, delimiter=",", names=True)
        dP = np.hypot(np.diff(T["x_mm"]), np.diff(T["y_mm"]))
        k = np.where(dP > 0.01)[0]
        if len(k):
            info["_t_ini_plan"] = float(T["t_s"][k[0]])
            info["_t_fin_plan"] = float(T["t_s"][k[-1] + 1])
    return t, P, info


def transf(par, P, esp):
    th, tx, ty = par
    Q = P * [-1, 1] if esp else P
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([c * Q[:, 0] - s * Q[:, 1] + tx, s * Q[:, 0] + c * Q[:, 1] + ty])


def asentamiento(tt, dev, tol):
    """Tiempo (desde tt[0]) después del cual la desviación queda siempre dentro de tol."""
    fuera = np.where(dev > tol)[0]
    if len(fuera) == 0:
        return 0.0, True
    j = fuera[-1]
    if j >= len(tt) - 3:
        return float("nan"), False
    return float(tt[j + 1] - tt[0]), True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("marcador_csv")
    ap.add_argument("corridas", nargs="+")
    ap.add_argument("--salida", default=".")
    ap.add_argument("--etiqueta", default="figuras")
    ap.add_argument("--sinc-libre", action="store_true", help="no usar las horas de las carpetas para sincronizar")
    ap.add_argument("--sinc-rafagas", action="store_true",
                    help="asignar cada corrida, en orden, a una ráfaga de movimiento del video")
    a = ap.parse_args()

    v = np.genfromtxt(a.marcador_csv, delimiter=",", names=True)
    _, Pv, rms_cuadro = vr.rectificar(v)
    tv = v["t_s"]
    okv = np.all(np.isfinite(Pv), axis=1)
    Pi = np.column_stack([np.interp(tv, tv[okv], Pv[okv, k]) for k in range(2)])
    rap = np.convolve(np.hypot(np.gradient(Pi[:, 0], tv), np.gradient(Pi[:, 1], tv)), np.ones(5) / 5, mode="same")
    rej = np.arange(tv[0], tv[-1], 0.005)
    sv = np.interp(rej, tv, rap)
    print("%s: %d cuadros, marcador en %.0f %%, cuadro de 20 mm tras rectificar: RMS %.2f mm"
          % (os.path.basename(a.marcador_csv), len(tv), 100 * okv.mean(), rms_cuadro))

    datos = [leer(c) for c in a.corridas]

    def hora(carpeta):
        b_ = os.path.basename(carpeta.rstrip("/\\"))
        hh, mm, ss = b_[11:19].split("-")
        return int(hh) * 3600 + int(mm) * 60 + int(ss)

    T = np.array([hora(c) for c in a.corridas], dtype=float)
    T -= T[0]

    def curva(s):
        t, P, _ = datos[s]
        sc = np.hypot(np.gradient(P[:, 0], t), np.gradient(P[:, 1], t))

        def r_de(d):
            mv = np.interp(t + d, rej, sv, left=np.nan, right=np.nan)
            ok = np.isfinite(mv)
            if ok.sum() < 0.6 * len(t):
                return -1.0
            return float(np.corrcoef(mv[ok], sc[ok])[0, 1])
        return r_de

    rdes = [curva(s) for s in range(len(datos))]
    if a.sinc_rafagas:
        # cada corrida corresponde, en orden, a una ráfaga de movimiento del video (se descartan las
        # ráfagas pequeñas); se ajusta el desfase en ±1 s alrededor del inicio de la ráfaga
        mv = rap > 15
        idx = np.where(mv)[0]
        raf = []
        if len(idx):
            i0 = i1 = idx[0]
            for i in idx[1:]:
                if tv[i] - tv[i1] > 0.8:
                    raf.append((i0, i1)); i0 = i
                i1 = i
            raf.append((i0, i1))
        raf = [(tv[i], tv[j]) for i, j in raf
               if np.ptp(Pi[i:j + 1, 0]) + np.ptp(Pi[i:j + 1, 1]) > 40 and tv[j] - tv[i] > 1.0]
        print("    ráfagas de movimiento en el video: %s" % ", ".join("%.1f–%.1f" % r_ for r_ in raf))
        if len(raf) != len(datos):
            print("    AVISO: %d ráfagas para %d corridas" % (len(raf), len(datos)))
        sincs = []
        for s_ in range(len(datos)):
            t_, P_, info_ = datos[s_]
            ti = info_.get("_t_ini_plan", t_[0])
            if s_ >= len(raf):
                sincs.append((-1.0, 0.0)); continue
            base = raf[s_][0] - ti
            cand = [(rdes[s_](dd), dd) for dd in np.arange(base - 1.0, base + 1.0, 0.005)]
            sincs.append(max(cand))
        mejor_total = None
    elif a.sinc_libre:
        # sincronización sin las horas de las carpetas: cada corrida se busca en orden en el video,
        # después del final de la anterior (sirve cuando el reloj de guardado no es regular)
        sincs, desde = [], tv[0] - datos[0][0][0]
        for s_ in range(len(datos)):
            t_ = datos[s_][0]
            cand = [(rdes[s_](dd), dd) for dd in np.arange(desde, tv[-1] - t_[0], 0.02)]
            if not cand:
                sincs.append((-1.0, desde)); continue
            r_, d_ = max(cand)
            print("    sincronización libre: corrida %d en %.2f s del video (r = %.2f)" % (s_ + 1, d_ + t_[0], r_))
            sincs.append((r_, d_))
            desde = d_ + (t_[-1] - t_[0]) * 0.9
        mejor_total = None
    # sincronización conjunta: el desfase de la corrida k es d0 + (hora_k - hora_0) + ajuste fino de ±1.5 s
    mejor_total = None
    for d0 in (np.arange(tv[0] - datos[0][0][-1], tv[-1], 0.05) if not (a.sinc_libre or a.sinc_rafagas) else []):
        tot, ds = 0.0, []
        for s in range(len(datos)):
            cand = [(rdes[s](d0 + T[s] + dd), d0 + T[s] + dd) for dd in np.arange(-1.5, 1.51, 0.1)]
            r_, d_ = max(cand)
            tot += r_
            ds.append(d_)
        if mejor_total is None or tot > mejor_total[0]:
            mejor_total = (tot, ds)
    if not (a.sinc_libre or a.sinc_rafagas):
        sincs = []
    for s, d_ in enumerate(mejor_total[1] if mejor_total else []):
        cand = [(rdes[s](d_ + dd), d_ + dd) for dd in np.arange(-0.15, 0.151, 0.005)]
        sincs.append(max(cand))
    filas = []
    graf = []
    for s, carpeta in enumerate(a.corridas):
        t, P, info = datos[s]
        r_sinc, d = sincs[s]
        if r_sinc < 0.5:
            print("  %s: no se encontró bien en el video (r = %.2f)" % (os.path.basename(carpeta), r_sinc))
            continue
        # tramo del video de esta corrida (tiempos de la corrida)
        w = (tv >= t[0] + d) & (tv <= t[-1] + d) & okv
        tw = tv[w] - d
        Pw = Pv[w]
        mov = np.where(np.hypot(*np.gradient(P, axis=0).T) > 1e-4)[0]
        t_ini, t_fin = t[mov[0]], t[mov[-1]]
        if "_t_fin_plan" in info:
            t_ini, t_fin = info["_t_ini_plan"], info["_t_fin_plan"]
            mov = np.where((t >= t_ini) & (t <= t_fin))[0]

        # alineación sobre el CONTORNO (sin tiempos, para que el retardo no la sesgue): giro + desplazamiento
        enmov = (tw >= t_ini) & (tw <= t_fin)
        poly = P[mov[0]:mov[-1] + 1]
        par, esp = vr.alinear(Pw, poly, enmov, tw, t, P, 0.0)
        Pr = vr.mover(par, Pw, esp)

        # métricas
        e_cont = vr.dist_polilinea(Pr[enmov], poly)
        # tamaño: desplazamiento radial medio respecto al centro de la figura (+ = más grande que el comando)
        cen = 0.5 * (poly.min(0) + poly.max(0))
        rr = np.hypot(*(Pr[enmov] - cen).T)
        idx = [np.argmin(np.hypot(*(poly - p_).T)) for p_ in Pr[enmov]]
        rc = np.hypot(*(poly[idx] - cen).T)
        off = float(np.median(rr - rc))
        tam = float(np.median(rr) / np.median(rc))
        lag = float("nan")
        esc = tam
        # parada final
        tras = tw > t_fin
        fin_cmd = P[-1]
        reposo = tras & (tw > tw[-1] - 0.4)
        fin_real = np.median(Pr[reposo], axis=0) if reposo.sum() > 5 else Pr[-1]
        dev = np.hypot(*(Pr[tras] - fin_real).T)
        ts1, a1 = asentamiento(tw[tras] - t_fin, dev, 1.0)
        ts05, a05 = asentamiento(tw[tras] - t_fin, dev, 0.5)
        fila = {"corrida": os.path.basename(carpeta), "vmax": info.get("vmax_mm_s"), "amax": info.get("amax_mm_s2"),
                "r_sinc": float(r_sinc), "desplazamiento_radial_mm": off,
                "contorno_rms_mm": float(np.sqrt(np.mean(e_cont ** 2))), "contorno_max_mm": float(e_cont.max()),
                                "escala_aparente": esc,
                "parada_desv_max_mm": float(dev.max()) if len(dev) else float("nan"),
                "asent_1mm_s": ts1, "asent_05mm_s": ts05,
                "grabado_tras_parada_s": float(tw[-1] - t_fin),
                "error_final_mm": float(np.hypot(*(fin_real - fin_cmd))),
                "marcador_visible_pct": 100.0 * float(np.mean(okv[(tv >= t[0] + d) & (tv <= t[-1] + d)]))}
        filas.append(fila)
        np.savetxt(os.path.join(a.salida, "real_%s.csv" % os.path.basename(carpeta.rstrip("/"))),
                   np.column_stack([tw, Pr]), delimiter=",", fmt="%.5f", header="t_s,x_mm,y_mm", comments="")
        graf.append((fila, P, Pr, tw, t_fin, fin_real, poly))
        print("  %-36s r=%.2f radial %+5.2f mm | contorno RMS %.2f máx %.2f mm | escala %.3f | parada máx %.2f mm, "
              "±1 mm a %.2f s, ±0.5 mm a %.2f s | final %.2f mm"
              % (fila["corrida"], r_sinc, off, fila["contorno_rms_mm"], fila["contorno_max_mm"], esc,
                 fila["parada_desv_max_mm"], ts1, ts05, fila["error_final_mm"]))

    os.makedirs(a.salida, exist_ok=True)
    with open(os.path.join(a.salida, "figuras_%s.json" % a.etiqueta), "w", encoding="utf-8") as f:
        json.dump(filas, f, indent=2, ensure_ascii=False)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(graf)
    fig, axs = plt.subplots(2, n, figsize=(4.6 * n, 8.5), squeeze=False)
    for i, (fila, P, Pr, tw, t_fin, fin_real, poly) in enumerate(graf):
        ax = axs[0, i]
        ax.plot(poly[:, 0], poly[:, 1], "--", color="#8794A1", lw=1, label="comando")
        ax.plot(Pr[:, 0], Pr[:, 1], color="#D9480F", lw=0.8, label="real (video)")
        ax.set_aspect("equal")
        ax.set_title("%s/%s · contorno RMS %.2f mm\nmáx %.2f mm · escala %.3f"
                     % (fila["vmax"], fila["amax"], fila["contorno_rms_mm"], fila["contorno_max_mm"],
                        fila["escala_aparente"]), fontsize=9)
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(fontsize=8)
        ax = axs[1, i]
        tras = tw > t_fin
        ax.plot(tw[tras] - t_fin, Pr[tras, 0] - fin_real[0], color="#D9480F", lw=0.9, label="x")
        ax.plot(tw[tras] - t_fin, Pr[tras, 1] - fin_real[1], color="#7048E8", lw=0.9, label="y")
        for s_ in (1, -1, 0.5, -0.5):
            ax.axhline(s_, color="k", lw=0.4, ls="--" if abs(s_) == 1 else ":")
        ax.set_xlabel("tiempo desde el fin del comando [s]")
        ax.set_ylabel("desviación del reposo final [mm]")
        ax.set_title("parada: máx %.2f mm · ±1 mm a %.2f s" % (fila["parada_desv_max_mm"], fila["asent_1mm_s"]),
                     fontsize=9)
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(a.salida, "figuras_%s.png" % a.etiqueta), dpi=100)
    print("Resultados en %s" % a.salida)


if __name__ == "__main__":
    main()
