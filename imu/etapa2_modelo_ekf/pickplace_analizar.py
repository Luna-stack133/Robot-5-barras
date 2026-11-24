"""
Bloque 2 — Pick and place: asentamiento y repetibilidad en cada llegada, con video y con la IMU.

Para cada llegada (fin de cada movimiento del comando, seguido de la pausa):
  IMU (giro del distal derecho, gz):
    - pico de |gz| después de parar y tiempo hasta quedar por debajo de 2 °/s.
  Video (posición del efector en mm, escala fijada por el marcador de 20 mm):
    - desviación máxima respecto al reposo final de esa pausa (sobrepaso + vibración);
    - tiempo hasta quedar dentro de ±1 mm y ±0.5 mm;
    - posición final de la pausa → repetibilidad en el punto de tomar y en el de soltar
      (ISO 9283: RP = media de la distancia al baricentro + 3·desviación estándar).

Uso:  python pickplace_analizar.py <corrida_pick_and_place> [--video <marcador.csv>] [--salida carpeta]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402
import video_real as vr  # noqa: E402

GZ_UMBRAL = 2.0        # °/s


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
        # firmware 1.3: los pulsos emitidos llevan la corrección aplicada; el plan es pulsos − Δq
        # (la Δq registrada en una muestra se emite en la siguiente: desfase de 1 muestra)
        apl = d["ctl_aplicando"].astype(float)
        q1d = q1d - np.roll(apl * d["dq1_ley_deg"], 1)
        q2d = q2d - np.roll(apl * d["dq2_ley_deg"], 1)
    q1, q2 = np.radians(q1d), np.radians(q2d)
    P = np.array([m.fk(a, b)[2] for a, b in zip(q1, q2)]) * 1000.0
    return t, P, d["gz"].astype(float) / m.K_GIRO, info          # °/s


def llegadas(t, P):
    """(t_fin_movimiento, t_fin_pausa, punto comandado) de cada llegada."""
    # en movimiento si la rapidez del plan (suavizada 0.1 s) supera 15 mm/s: tolera el ruido de ±0.1 mm de
    # la telemetría cuando la corrección con IMU está activa
    v = np.hypot(*np.gradient(P, t, axis=0).T)
    v = np.convolve(v, np.ones(11) / 11, mode="same")
    mov = v > 15.0
    out = []
    i = 1
    while i < len(t):
        if mov[i - 1] and not mov[i]:
            j = i
            while j < len(t) and not mov[j]:
                j += 1
            if t[min(j, len(t) - 1)] - t[i] > 0.8:
                out.append((t[i], t[min(j, len(t) - 1)], np.median(P[i:j], axis=0)))
            i = j
        else:
            i += 1
    return out


def asentar(tt, dev, tol):
    fuera = np.where(dev > tol)[0]
    if len(fuera) == 0:
        return 0.0
    if fuera[-1] >= len(tt) - 2:
        return float("nan")
    return float(tt[fuera[-1] + 1] - tt[0])


def rp_iso(X):
    c = X.mean(0)
    li = np.hypot(*(X - c).T)
    return float(li.mean() + 3 * li.std(ddof=1)) if len(X) > 2 else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corrida")
    ap.add_argument("--video")
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    salida = a.salida or a.corrida
    t, P, gz, info = leer(a.corrida)
    L = llegadas(t, P)
    print("%s: %d llegadas (pausa %.1f s, %s/%s)" % (os.path.basename(a.corrida), len(L), info.get("pausa_s", 0),
                                                     info.get("vmax_mm_s"), info.get("amax_mm_s2")))
    filas = []
    for k, (t0, t1, pc) in enumerate(L):
        w = (t >= t0) & (t <= t1)
        g = np.abs(gz[w])
        filas.append({"n": k + 1, "punto": "tomar" if k % 2 == 0 else "soltar", "t_llegada_s": float(t0),
                      "x_cmd": float(pc[0]), "y_cmd": float(pc[1]),
                      "gz_pico": float(g.max()), "gz_asent_s": asentar(t[w], g, GZ_UMBRAL)})

    # ---------------- video
    Pr = tr = None
    if a.video:
        v = np.genfromtxt(a.video, delimiter=",", names=True)
        _, Pv, rms_cuadro = vr.rectificar(v)
        tv = v["t_s"]
        okv = np.all(np.isfinite(Pv), axis=1)
        Pi = np.column_stack([np.interp(tv, tv[okv], Pv[okv, k]) for k in range(2)])
        rap = np.convolve(np.hypot(np.gradient(Pi[:, 0], tv), np.gradient(Pi[:, 1], tv)), np.ones(5) / 5, "same")
        rej = np.arange(tv[0], tv[-1], 0.005)
        sv = np.interp(rej, tv, rap)
        sc = np.hypot(*np.gradient(P, t, axis=0).T)
        mejor = None
        for d in np.arange(tv[0] - t[-1], tv[-1], 0.01):
            mv = np.interp(t + d, rej, sv, left=np.nan, right=np.nan)
            ok = np.isfinite(mv)
            if ok.sum() < 0.6 * len(t):
                continue
            r = np.corrcoef(mv[ok], sc[ok])[0, 1]
            if mejor is None or r > mejor[0]:
                mejor = (r, d)
        r_sinc, d = mejor
        w = okv & (tv - d >= t[0]) & (tv - d <= t[-1])
        tr = tv[w] - d
        Pw = Pv[w]
        movc = np.interp(tr, t, (sc > 15.0).astype(float)) > 0.5
        par, esp = vr.alinear(Pw, P[sc > 15.0][::2], movc, tr, t, P, 0.0)
        Pr = vr.mover(par, Pw, esp)
        if a.salida:
            os.makedirs(a.salida, exist_ok=True)
            np.savetxt(os.path.join(a.salida, "real_%s.csv" % os.path.basename(a.corrida.rstrip("/"))),
                       np.column_stack([tr, Pr]), delimiter=",", fmt="%.5f", header="t_s,x_mm,y_mm", comments="")
        print("video: marcador visible %.0f %%, cuadro de 20 mm tras rectificar RMS %.2f mm, sincronización r = %.2f"
              % (100 * okv.mean(), rms_cuadro, r_sinc))
        for f_, (t0, t1, pc) in zip(filas, L):
            s = (tr >= t0) & (tr <= t1)
            if s.sum() < 30:
                continue
            reposo = s & (tr >= t1 - 0.4)
            fin = np.median(Pr[reposo], axis=0)
            dev = np.hypot(*(Pr[s] - fin).T)
            f_.update({"x_real": float(fin[0]), "y_real": float(fin[1]),
                       "dev_max_mm": float(dev.max()),
                       "asent_1mm_s": asentar(tr[s], dev, 1.0), "asent_05mm_s": asentar(tr[s], dev, 0.5),
                       "error_final_mm": float(np.hypot(*(fin - pc)))})

    # ---------------- resumen
    def med(clave, sel):
        x = np.array([f_[clave] for f_ in filas if f_["punto"] in sel and clave in f_], dtype=float)
        x = x[np.isfinite(x)]
        return (float(np.median(x)), float(x.min()), float(x.max()), len(x)) if len(x) else None

    print("\n%-28s %-24s %-24s" % ("", "tomar (0, 180)", "soltar (150, 250)"))
    claves = [("gz_pico", "pico |gz| [°/s]"), ("gz_asent_s", "|gz| < 2 °/s a [s]"),
              ("dev_max_mm", "desviación máx [mm]"), ("asent_1mm_s", "dentro de ±1 mm a [s]"),
              ("asent_05mm_s", "dentro de ±0.5 mm a [s]")]
    resumen = {}
    for c, nombre in claves:
        a_, b_ = med(c, ("tomar",)), med(c, ("soltar",))
        if a_ is None and b_ is None:
            continue
        fmt = lambda r: "—" if r is None else "%.2f (%.2f–%.2f)" % r[:3]
        print("%-28s %-24s %-24s" % (nombre, fmt(a_), fmt(b_)))
        resumen[c] = {"tomar": a_, "soltar": b_}
    if Pr is not None:
        for p in ("tomar", "soltar"):
            X = np.array([[f_["x_real"], f_["y_real"]] for f_ in filas if f_["punto"] == p and "x_real" in f_])
            if len(X) > 2:
                rp = rp_iso(X)
                print("repetibilidad %-6s (ISO 9283, n=%d): RP = %.2f mm | dispersión máx entre llegadas %.2f mm"
                      % (p, len(X), rp, max(np.hypot(*(X[i] - X[j])) for i in range(len(X)) for j in range(i + 1, len(X)))))
                resumen["RP_" + p] = rp
    os.makedirs(salida, exist_ok=True)
    with open(os.path.join(salida, "pickplace.json"), "w", encoding="utf-8") as f:
        json.dump({"llegadas": filas, "resumen": resumen}, f, indent=2, ensure_ascii=False)

    # ---------------- gráfica
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3 if Pr is not None else 1, figsize=(17 if Pr is not None else 7, 5.2), squeeze=False)
    ax = axs[0, 0]
    for (t0, t1, pc), f_ in zip(L, filas):
        w = (t >= t0) & (t <= t1)
        ax.plot(t[w] - t0, gz[w], lw=0.7, color="#0E7C86" if f_["punto"] == "tomar" else "#D9480F", alpha=0.7)
    ax.axhline(GZ_UMBRAL, color="k", lw=0.5, ls="--")
    ax.axhline(-GZ_UMBRAL, color="k", lw=0.5, ls="--")
    ax.set_xlabel("tiempo desde la llegada [s]")
    ax.set_ylabel("giro del distal derecho [°/s]")
    ax.set_title("IMU: las 20 llegadas (verde = tomar, naranja = soltar)")
    ax.grid(alpha=0.3)
    if Pr is not None:
        ax = axs[0, 1]
        for (t0, t1, pc), f_ in zip(L, filas):
            s = (tr >= t0) & (tr <= t1)
            if s.sum() < 30 or "x_real" not in f_:
                continue
            dev = np.hypot(*(Pr[s] - [f_["x_real"], f_["y_real"]]).T)
            ax.plot(tr[s] - t0, dev, lw=0.7, color="#0E7C86" if f_["punto"] == "tomar" else "#D9480F", alpha=0.7)
        for y_ in (1, 0.5):
            ax.axhline(y_, color="k", lw=0.5, ls="--")
        ax.set_yscale("log")
        ax.set_ylim(0.05, 20)
        ax.set_xlabel("tiempo desde la llegada [s]")
        ax.set_ylabel("distancia al reposo final [mm]")
        ax.set_title("Video: asentamiento en cada llegada")
        ax.grid(alpha=0.3, which="both")
        ax = axs[0, 2]
        for p, col in (("tomar", "#0E7C86"), ("soltar", "#D9480F")):
            X = np.array([[f_["x_real"], f_["y_real"]] for f_ in filas if f_["punto"] == p and "x_real" in f_])
            if len(X):
                c = X.mean(0)
                ax.plot(X[:, 0] - c[0], X[:, 1] - c[1], "o", color=col, ms=5, label="%s (RP %.2f mm)" % (p, resumen.get("RP_" + p, float("nan"))))
        ax.set_aspect("equal")
        ax.set_xlabel("x respecto a la media [mm]")
        ax.set_ylabel("y respecto a la media [mm]")
        ax.set_title("Repetibilidad del punto final")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "pickplace.png"), dpi=100)
    print("Resultados en %s" % salida)


if __name__ == "__main__":
    main()
