"""
Compara la posición REAL del efector medida con video con la trayectoria comandada y con el modelo.

Pasos:
 1. Simula la corrida con el modelo identificado (modelo5b) → posición del efector P_mod(t).
 2. Sincroniza el video con la corrida correlacionando la rapidez del efector (video) con la del modelo.
 3. Ajusta una homografía plano (mm) → imagen (px) con todos los cuadros de la corrida: corrige la
    perspectiva de la cámara (8 parámetros globales; no puede absorber vibraciones ni retardos).
 4. Métricas en mm: error de contorno (distancia a la trayectoria ideal), error frente al comando en
    el tiempo, error del modelo, y vibración después de la última parada.

Uso:  python video_comparar.py <video_marcador.csv> <carpeta de la corrida> [--modelo modelo_identificado.json]
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402


def efector(q1, q2):
    P = np.array([m.fk(a, b)[2] for a, b in zip(q1, q2)])
    return P * 1000.0                                             # mm


def dist_polilinea(p, poly):
    """Distancia de cada punto p (n×2) a la polilínea poly (k×2)."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(p))
    for i, x in enumerate(p):
        t = np.clip(((x - a) * ab).sum(1) / L2, 0, 1)
        proj = a + t[:, None] * ab
        out[i] = np.sqrt(((proj - x) ** 2).sum(1)).min()
    return out


def analizar(video_csv, carpeta, params, salida, etiqueta):
    v = np.genfromtxt(video_csv, delimiter=",", names=True)
    tv, px = v["t_s"], np.column_stack([v["cx_px"], v["cy_px"]])
    c = m.preparar(m.leer_corrida(carpeta))
    t = c["t"]
    q, w, _ = m.simular(c, params)
    P_mod = efector(q[:, 0], q[:, 1])
    P_cmd = efector(c["q1c"], c["q2c"])

    # --- 1. sincronización: rapidez del efector (video, px/s → mm/s aprox.) frente a la del modelo
    esc = np.median(v["lado_px"]) / 20.0
    vv = np.hypot(np.gradient(px[:, 0], tv), np.gradient(px[:, 1], tv)) / esc
    vm = np.hypot(np.gradient(P_mod[:, 0], t), np.gradient(P_mod[:, 1], t))
    rejilla = np.arange(tv[0], tv[-1], 0.005)
    vvg = np.interp(rejilla, tv, np.convolve(vv, np.ones(5) / 5, mode="same"))
    mejor = None
    for d in np.arange(tv[0] - t[0] - 2, tv[-1] - t[-1] + 2, 0.005):
        tm = t + d
        sel = (tm > tv[0]) & (tm < tv[-1])
        if sel.sum() < 0.8 * len(t):
            continue
        a = np.interp(tm[sel], rejilla, vvg)
        b = vm[sel]
        r = np.corrcoef(a, b)[0, 1]
        if mejor is None or r > mejor[1]:
            mejor = (d, r)
    desfase, rcorr = mejor

    # --- 2. homografía mm → px con los cuadros que caen dentro de la corrida
    tm = t + desfase
    dentro = (tv >= tm[0]) & (tv <= tm[-1])
    Pm_v = np.column_stack([np.interp(tv[dentro], tm, P_mod[:, k]) for k in range(2)])
    H, _ = cv2.findHomography(Pm_v.astype(np.float64), px[dentro].astype(np.float64), 0)
    Hi = np.linalg.inv(H)
    pv = cv2.perspectiveTransform(px[dentro].reshape(-1, 1, 2).astype(np.float64), Hi).reshape(-1, 2)
    tt = tv[dentro] - desfase                                       # tiempo de la corrida
    Pc_v = np.column_stack([np.interp(tt, t, P_cmd[:, k]) for k in range(2)])

    # --- 3. métricas
    i0 = np.where(np.hypot(*np.gradient(P_cmd, axis=0).T) > 1e-3)[0]
    t_ini, t_fin = t[i0[0]], t[i0[-1]]                              # comando en movimiento
    mov = (tt >= t_ini) & (tt <= t_fin)
    e_cont = dist_polilinea(pv[mov], P_cmd[i0[0]:i0[-1] + 1])
    e_cmd = np.hypot(*(pv - Pc_v).T)
    e_mod = np.hypot(*(pv - Pm_v).T)
    # vibración al parar: con TODO el video posterior, respecto a la posición final de reposo del propio video
    pall = cv2.perspectiveTransform(px.reshape(-1, 1, 2).astype(np.float64), Hi).reshape(-1, 2)
    tall = tv - desfase
    tras_v = tall > t_fin
    reposo = tall > tall[-1] - 0.5
    final_real = np.median(pall[reposo], axis=0)
    desv = np.hypot(*(pall[tras_v] - final_real).T)
    fuera = np.where(desv > 0.3)[0]
    asent = (tall[tras_v][fuera[-1]] - t_fin) if len(fuera) else 0.0
    grabado_despues = tall[-1] - t_fin
    final = P_cmd[-1]
    tras = tt > t_fin
    rep = {
        "corrida": os.path.basename(carpeta), "video": os.path.basename(video_csv),
        "vmax": c["info"].get("vmax_mm_s"), "amax": c["info"].get("amax_mm_s2"),
        "desfase_s": float(desfase), "correlacion_sinc": float(rcorr),
        "error_contorno_rms_mm": float(np.sqrt(np.mean(e_cont ** 2))), "error_contorno_max_mm": float(e_cont.max()),
        "error_vs_comando_rms_mm": float(np.sqrt(np.mean(e_cmd[mov] ** 2))), "error_vs_comando_max_mm": float(e_cmd[mov].max()),
        "error_modelo_rms_mm": float(np.sqrt(np.mean(e_mod[mov] ** 2))),
        "error_cinematica_rigida_rms_mm": float(np.sqrt(np.mean(e_cmd[mov] ** 2))),
        "vibracion_parada_max_mm": float(desv.max()) if len(desv) else float("nan"),
        "asentamiento_0p3mm_s": float(asent), "grabado_tras_parada_s": float(grabado_despues),
        "asentado": bool(len(fuera) == 0 or tall[tras_v][fuera[-1]] < tall[-1] - 0.5),
        "ruido_reposo_mm": float(np.std(np.hypot(*(pall[reposo] - final_real).T))),
        "desfase_estatico_final_mm": float(np.hypot(*(final_real - final))),
    }
    dibujar(salida, etiqueta, t, P_cmd, P_mod, tt, pv, Pm_v, Pc_v, mov, tras, t_fin, final, rep,
            (tall[tras_v] - t_fin, pall[tras_v] - final_real))
    return rep


def dibujar(salida, etiqueta, t, P_cmd, P_mod, tt, pv, Pm_v, Pc_v, mov, tras, t_fin, final, rep, vib):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.5])
    a0 = fig.add_subplot(gs[:, 0])
    a0.plot(P_cmd[:, 0], P_cmd[:, 1], "--", color="#8794A1", lw=1.2, label="comando (pulsos)")
    a0.plot(P_mod[:, 0], P_mod[:, 1], color="#0E7C86", lw=1.0, label="modelo dinámico")
    a0.plot(pv[:, 0], pv[:, 1], color="#D9480F", lw=1.0, label="real (video)")
    a0.set_aspect("equal")
    a0.set_xlabel("x [mm]")
    a0.set_ylabel("y [mm]")
    a0.legend(loc="upper right")
    a0.set_title("%s · %s/%s" % (etiqueta, rep["vmax"], rep["amax"]))
    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(tt, np.hypot(*(pv - Pc_v).T), color="#8794A1", lw=1, label="real − comando")
    a1.plot(tt, np.hypot(*(pv - Pm_v).T), color="#0E7C86", lw=1, label="real − modelo")
    a1.axvline(t_fin, color="k", lw=0.6)
    a1.set_ylabel("distancia [mm]")
    a1.set_title("Error de posición del efector (contorno RMS %.2f mm, máx %.2f mm)"
                 % (rep["error_contorno_rms_mm"], rep["error_contorno_max_mm"]))
    a1.legend(loc="upper right")
    a2 = fig.add_subplot(gs[1, 1])
    a2.plot(vib[0], vib[1][:, 0], color="#D9480F", lw=1, label="x real")
    a2.plot(vib[0], vib[1][:, 1], color="#7048E8", lw=1, label="y real")
    a2.plot(tt[tras] - t_fin, Pm_v[tras, 0] - final[0], color="#D9480F", lw=0.8, ls=":", label="x modelo")
    a2.plot(tt[tras] - t_fin, Pm_v[tras, 1] - final[1], color="#7048E8", lw=0.8, ls=":", label="y modelo")
    for s in (0.3, -0.3):
        a2.axhline(s, color="k", lw=0.5, ls="--")
    a2.set_xlabel("tiempo desde la última parada del comando [s]")
    a2.set_ylabel("desviación respecto al reposo final [mm]")
    a2.set_title("Vibración al parar: máx %.2f mm · dentro de ±0.3 mm a los %.2f s"
                 % (rep["vibracion_parada_max_mm"], rep["asentamiento_0p3mm_s"]))
    a2.legend(loc="upper right", ncol=2)
    for a in (a0, a1, a2):
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "video_%s.png" % etiqueta), dpi=110)
    plt.close("all")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("marcador_csv")
    ap.add_argument("corrida")
    ap.add_argument("--modelo", default=None)
    ap.add_argument("--etiqueta", default="corrida")
    ap.add_argument("--salida", default=".")
    a = ap.parse_args()
    ruta = a.modelo or os.path.join(os.path.expanduser("~"), "Downloads", "Resultados_BNO055", "etapa2",
                                    "modelo_identificado.json")
    with open(ruta, encoding="utf-8") as f:
        params = json.load(f)["parametros"]
    rep = analizar(a.marcador_csv, a.corrida, params, a.salida, a.etiqueta)
    for k, v in rep.items():
        print("  %-32s %s" % (k, ("%.3f" % v) if isinstance(v, float) else v))


if __name__ == "__main__":
    main()
