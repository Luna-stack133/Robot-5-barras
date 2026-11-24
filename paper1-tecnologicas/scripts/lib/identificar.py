"""
Etapa 2 — Identificación del modelo dinámico con las corridas de la etapa 1.

Ajusta los parámetros del servo (rigidez K, amortiguamiento B, feedforward ff, retardo tau)
para que el gz simulado coincida con el medido. Usa una corrida de cada par de cuadrados para
ajustar y la otra para validar; las secuencias de calibración quedan también para validar.

Uso (desde la carpeta del script):
    python identificar.py                       usa Descargas/Resultados_BNO055/etapa1
    python identificar.py --raiz "ruta\\etapa1"
    python identificar.py --comun               el mismo K y B para los dos motores (por defecto, uno por motor)
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402


def raiz_defecto():
    return os.path.join(os.path.expanduser("~"), "Downloads", "Resultados_BNO055", "etapa1")


def corridas_con_figura(raiz):
    out = []
    for c in sorted(glob.glob(os.path.join(raiz, "*"))):
        if not os.path.isfile(os.path.join(c, "imu.csv")):
            continue
        nombre = os.path.basename(c)
        if "Cuadrado" in nombre or "Secuencia" in nombre or "rculo" in nombre or "Pick" in nombre:
            out.append(c)
    return out


def unpack(x, libres, base):
    p = dict(base)
    for k, v in zip(libres, x):
        p[k] = float(v)
    return p


def residuos(x, libres, base, datos, sel):
    p = unpack(x, libres, base)
    r = []
    for c in datos:
        _, _, w3 = m.simular(c, p)
        e = (c["gz"] / m.K_GIRO - w3)[sel(c)]
        r.append(e / math.sqrt(len(e)))          # cada corrida pesa lo mismo
    return np.concatenate(r)


def ajustar(datos, libres, base, x0, escala, sel):
    """Levenberg-Marquardt en variables escaladas."""
    x = np.array(x0, dtype=float)
    f = lambda xx: residuos(xx, libres, base, datos, sel)
    r = f(x)
    costo = float(r @ r)
    lam = 1e-2
    for it in range(40):
        J = np.empty((len(r), len(x)))
        for j in range(len(x)):
            d = np.zeros_like(x)
            d[j] = 1e-3 * escala[j]
            J[:, j] = (f(x + d) - f(x - d)) / (2 * d[j])
        A = J.T @ J
        g = J.T @ r
        mejora = False
        for _ in range(12):
            dx = -np.linalg.solve(A + lam * np.diag(np.diag(A)), g)
            xn = x + dx
            if any(v <= 0 for k, v in zip(libres, xn) if k != "tau") or not (0 <= unpack(xn, libres, base)["ff"] <= 1.5):
                lam *= 4
                continue
            rn = f(xn)
            cn = float(rn @ rn)
            if cn < costo:
                mejora = (costo - cn) > 1e-6 * costo
                x, r, costo, lam = xn, rn, cn, max(lam / 3, 1e-6)
                break
            lam *= 4
        if not mejora:
            break
    cov = np.linalg.pinv(J.T @ J) * costo / max(1, len(r) - len(x))
    return x, cov


def metricas(c, p, tau_rig):
    gz = c["gz"] / m.K_GIRO
    _, _, w3 = m.simular(c, p)
    rig = m.rigido(c, tau_rig)
    e_mod = gz - w3
    e_rig = gz - rig
    fit = lambda e: 100 * (1 - np.linalg.norm(e) / np.linalg.norm(gz - gz.mean()))
    return {"rms_med": float(np.degrees(np.std(gz))),
            "rms_rig": float(np.degrees(np.sqrt(np.mean(e_rig ** 2)))),
            "rms_mod": float(np.degrees(np.sqrt(np.mean(e_mod ** 2)))),
            "fit_rig": fit(e_rig), "fit_mod": fit(e_mod)}, w3, rig


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--raiz")
    ap.add_argument("--comun", action="store_true", help="mismo K y B para los dos motores")
    ap.add_argument("--sin-graficas", action="store_true")
    a = ap.parse_args()
    raiz = os.path.abspath(a.raiz) if a.raiz else raiz_defecto()
    carpetas = corridas_con_figura(raiz)
    if not carpetas:
        sys.exit("No hay cuadrados ni secuencias en %s" % raiz)
    datos = [m.preparar(m.leer_corrida(c)) for c in carpetas]
    # pares de cuadrados con los mismos Vmax/Amax: el primero ajusta, el segundo valida
    grupos = {}
    for c in datos:
        if "Cuadrado" in c["nombre"]:
            i = c["info"]
            grupos.setdefault((i.get("vmax_mm_s"), round(i.get("amax_mm_s2", 0), -1)), []).append(c)
    ajuste = [g[0] for g in grupos.values() if len(g) >= 2]
    ids = {id(c) for c in ajuste}
    valid = [c for c in datos if id(c) not in ids]
    if not ajuste:
        ajuste, valid = datos, []
    sel = lambda c: slice(None)

    base = {"K": 2.75, "B": 2.75 / 20, "ff": 0.30, "tau": 0.0}
    if not a.comun:
        libres = ["K1", "K2", "B1", "B2", "ff", "tau"]
        x0 = [2.75, 2.75, 0.14, 0.14, 0.3, 0.0]
        esc = [1, 1, 0.1, 0.1, 0.1, 0.01]
    else:
        libres = ["K", "B", "ff", "tau"]
        x0 = [2.75, 0.14, 0.3, 0.0]
        esc = [1, 0.1, 0.1, 0.01]
    print("Ajustando con %d corridas (validación con %d)..." % (len(ajuste), len(valid)))
    x, cov = ajustar(ajuste, libres, base, x0, esc, sel)
    p = unpack(x, libres, base)
    err = np.sqrt(np.diag(cov))
    print("\nParámetros identificados:")
    nombres = {"K": "Rigidez del servo K [N·m/rad]", "K1": "Rigidez motor 1 [N·m/rad]", "K2": "Rigidez motor 2 [N·m/rad]",
               "B": "Amortiguamiento B [N·m·s/rad]", "B1": "Amortiguamiento motor 1", "B2": "Amortiguamiento motor 2",
               "ff": "Feedforward de velocidad ff [-]", "tau": "Retardo del comando tau [s]"}
    for k, v, e in zip(libres, x, err):
        print("  %-34s %9.4f  ± %.4f" % (nombres[k], v, e))
    KK = p.get("K1", p["K"]), p.get("K2", p["K"])
    BB = p.get("B1", p["B"]), p.get("B2", p["B"])
    print("  K/B (debería parecerse a Kp = 20 1/s):  %.1f / %.1f 1/s" % (KK[0] / BB[0], KK[1] / BB[1]))

    # retardo del modelo rígido (referencia): el mejor para cada corrida
    print("\n%-46s %8s | %-21s | %-21s" % ("Corrida", "gz RMS", "cinemática rígida", "modelo dinámico"))
    print("%-46s %8s | %9s %10s | %9s %10s" % ("", "°/s", "error °/s", "ajuste %", "error °/s", "ajuste %"))
    filas = []
    for grupo, lista in (("AJUSTE", ajuste), ("VALIDACIÓN", valid)):
        if not lista:
            continue
        print("-- %s" % grupo)
        for c in lista:
            mejor = min(np.arange(0, 0.15, 0.005), key=lambda tr: np.mean((c["gz"] / m.K_GIRO - m.rigido(c, tr)) ** 2))
            r, w3, rig = metricas(c, p, mejor)
            c["w3"], c["rig"] = w3, rig
            i = c["info"]
            etiqueta = "%s  %s/%s" % (c["nombre"][:30], int(i.get("vmax_mm_s", 0)), int(round(i.get("amax_mm_s2", 0))))
            print("%-46s %8.2f | %9.2f %9.1f%% | %9.2f %9.1f%%"
                  % (etiqueta, r["rms_med"], r["rms_rig"], r["fit_rig"], r["rms_mod"], r["fit_mod"]))
            filas.append((grupo, c["nombre"], r))
    salida = os.path.join(os.path.dirname(raiz), "etapa2")
    os.makedirs(salida, exist_ok=True)
    with open(os.path.join(salida, "modelo_identificado.json"), "w", encoding="utf-8") as f:
        json.dump({"parametros": p, "incertidumbre": dict(zip(libres, map(float, err))),
                   "masas_kg": m.MASAS, "geom_m": m.GEOM, "k_giro": m.K_GIRO,
                   "corridas": [{"grupo": g, "nombre": n, **r} for g, n, r in filas]}, f, indent=2, ensure_ascii=False)
    if not a.sin_graficas:
        dibujar(salida, ajuste + valid)
    print("\nResultados en %s" % salida)


def dibujar(salida, datos):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for c in datos:
        t = c["t"]
        gz = np.degrees(c["gz"] / m.K_GIRO)
        fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
        axs[0].plot(t, gz, color="#222", lw=0.9, label="medido (IMU)")
        axs[0].plot(t, np.degrees(c["rig"]), color="#8794A1", lw=0.9, ls="--", label="cinemática rígida")
        axs[0].plot(t, np.degrees(c["w3"]), color="#0E7C86", lw=1.1, label="modelo dinámico")
        axs[0].set_ylabel("giro del distal derecho [°/s]")
        axs[0].legend(loc="upper right")
        i = c["info"]
        axs[0].set_title("%s · Vmax %s mm/s · Amax %s mm/s²" % (c["nombre"], i.get("vmax_mm_s"), i.get("amax_mm_s2")))
        axs[1].plot(t, gz - np.degrees(c["rig"]), color="#8794A1", lw=0.8, label="error cinemática rígida")
        axs[1].plot(t, gz - np.degrees(c["w3"]), color="#0E7C86", lw=0.9, label="error modelo dinámico")
        axs[1].axhline(0, color="k", lw=0.5)
        axs[1].set_ylabel("medido − predicho [°/s]")
        axs[1].set_xlabel("tiempo [s]")
        axs[1].legend(loc="upper right")
        for ax in axs:
            ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(salida, "modelo_%s.png" % c["nombre"]), dpi=110)
        plt.close("all")


if __name__ == "__main__":
    main()
