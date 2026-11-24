"""
Calibración cinemática del robot de 5 barras con la IMU (BNO055 en un eslabón distal).

Idea: en cada movimiento de un punto de reposo A a otro punto de reposo B se compara
    - el giro MEDIDO del eslabón:   Δφ_med  = ∫ (gz − sesgo) dt  entre A y B
    - el giro PREDICHO:             Δφ_pred = φ(B) − φ(A), con la cinemática y los parámetros del robot
Entre dos reposos la vibración y el retardo del servo no cuentan: solo la geometría.
Se buscan los parámetros que mejor explican todos los movimientos (mínimos cuadrados no lineales,
Levenberg-Marquardt) y se usan los valores de diseño como información previa:

    δ1, δ2      error del cero de cada motor (grados; diseño 0)
    L1..L4      longitudes de los eslabones (mm; diseño 120/180/180/120)
    k           escala del giroscopio (diseño 1)
    D           fijo (medido): sirve de escala, porque los ángulos no cambian si todo el robot se
                agranda en la misma proporción

Uso (desde la carpeta del script):
    python calibrar_cinematica.py                 usa todas las corridas de Descargas/Resultados_BNO055/etapa1
    python calibrar_cinematica.py --solo-pausa    solo las secuencias con pausa (corrida de calibración)
    python calibrar_cinematica.py --libres d1,d2,L3,k     elige qué parámetros se ajustan
Opciones: --raiz, --eslabon izq|der (por defecto der), --sin-graficas

Resultados en etapa1/calibracion/:
    calibracion_parametros.csv     cada parámetro: diseño, calibrado, incertidumbre, ¿identificable?
    calibracion_movimientos.csv    cada movimiento: giro medido, predicho antes y después
    geometria_calibrada.json       para usar en el modelo y en el EKF
    calibracion.png
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from etapa1_analizar import corridas, fk_ambas, leer_imu, leer_info, num, raiz_etapa1  # noqa: E402

PARAMS = ["d1_deg", "d2_deg", "L1", "L2", "L3", "L4", "k"]
NOMBRES = {"d1_deg": "Error del cero, motor 1", "d2_deg": "Error del cero, motor 2",
           "L1": "L1 proximal izquierdo", "L2": "L2 distal izquierdo", "L3": "L3 distal derecho",
           "L4": "L4 proximal derecho", "k": "Escala del giroscopio"}
UNIDADES = {"d1_deg": "°", "d2_deg": "°", "L1": "mm", "L2": "mm", "L3": "mm", "L4": "mm", "k": ""}
# información previa: qué tanto pueden apartarse del diseño (1 sigma)
PREVIA = {"d1_deg": 2.0, "d2_deg": 2.0, "L1": 2.0, "L2": 2.0, "L3": 2.0, "L4": 2.0, "k": 0.05}

REPOSO_MIN_S = 0.45       # un reposo debe durar al menos esto (al inicio de cada corrida se graban 0.5 s)
VENTANA_S = 0.4           # se promedia el ángulo en el final del reposo (ya asentado)
GZ_ASENTADO = 1.5         # °/s: desviación máxima de gz en la ventana para aceptarla
SIGMA_MED = 0.10          # °: incertidumbre de cada giro medido (se reajusta con los residuos)
DERIVA = 0.02             # °/s: sesgo que queda sin corregir; pesa más en los movimientos largos


# ============================================================================ datos
def reposos(t, q1, q2, gz, fs):
    """Intervalos con los motores quietos. Devuelve [(i_ini_ventana, i_fin), ...]."""
    quieto = np.r_[True, (np.diff(q1) == 0) & (np.diff(q2) == 0)]
    out = []
    i, n = 0, len(t)
    while i < n:
        if quieto[i]:
            j = i
            while j < n and quieto[j]:
                j += 1
            if (j - i) / fs >= REPOSO_MIN_S:
                w0 = max(i, j - int(VENTANA_S * fs))
                out.append((w0, j))
            i = j
        else:
            i += 1
    return out


def movimientos(carpeta, eslabon):
    info = leer_info(carpeta)
    if info.get("tipo") == "grabacion":
        return [], info
    d, _ = leer_imu(carpeta)
    if d.size < 100:
        return [], info
    t = d["t_s"]
    fs = 1.0 / np.median(np.diff(t))
    q1, q2 = d["q1_deg"].astype(float), d["q2_deg"].astype(float)
    gz = d["gz"].astype(float)
    rep = reposos(t, q1, q2, gz, fs)
    buenos = [(a, b) for a, b in rep if np.std(gz[a:b]) < GZ_ASENTADO]
    if len(buenos) < 2:
        return [], info
    sesgo = float(np.median(np.concatenate([gz[a:b] for a, b in buenos])))
    dt = np.diff(t, prepend=t[0] - 1.0 / fs)
    ang = np.cumsum((gz - sesgo) * dt)                       # ángulo integrado, grados
    out = []
    for (a0, b0), (a1, b1) in zip(buenos[:-1], buenos[1:]):
        out.append({
            "corrida": os.path.basename(carpeta),
            "q1A": float(np.median(q1[a0:b0])), "q2A": float(np.median(q2[a0:b0])),
            "q1B": float(np.median(q1[a1:b1])), "q2B": float(np.median(q2[a1:b1])),
            "med": float(np.mean(ang[a1:b1]) - np.mean(ang[a0:b0])),
            "dur_s": float(t[a1] - t[b0 - 1]),
            "pausa": bool(info.get("pausa_s")),
        })
    # movimientos sin giro de los motores no aportan
    out = [m for m in out if abs(m["q1B"] - m["q1A"]) + abs(m["q2B"] - m["q2A"]) > 0.5]
    for m in out:
        m["modo"] = int(info.get("modo_ensamblaje", 1))
        m["eslabon"] = eslabon
    return out, info


# ============================================================================ modelo
def geom(p, D):
    return (p["L1"] / 1000.0, p["L2"] / 1000.0, p["L3"] / 1000.0, p["L4"] / 1000.0, D / 1000.0)


def angulo(q1d, q2d, p, D, ref, eslabon):
    """Ángulo (grados) del eslabón distal, con los parámetros p. ref: posición aproximada del efector."""
    g = geom(p, D)
    q1 = math.radians(q1d + p["d1_deg"])
    q2 = math.radians(q2d + p["d2_deg"])
    sol = fk_ambas(q1, q2, g)
    if sol is None:
        return float("nan")
    P = min(sol, key=lambda s: (s[0] - ref[0]) ** 2 + (s[1] - ref[1]) ** 2)
    if eslabon == "izq":
        bx, by = g[0] * math.cos(q1), g[0] * math.sin(q1)
    else:
        bx, by = g[4] + g[3] * math.cos(q2), g[3] * math.sin(q2)
    return math.degrees(math.atan2(P[1] - by, P[0] - bx))


def referencia(q1d, q2d, p0, D, modo):
    """Posición nominal del efector. En los modos de trabajo el efector siempre queda por encima de la
    base (y > 0), así que de las dos soluciones de la cinemática directa se toma la de mayor y."""
    g = geom(p0, D)
    sol = fk_ambas(math.radians(q1d), math.radians(q2d), g)
    if sol is None:
        return None
    return max(sol, key=lambda s: s[1])


def predichos(movs, p, D, signo):
    out = np.empty(len(movs))
    for i, m in enumerate(movs):
        a = angulo(m["q1A"], m["q2A"], p, D, m["refA"], m["eslabon"])
        b = angulo(m["q1B"], m["q2B"], p, D, m["refB"], m["eslabon"])
        out[i] = signo * p["k"] * math.remainder(b - a, 360.0)
    return out


def sigmas(movs):
    return np.array([math.hypot(SIGMA_MED, DERIVA * m["dur_s"]) for m in movs])


def residuos(x, libres, p0, D, movs, med, signo, sigma):
    p = dict(p0)
    for k, v in zip(libres, x):
        p[k] = v
    r_med = (med - predichos(movs, p, D, signo)) / sigma
    r_pre = np.array([(p[k] - p0[k]) / PREVIA[k] for k in libres])
    return np.concatenate([r_med, r_pre])


def ajustar(movs, p0, D, signo, libres):
    """Levenberg-Marquardt con jacobiano numérico. Devuelve (p, covarianza)."""
    med = np.array([m["med"] for m in movs])
    sigma = sigmas(movs)
    x = np.array([p0[k] for k in libres], dtype=float)
    lam = 1e-2
    f = lambda xx: residuos(xx, libres, p0, D, movs, med, signo, sigma)
    r = f(x)
    costo = float(r @ r)
    pasos = np.array([1e-3 if k == "k" else 1e-2 for k in libres])
    for _ in range(100):
        J = np.empty((len(r), len(x)))
        for j in range(len(x)):
            dx = np.zeros_like(x)
            dx[j] = pasos[j]
            J[:, j] = (f(x + dx) - f(x - dx)) / (2 * pasos[j])
        A = J.T @ J
        g = J.T @ r
        mejora = False
        for _ in range(20):
            delta = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            r2 = f(x + delta)
            c2 = float(r2 @ r2)
            if np.isfinite(c2) and c2 < costo:
                x, r, lam = x + delta, r2, max(lam / 3, 1e-7)
                mejora = abs(costo - c2) > 1e-10 * max(1.0, costo)
                costo = c2
                break
            lam *= 4
        if not mejora:
            break
    J = np.empty((len(r), len(x)))
    for j in range(len(x)):
        dx = np.zeros_like(x)
        dx[j] = pasos[j]
        J[:, j] = (f(x + dx) - f(x - dx)) / (2 * pasos[j])
    cov = np.linalg.pinv(J.T @ J)
    p = dict(p0)
    for k, v in zip(libres, x):
        p[k] = float(v)
    return p, cov


# ============================================================================ main
def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description="Calibración cinemática con la IMU")
    ap.add_argument("--raiz", help="carpeta etapa1 (por defecto Descargas/Resultados_BNO055/etapa1)")
    ap.add_argument("--eslabon", choices=["izq", "der"], default="der")
    ap.add_argument("--solo-pausa", action="store_true", help="usar solo las secuencias con pausa")
    ap.add_argument("--libres", default=",".join(PARAMS),
                    help="parámetros a ajustar, separados por coma (de: %s)" % ",".join(PARAMS))
    ap.add_argument("--k", type=float, default=1.0,
                    help="escala del giroscopio conocida (p. ej. de la prueba de giro de 180°); si k no está "
                         "en --libres queda fija en este valor")
    ap.add_argument("--sin-graficas", action="store_true")
    a = ap.parse_args()
    libres = [s.strip() for s in a.libres.split(",") if s.strip()]
    for k in libres:
        if k not in PARAMS:
            sys.exit("Parámetro desconocido: %s (use %s)" % (k, ", ".join(PARAMS)))

    raiz = os.path.abspath(a.raiz) if a.raiz else raiz_etapa1()
    movs, geoms = [], []
    for c in corridas(raiz):
        ms, info = movimientos(c, a.eslabon)
        if a.solo_pausa:
            ms = [m for m in ms if m["pausa"]]
        if ms:
            geoms.append(info.get("geometria_mm"))
        movs += ms
    if len(movs) < 3:
        sys.exit("Hay muy pocos movimientos entre reposos (%d). Hagan la corrida de calibración: "
                 "Trayectorias -> Secuencia -> Puntos de calibración." % len(movs))
    gm = next((g for g in geoms if g), None) or {"L1": 120, "L2": 180, "L3": 180, "L4": 120, "D": 150}
    D = float(gm["D"])
    p0 = {"d1_deg": 0.0, "d2_deg": 0.0, "L1": float(gm["L1"]), "L2": float(gm["L2"]),
          "L3": float(gm["L3"]), "L4": float(gm["L4"]), "k": float(a.k)}
    for m in movs:
        m["refA"] = referencia(m["q1A"], m["q2A"], p0, D, m["modo"])
        m["refB"] = referencia(m["q1B"], m["q2B"], p0, D, m["modo"])
    movs = [m for m in movs if m["refA"] is not None and m["refB"] is not None]
    med = np.array([m["med"] for m in movs])
    pred0 = predichos(movs, p0, D, 1.0)
    signo = 1.0 if float(np.dot(med, pred0)) >= 0 else -1.0
    pred0 *= signo
    corridas_usadas = sorted({m["corrida"] for m in movs})
    print("\nMovimientos entre reposos: %d, de %d corridas | giros de %.1f° a %.1f°"
          % (len(movs), len(corridas_usadas), np.min(np.abs(med)), np.max(np.abs(med))))

    # ---- ajuste con todos los movimientos
    p, cov = ajustar(movs, p0, D, signo, libres)
    pred1 = predichos(movs, p, D, signo)
    res0, res1 = med - pred0, med - pred1
    chi2 = float(np.sum((res1 / sigmas(movs)) ** 2))
    escala = max(1.0, math.sqrt(chi2 / max(1, len(res1) - len(libres))))   # si el ruido real es mayor
    err = {k: float(math.sqrt(cov[i, i])) * escala for i, k in enumerate(libres)}

    # ---- validación: dejar fuera una corrida cada vez
    val = []
    if len(corridas_usadas) >= 2:
        for cr in corridas_usadas:
            ent = [m for m in movs if m["corrida"] != cr]
            prueba = [m for m in movs if m["corrida"] == cr]
            if len(ent) < 3 or not prueba:
                continue
            pv, _ = ajustar(ent, p0, D, signo, libres)
            mv = np.array([m["med"] for m in prueba])
            val.append((cr, len(prueba), float(np.sqrt(np.mean((mv - predichos(prueba, p0, D, signo)) ** 2))),
                        float(np.sqrt(np.mean((mv - predichos(prueba, pv, D, signo)) ** 2)))))

    # ---- salida
    rms0, rms1 = float(np.sqrt(np.mean(res0 ** 2))), float(np.sqrt(np.mean(res1 ** 2)))
    rel0 = float(np.dot(med, pred0) / np.dot(pred0, pred0))
    rel1 = float(np.dot(med, pred1) / np.dot(pred1, pred1))
    print("Error de los giros: RMS %.3f° con el diseño -> %.3f° calibrado | razón medido/predicho %.4f -> %.4f"
          % (rms0, rms1, rel0, rel1))
    print("\n%-28s %10s %10s %9s  %s" % ("Parámetro", "Diseño", "Calibrado", "± 1σ", "¿Identificable?"))
    filas = []
    for k in PARAMS:
        if k in libres:
            ident = err[k] < 0.5 * PREVIA[k]
            filas.append((k, p0[k], p[k], err[k], "sí" if ident else "no (lo fija la previa)"))
            print("%-28s %10.3f %10.3f %9.3f  %s" % (NOMBRES[k] + " [" + (UNIDADES[k] or "-") + "]", p0[k], p[k],
                                                     err[k], filas[-1][4]))
        else:
            filas.append((k, p0[k], p0[k], float("nan"), "fijo"))
    if val:
        print("\nValidación (ajustando sin la corrida y prediciéndola):")
        for cr, n, r0, r1 in val:
            print("  %-36s %2d mov | RMS %.3f° con el diseño -> %.3f° calibrado" % (cr, n, r0, r1))
    if len(libres) > 1:
        sd = np.sqrt(np.diag(cov))
        corr_m = cov / np.outer(sd, sd)
        pares = [(abs(corr_m[i, j]), libres[i], libres[j], corr_m[i, j])
                 for i in range(len(libres)) for j in range(i + 1, len(libres))]
        pares = [x for x in sorted(pares, reverse=True) if x[0] > 0.8]
        if pares:
            print("\nParámetros muy correlacionados (los datos no los separan bien):")
            for _, k1, k2, c in pares[:6]:
                print("  %s y %s: %.2f" % (k1, k2, c))

    salida = os.path.join(raiz, "calibracion")
    os.makedirs(salida, exist_ok=True)
    with open(os.path.join(salida, "calibracion_parametros.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["Parametro", "Clave", "Diseno", "Calibrado", "Incertidumbre_1sigma", "Unidad", "Identificable"])
        for k, v0, v1, e, idn in filas:
            w.writerow([NOMBRES[k], k, num(v0), num(v1), num(e), UNIDADES[k], idn])
        w.writerow([])
        w.writerow(["Movimientos", "", len(movs)])
        w.writerow(["RMS giros con el diseño", "", num(rms0), "", "", "°"])
        w.writerow(["RMS giros calibrado", "", num(rms1), "", "", "°"])
        w.writerow(["Razón medido/predicho con el diseño", "", num(rel0)])
        w.writerow(["Razón medido/predicho calibrado", "", num(rel1)])
    with open(os.path.join(salida, "calibracion_movimientos.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["corrida", "q1_A", "q2_A", "q1_B", "q2_B", "giro_medido_deg", "predicho_diseno_deg",
                    "predicho_calibrado_deg", "residuo_diseno_deg", "residuo_calibrado_deg"])
        for m, a0, a1 in zip(movs, pred0, pred1):
            w.writerow([m["corrida"], num(m["q1A"]), num(m["q2A"]), num(m["q1B"]), num(m["q2B"]), num(m["med"]),
                        num(a0), num(a1), num(m["med"] - a0), num(m["med"] - a1)])
    with open(os.path.join(salida, "geometria_calibrada.json"), "w", encoding="utf-8") as f:
        json.dump({"D_mm": D, "parametros": {k: p[k] for k in PARAMS}, "incertidumbre": err, "libres": libres,
                   "eslabon": a.eslabon, "signo_gz": signo, "movimientos": len(movs),
                   "rms_diseno_deg": rms0, "rms_calibrado_deg": rms1}, f, indent=2, ensure_ascii=False)
    if not a.sin_graficas:
        dibujar(salida, med, pred0, pred1, movs)
    print("\nResultados en %s" % salida)


def dibujar(salida, med, pred0, pred1, movs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.5))
    lim = 1.1 * max(np.max(np.abs(med)), 1.0)
    axs[0].plot([-lim, lim], [-lim, lim], color="k", lw=0.6)
    axs[0].scatter(pred0, med, s=18, color="#8794A1", label="con el diseño")
    axs[0].scatter(pred1, med, s=18, color="#0E7C86", label="calibrado")
    axs[0].set_xlabel("giro predicho por la cinemática [°]")
    axs[0].set_ylabel("giro medido por la IMU [°]")
    axs[0].legend()
    axs[0].set_title("Giro neto del eslabón entre reposos")
    i = np.arange(len(med))
    axs[1].bar(i - 0.2, med - pred0, width=0.4, color="#8794A1", label="con el diseño")
    axs[1].bar(i + 0.2, med - pred1, width=0.4, color="#0E7C86", label="calibrado")
    axs[1].axhline(0, color="k", lw=0.6)
    axs[1].set_xlabel("movimiento")
    axs[1].set_ylabel("medido − predicho [°]")
    axs[1].legend()
    axs[1].set_title("Error de cada movimiento: RMS %.3f° → %.3f°"
                     % (np.sqrt(np.mean((med - pred0) ** 2)), np.sqrt(np.mean((med - pred1) ** 2))))
    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "calibracion.png"), dpi=120)
    plt.close("all")


if __name__ == "__main__":
    main()
