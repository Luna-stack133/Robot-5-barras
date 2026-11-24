r"""Etapa 1 - Comparar lo que mide la BNO055 con lo que predice la cinemática.

R5B Lab guarda sola cada corrida del robot real con IMU en
    Descargas\Resultados_BNO055\etapa1\<fecha>_<hora>_<nombre>\   (imu.csv, telemetria.csv, info.json)

Uso:
    python etapa1_analizar.py --ultima                  analiza la corrida más reciente
    python etapa1_analizar.py --todas                   analiza todas las corridas que aún no tienen resultados
    python etapa1_analizar.py --todas --rehacer         vuelve a analizar todas y rehace los resúmenes
    python etapa1_analizar.py "ruta\a\la\carpeta"       analiza una corrida concreta
Opciones:
    --r_mm 33.5      distancia (mm) del eje del efector al centro del chip, a lo largo del eslabón
    --eslabon der    eslabón distal donde va la IMU (izq / der). En las figuras se detecta solo;
                     en las grabaciones de golpes (robot quieto) se usa 'der' si no se indica.

Las grabaciones de "Grabar IMU" (prueba de golpes, "tipo": "grabacion" en info.json) se analizan
aparte: frecuencia natural, amortiguamiento ζ y asentamiento de cada golpe, y la gravedad que se
cuela en ax/ay con el robot quieto. Resultados en golpes.png y resultados.csv, una fila por postura
en etapa1\resumen_golpes.csv y, con 2 posturas o más, la inclinación separada en parte fija
(IMU en su bolsillo) y parte que gira con el eslabón (plano del robot) en resumen_inclinacion.csv.

Qué calcula (todo con los ángulos de los motores que llegan junto a cada muestra de la IMU):
    - gz predicho: velocidad de giro del eslabón distal según la cinemática, y gz medido por la IMU.
    - retardo entre la IMU y el comando, ganancia y error RMS.
    - oscilación al detenerse: amplitud, frecuencia y tiempo de asentamiento.
    - aceleración en el plano: predicha (con r) y medida; estima el ángulo de montaje del chip.
    - rumbo de la fusión frente al ángulo del eslabón.
Resultados en la misma carpeta (resultados.csv para Excel, resultados.json, gráficas .png) y una
fila por corrida en etapa1\resumen_etapa1.csv.

Dependencias:  pip install numpy matplotlib
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

G0 = 9.80665
SIGNOS = {1: (1, -1), 2: (-1, 1), 3: (1, 1), 4: (-1, -1)}
RUIDO_GZ_REPOSO = 0.10      # °/s, desviación estándar medida en la etapa 0 (servos energizados)
VENTANA_ASENT_S = 1.5       # s: ventana para medir el asentamiento después de cada parada


# ============================================================================ carpetas
def raiz_etapa1():
    descargas = os.path.join(os.path.expanduser("~"), "Downloads")
    base = descargas if os.path.isdir(descargas) else os.getcwd()
    return os.path.join(base, "Resultados_BNO055", "etapa1")


def corridas(raiz):
    return sorted(d for d in glob.glob(os.path.join(raiz, "*")) if os.path.isfile(os.path.join(d, "imu.csv")))


def leer_info(carpeta):
    ruta = os.path.join(carpeta, "info.json")
    if os.path.isfile(ruta):
        with open(ruta, encoding="utf-8") as f:
            return json.load(f)
    return {}


ESCALA_GIRO = 1.0           # escala del giroscopio (calibrar_cinematica.py); se cambia con --k


def leer_imu(carpeta):
    """Lee imu.csv y repara las muestras dañadas por ruido en el I2C (se reemplazan por la anterior).
    Devuelve (datos, número de muestras reparadas)."""
    d = np.genfromtxt(os.path.join(carpeta, "imu.csv"), delimiter=",", names=True)
    d = np.atleast_1d(d)
    if ESCALA_GIRO != 1.0:
        for k in ("gx", "gy", "gz"):
            d[k] = d[k] / ESCALA_GIRO
    if d.size < 3:
        return d, 0
    amag = np.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
    malas = ~np.isfinite(amag) | (amag < 2.0) | (amag > 40.0)
    # registro de calibración leído como 0xFF: todo en 3, incluido el magnetómetro (apagado en modo IMU)
    malas |= (d["cal_sis"] == 3) & (d["cal_gir"] == 3) & (d["cal_acc"] == 3) & (d["cal_mag"] == 3)
    # pico aislado de una sola muestra en el giroscopio
    for k in ("gx", "gy", "gz"):
        x = d[k]
        salto = np.zeros(len(x), dtype=bool)
        salto[1:-1] = (np.abs(x[1:-1] - x[:-2]) > 100) & (np.abs(x[1:-1] - x[2:]) > 100) & \
                      (np.abs(x[2:] - x[:-2]) < 50)
        malas |= salto
    malas |= ~np.isfinite(d["gz"])
    n = int(malas.sum())
    if n:
        cols = [c for c in d.dtype.names if c not in ("t_s", "q1_deg", "q2_deg")]
        idx = np.where(malas)[0]
        for i in idx:
            j = i - 1
            while j >= 0 and malas[j]:
                j -= 1
            if j < 0:
                j = i + 1
                while j < len(d) and malas[j]:
                    j += 1
                if j >= len(d):
                    continue
            for c in cols:
                d[c][i] = d[c][j]
    return d, n


def primera_posicion(carpeta):
    """Posición del efector (m) en la primera fila de telemetria.csv, o None."""
    ruta = os.path.join(carpeta, "telemetria.csv")
    if not os.path.isfile(ruta):
        return None
    try:
        tel = np.atleast_1d(np.genfromtxt(ruta, delimiter=",", names=True))
        if tel.size and np.isfinite(tel["x_mm"][0]) and np.isfinite(tel["y_mm"][0]):
            return float(tel["x_mm"][0]) / 1000.0, float(tel["y_mm"][0]) / 1000.0
    except (ValueError, KeyError, IndexError):
        pass
    return None


# ============================================================================ cinemática
def fk_ambas(q1, q2, g):
    """Las dos soluciones del efector para unos ángulos (rad). Devuelve (P0, P1) o None."""
    L1, L2, L3, L4, D = g
    ax, ay = L1 * math.cos(q1), L1 * math.sin(q1)
    bx, by = D + L4 * math.cos(q2), L4 * math.sin(q2)
    ex, ey = bx - ax, by - ay
    d = math.hypot(ex, ey)
    if d > L2 + L3 + 1e-9 or d < abs(L2 - L3) - 1e-9 or d < 1e-9:
        return None
    a = (L2 * L2 - L3 * L3 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, L2 * L2 - a * a))
    mx, my = ax + a * ex / d, ay + a * ey / d
    px, py = -ey / d, ex / d
    return (mx + h * px, my + h * py), (mx - h * px, my - h * py)


def ik(x, y, modo, g):
    L1, L2, L3, L4, D = g
    s1, s2 = SIGNOS[modo]
    r1, r2 = math.hypot(x, y), math.hypot(x - D, y)
    if r1 < 1e-9 or r2 < 1e-9:
        return None
    c1 = max(-1.0, min(1.0, (r1 * r1 + L1 * L1 - L2 * L2) / (2 * r1 * L1)))
    c2 = max(-1.0, min(1.0, (r2 * r2 + L4 * L4 - L3 * L3) / (2 * r2 * L4)))
    return math.atan2(y, x) + s1 * math.acos(c1), math.atan2(y, x - D) + s2 * math.acos(c2)


def efector(q1, q2, modo, g, p0=None):
    """Posición del efector (m) siguiendo la rama física por continuidad.
    p0: posición inicial conocida (de la telemetría); decide la rama de la primera muestra."""
    n = len(q1)
    P = np.full((n, 2), np.nan)
    ref = None
    for i in range(n):
        sol = fk_ambas(q1[i], q2[i], g)
        if sol is None:
            continue
        if ref is None and p0 is not None:
            ref = min(sol, key=lambda p: (p[0] - p0[0]) ** 2 + (p[1] - p0[1]) ** 2)
        elif ref is None:
            # primera muestra: la rama cuya IK en el modo activo reproduce los ángulos
            def error(p):
                q = ik(p[0], p[1], modo, g)
                if q is None:
                    return float("inf")
                return abs(math.remainder(q[0] - q1[i], 2 * math.pi)) + abs(math.remainder(q[1] - q2[i], 2 * math.pi))
            errores = [error(p) for p in sol]
            k = int(np.argmin(errores))
            ref = sol[k] if errores[k] < 1e-2 else max(sol, key=lambda p: p[1])
        else:
            ref = min(sol, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
        P[i] = ref
    return P


# ============================================================================ señales
def derivada(x, t, k=2):
    """Derivada central sobre ±k muestras (con k=2 y 100 Hz: ventana de 40 ms)."""
    d = np.full_like(x, np.nan, dtype=float)
    d[k:-k] = (x[2 * k:] - x[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    return d


def suavizar(x, n=3):
    if n <= 1:
        return x
    k = np.ones(n) / n
    y = np.convolve(np.nan_to_num(x), k, mode="same")
    y[np.isnan(x)] = np.nan
    return y


def retardo(med, pred, fs, max_s=0.25):
    """Retardo (s) de 'med' respecto a 'pred' por correlación cruzada (positivo: la IMU va atrás)."""
    ok = ~(np.isnan(med) | np.isnan(pred))
    a = med[ok] - np.mean(med[ok])
    b = pred[ok] - np.mean(pred[ok])
    m = int(max_s * fs)
    lags = np.arange(-m, m + 1)
    c = np.array([np.dot(a[max(0, L):len(a) + min(0, L)], b[max(0, -L):len(b) - max(0, L)]) for L in lags])
    i = int(np.argmax(c))
    frac = 0.0
    if 0 < i < len(c) - 1:                       # refinamiento parabólico
        den = c[i - 1] - 2 * c[i] + c[i + 1]
        if den != 0:
            frac = 0.5 * (c[i - 1] - c[i + 1]) / den
    return (lags[i] + frac) / fs


def desplazar(x, t, dt):
    """x(t - dt) por interpolación (dt > 0 retrasa la señal)."""
    ok = ~np.isnan(x)
    return np.interp(t - dt, t[ok], x[ok], left=np.nan, right=np.nan)


def rms(x):
    x = x[~np.isnan(x)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")


def corr(a, b):
    ok = ~(np.isnan(a) | np.isnan(b))
    if ok.sum() < 10 or np.std(a[ok]) == 0 or np.std(b[ok]) == 0:
        return float("nan")
    return float(np.corrcoef(a[ok], b[ok])[0, 1])


def paradas(w_pred, t, mov=10.0, quieto=2.0, minimo=0.25):
    """Instantes en que el eslabón se detiene: |ω| baja de 'quieto' °/s tras superar 'mov' °/s,
    y sigue quieto al menos 'minimo' segundos."""
    idx = []
    moviendo = False
    n = len(w_pred)
    for i in range(n):
        v = abs(w_pred[i]) if not math.isnan(w_pred[i]) else 0.0
        if v > mov:
            moviendo = True
        elif moviendo and v < quieto:
            j = i
            while j < n and t[j] - t[i] < minimo and (math.isnan(w_pred[j]) or abs(w_pred[j]) < quieto):
                j += 1
            if j >= n or t[j] - t[i] >= minimo:
                idx.append(i)
                moviendo = False
    return idx


def frecuencia_dominante(x, fs, fmin=2.0):
    x = x[~np.isnan(x)]
    if len(x) < 16:
        return float("nan")
    n = 1 << int(math.ceil(math.log2(len(x) * 8)))
    X = np.abs(np.fft.rfft((x - np.mean(x)) * np.hanning(len(x)), n))
    f = np.fft.rfftfreq(n, 1 / fs)
    m = f >= fmin
    return float(f[m][np.argmax(X[m])]) if np.any(m) else float("nan")


# ============================================================================ análisis
def analizar(carpeta, r_mm=None, eslabon=None, raiz=None, graficas=True):
    info = leer_info(carpeta)
    if info.get("tipo") == "grabacion":
        return analizar_golpes(carpeta, eslabon=eslabon, raiz=raiz, graficas=graficas)
    d, reparadas = leer_imu(carpeta)
    if d.size < 50:
        print("  %s: muy pocas muestras de IMU (%d)." % (os.path.basename(carpeta), d.size))
        return None
    gm = info.get("geometria_mm") or {"L1": 120, "L2": 180, "L3": 180, "L4": 120, "D": 150}
    g = tuple(gm[k] / 1000.0 for k in ("L1", "L2", "L3", "L4", "D"))
    modo = int(info.get("modo_ensamblaje", 1))
    r = (r_mm if r_mm is not None else float(info.get("r_mm", 0.0))) / 1000.0

    t = d["t_s"]
    fs = 1.0 / np.median(np.diff(t))
    huecos = int(np.sum(np.diff(t) > 1.5 / fs))
    q1, q2 = np.radians(d["q1_deg"]), np.radians(d["q2_deg"])
    P = efector(q1, q2, modo, g, primera_posicion(carpeta))
    C = np.column_stack([g[0] * np.cos(q1), g[0] * np.sin(q1)])
    Dd = np.column_stack([g[4] + g[3] * np.cos(q2), g[3] * np.sin(q2)])
    phi = {"izq": np.unwrap(np.arctan2(P[:, 1] - C[:, 1], P[:, 0] - C[:, 0])),
           "der": np.unwrap(np.arctan2(P[:, 1] - Dd[:, 1], P[:, 0] - Dd[:, 0]))}
    w_pred_todos = {k: np.degrees(derivada(v, t)) for k, v in phi.items()}
    gz = d["gz"].astype(float)

    # --- ¿en qué eslabón está la IMU?
    cors = {k: corr(gz, v) for k, v in w_pred_todos.items()}
    if eslabon is None:
        eslabon = max(cors, key=lambda k: abs(cors[k]) if not math.isnan(cors[k]) else -1)
    signo = 1.0 if (cors.get(eslabon, 1) or 1) >= 0 else -1.0
    w_pred = w_pred_todos[eslabon]
    fi = phi[eslabon]

    # --- retardo, ganancia y error
    mov = np.abs(np.nan_to_num(w_pred)) > 5.0
    lag = retardo(gz, w_pred, fs) if mov.sum() > 20 else float("nan")
    w_al = desplazar(w_pred, t, lag) if not math.isnan(lag) else w_pred
    ok = ~(np.isnan(w_al) | np.isnan(gz))
    ganancia = float(np.dot(gz[ok], w_al[ok]) / np.dot(w_al[ok], w_al[ok])) if np.any(w_al[ok]) else float("nan")
    res = gz - signo * w_al
    mov_al = np.abs(np.nan_to_num(w_al)) > 5.0
    rep = {
        "carpeta": os.path.basename(carpeta), "nombre": info.get("nombre", ""), "fecha": info.get("fecha", ""),
        "vmax_mm_s": info.get("vmax_mm_s"), "amax_mm_s2": info.get("amax_mm_s2"),
        "modo_ensamblaje": modo, "duracion_s": float(t[-1] - t[0]), "muestras": int(d.size),
        "frecuencia_hz": float(fs), "huecos": huecos, "muestras_reparadas": reparadas,
        "eslabon": eslabon, "signo": signo, "correlacion_gz": cors.get(eslabon),
        "correlacion_otro_eslabon": cors.get("der" if eslabon == "izq" else "izq"),
        "retardo_ms": lag * 1000 if not math.isnan(lag) else float("nan"),
        "ganancia_gz": ganancia,
        "gz_pico_predicho": float(np.nanmax(np.abs(w_pred))), "gz_pico_medido": float(np.nanmax(np.abs(gz))),
        "error_rms_gz": rms(res), "error_rms_gz_movimiento": rms(res[mov_al]),
        "error_rms_gz_quieto": rms(res[~mov_al]),
    }

    # --- oscilación al detenerse
    stops = paradas(w_al, t)
    amps, freqs, asent = [], [], []
    sin_asentar = 0
    umbral = 3 * RUIDO_GZ_REPOSO
    ventanas = []
    for i in stops:
        # ventana: hasta VENTANA_ASENT_S después de parar o hasta que el eslabón vuelve a moverse
        fin = t[i] + VENTANA_ASENT_S
        sig = np.where((t > t[i]) & (np.abs(np.nan_to_num(w_al)) > 2.0))[0]
        if len(sig):
            fin = min(fin, t[sig[0]])
        v = (t >= t[i]) & (t < fin)
        ventanas.append((i, fin))
        e = res[v]
        if len(e) < 20:
            continue
        amps.append(rms(e[: int(0.5 * fs)]))
        freqs.append(frecuencia_dominante(e, fs))
        env = np.sqrt(np.convolve(np.nan_to_num(e) ** 2, np.ones(5) / 5, mode="same"))
        fuera = np.where(env > umbral)[0]
        if len(fuera) and fuera[-1] >= len(e) - 3:
            sin_asentar += 1                      # sigue oscilando al final de la ventana
        else:
            asent.append(float((fuera[-1] + 1) / fs) if len(fuera) else 0.0)
    rep.update({"paradas": len(stops),
                "oscilacion_rms_dps": float(np.nanmean(amps)) if amps else float("nan"),
                "oscilacion_frecuencia_hz": float(np.nanmedian(freqs)) if freqs else float("nan"),
                "asentamiento_s": float(np.nanmean(asent)) if asent else float("nan"),
                "paradas_sin_asentar": sin_asentar})

    # --- aceleración en el plano
    pos = P - r * np.column_stack([np.cos(fi), np.sin(fi)])     # la IMU está a r del efector, hacia el codo
    vel = np.column_stack([derivada(pos[:, 0], t), derivada(pos[:, 1], t)])
    acc_w = np.column_stack([suavizar(derivada(vel[:, 0], t), 3), suavizar(derivada(vel[:, 1], t), 3)])
    p_link = (acc_w[:, 0] + 1j * acc_w[:, 1]) * np.exp(-1j * fi)   # predicha, en ejes del eslabón
    quieto_ini = (t < t[0] + 0.4) | (np.abs(np.nan_to_num(w_pred)) < 0.5)
    ax0, ay0 = np.median(d["ax"][quieto_ini]), np.median(d["ay"][quieto_ini])
    m = (d["ax"] - ax0) + 1j * (d["ay"] - ay0)                  # medida, sin la gravedad que se cuela
    if not math.isnan(lag):
        p_link = desplazar(p_link.real, t, lag) + 1j * desplazar(p_link.imag, t, lag)
    okc = ~(np.isnan(p_link.real) | np.isnan(p_link.imag))
    a_max = float(np.nanmax(np.abs(p_link))) if okc.any() else 0.0
    psi = float(np.angle(np.sum(p_link[okc] * np.conj(m[okc])))) if okc.any() else 0.0
    p_sensor = p_link * np.exp(-1j * psi)                        # predicha, en ejes del chip
    rep.update({
        "r_mm": r * 1000, "acel_pico_predicha_ms2": a_max,
        "angulo_montaje_deg": math.degrees(psi),
        "correlacion_ax": corr(m.real, p_sensor.real), "correlacion_ay": corr(m.imag, p_sensor.imag),
        "error_rms_acel_ms2": rms(np.abs(m - p_sensor)[okc]),
        "gravedad_colada_ax_ms2": float(ax0), "gravedad_colada_ay_ms2": float(ay0),
    })

    # --- rumbo de la fusión frente al ángulo del eslabón
    yaw = -np.unwrap(np.radians(d["rumbo"]))                    # el rumbo de la BNO055 crece en sentido horario
    dif = np.degrees((yaw - yaw[0]) * signo - (fi - fi[0]))
    rep.update({"rumbo_error_rms_deg": rms(dif - np.nanmean(dif[: max(1, int(0.3 * fs))])),
                "rumbo_error_final_deg": float(dif[-1] - dif[0])})

    # --- salida
    guardar_csv(os.path.join(carpeta, "resultados.csv"), rep)
    with open(os.path.join(carpeta, "resultados.json"), "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False, default=float)
    if raiz:
        agregar_resumen(raiz, rep)
    if graficas:
        dibujar(carpeta, t, gz, w_al, signo, res, ventanas, fs, m, p_sensor, dif, rep)
    imprimir(rep)
    return rep


# ============================================================================ CSV para Excel
def num(v, dec=5):
    if isinstance(v, bool) or v is None:
        return "" if v is None else str(v)
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return "" if math.isnan(v) else ("%.*g" % (dec, v)).replace(".", ",")
    return str(v)


FILAS = [
    ("Corrida", "Nombre", "nombre", ""), ("Corrida", "Fecha", "fecha", ""),
    ("Corrida", "Velocidad maxima", "vmax_mm_s", "mm/s"), ("Corrida", "Aceleracion maxima", "amax_mm_s2", "mm/s²"),
    ("Corrida", "Modo de ensamblaje", "modo_ensamblaje", ""), ("Corrida", "Duracion", "duracion_s", "s"),
    ("Corrida", "Muestras de la IMU", "muestras", ""), ("Corrida", "Frecuencia", "frecuencia_hz", "Hz"),
    ("Corrida", "Huecos en el tiempo", "huecos", ""),
    ("Corrida", "Muestras danadas reparadas", "muestras_reparadas", ""),
    ("Giroscopio", "Eslabon distal de la IMU", "eslabon", ""),
    ("Giroscopio", "Correlacion gz medido / predicho", "correlacion_gz", ""),
    ("Giroscopio", "Correlacion con el otro eslabon", "correlacion_otro_eslabon", ""),
    ("Giroscopio", "Retardo de la IMU respecto al comando", "retardo_ms", "ms"),
    ("Giroscopio", "Ganancia (medido / predicho)", "ganancia_gz", ""),
    ("Giroscopio", "Pico predicho", "gz_pico_predicho", "°/s"), ("Giroscopio", "Pico medido", "gz_pico_medido", "°/s"),
    ("Giroscopio", "Error RMS total", "error_rms_gz", "°/s"),
    ("Giroscopio", "Error RMS en movimiento", "error_rms_gz_movimiento", "°/s"),
    ("Giroscopio", "Error RMS quieto", "error_rms_gz_quieto", "°/s"),
    ("Paradas", "Numero de paradas", "paradas", ""),
    ("Paradas", "Oscilacion RMS (0-0,5 s tras parar)", "oscilacion_rms_dps", "°/s"),
    ("Paradas", "Frecuencia de la oscilacion", "oscilacion_frecuencia_hz", "Hz"),
    ("Paradas", "Tiempo de asentamiento", "asentamiento_s", "s"),
    ("Paradas", "Paradas que no se asentaron (antes de 1,5 s o de volver a moverse)", "paradas_sin_asentar", ""),
    ("Acelerometro", "Distancia r usada", "r_mm", "mm"),
    ("Acelerometro", "Aceleracion pico predicha", "acel_pico_predicha_ms2", "m/s²"),
    ("Acelerometro", "Angulo de montaje del chip (estimado)", "angulo_montaje_deg", "°"),
    ("Acelerometro", "Correlacion ax", "correlacion_ax", ""), ("Acelerometro", "Correlacion ay", "correlacion_ay", ""),
    ("Acelerometro", "Error RMS en el plano", "error_rms_acel_ms2", "m/s²"),
    ("Acelerometro", "Gravedad colada en ax (reposo)", "gravedad_colada_ax_ms2", "m/s²"),
    ("Acelerometro", "Gravedad colada en ay (reposo)", "gravedad_colada_ay_ms2", "m/s²"),
    ("Rumbo", "Error RMS rumbo vs eslabon", "rumbo_error_rms_deg", "°"),
    ("Rumbo", "Diferencia al final", "rumbo_error_final_deg", "°"),
]


def guardar_csv(ruta, rep, filas=None):
    with open(ruta, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["Seccion", "Parametro", "Valor", "Unidad"])
        for sec, nombre, clave, unidad in (filas or FILAS):
            w.writerow([sec, nombre, num(rep.get(clave)), unidad])


def agregar_resumen(raiz, rep, nombre="resumen_etapa1.csv", filas=None):
    ruta = os.path.join(raiz, nombre)
    claves = [c for _, _, c, _ in (filas or FILAS)]
    nuevo = not os.path.exists(ruta)
    try:
        with open(ruta, "a", newline="", encoding="utf-8-sig" if nuevo else "utf-8") as f:
            w = csv.writer(f, delimiter=";")
            if nuevo:
                w.writerow(["carpeta"] + claves)
            w.writerow([rep["carpeta"]] + [num(rep.get(c)) for c in claves])
    except PermissionError:
        print("  AVISO: no se pudo escribir %s (¿abierto en Excel?)." % ruta)


def imprimir(rep):
    def f(v, fmt):
        return "—" if v is None or (isinstance(v, float) and math.isnan(v)) else fmt % v
    print("\n=== %s ===" % rep["carpeta"])
    print("  Vmax %s mm/s | Amax %s mm/s² | %.1f s | %d muestras a %.0f Hz | huecos %d"
          % (rep["vmax_mm_s"], rep["amax_mm_s2"], rep["duracion_s"], rep["muestras"], rep["frecuencia_hz"], rep["huecos"]))
    print("  Giroscopio: eslabon %s | correlacion %s | retardo %s | ganancia %s | error RMS %s (mov. %s, quieto %s)"
          % (rep["eslabon"], f(rep["correlacion_gz"], "%.3f"), f(rep["retardo_ms"], "%.0f ms"),
             f(rep["ganancia_gz"], "%.3f"), f(rep["error_rms_gz"], "%.2f °/s"),
             f(rep["error_rms_gz_movimiento"], "%.2f"), f(rep["error_rms_gz_quieto"], "%.2f")))
    print("  Paradas: %d | oscilacion %s a %s | asentamiento %s | sin asentarse %d"
          % (rep["paradas"], f(rep["oscilacion_rms_dps"], "%.2f °/s"), f(rep["oscilacion_frecuencia_hz"], "%.1f Hz"),
             f(rep["asentamiento_s"], "%.2f s"), rep["paradas_sin_asentar"]))
    if rep.get("muestras_reparadas"):
        print("  Muestras de la IMU danadas y reparadas: %d" % rep["muestras_reparadas"])
    print("  Acelerometro: pico predicho %s | montaje %s | correlacion ax %s, ay %s | error RMS %s"
          % (f(rep["acel_pico_predicha_ms2"], "%.2f m/s²"), f(rep["angulo_montaje_deg"], "%.0f°"),
             f(rep["correlacion_ax"], "%.2f"), f(rep["correlacion_ay"], "%.2f"), f(rep["error_rms_acel_ms2"], "%.3f m/s²")))
    if rep["acel_pico_predicha_ms2"] < 0.3:
        print("  (aceleraciones pequenas frente al ruido: para comparar el acelerometro suban Amax)")
    print("  Rumbo vs eslabon: error RMS %s, al final %s"
          % (f(rep["rumbo_error_rms_deg"], "%.2f°"), f(rep["rumbo_error_final_deg"], "%.2f°")))


# ============================================================================ gráficas
def dibujar(carpeta, t, gz, w_al, signo, res, ventanas, fs, m, p_sensor, dif, rep):
    stops = [i for i, _ in ventanas]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    axs[0].plot(t, signo * w_al, color="#8794A1", lw=1.4, label="predicho por la cinemática (alineado)")
    axs[0].plot(t, gz, color="#0E7C86", lw=0.8, label="medido por la IMU")
    for i in stops:
        axs[0].axvline(t[i], color="#D97A1E", lw=0.6, alpha=0.5)
    axs[0].set_ylabel("gz [°/s]")
    axs[0].legend(loc="upper right")
    axs[0].set_title("%s · retardo %.0f ms · error RMS %.2f °/s" % (rep["nombre"] or rep["carpeta"],
                                                                     rep["retardo_ms"], rep["error_rms_gz"]))
    axs[1].plot(t, res, color="#C0392B", lw=0.7)
    axs[1].set_ylabel("medido − predicho [°/s]")
    axs[1].set_xlabel("tiempo [s]")
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "gz_medido_vs_predicho.png"), dpi=120)

    if stops:
        fig, ax = plt.subplots(figsize=(10, 4.5))
        for i, fin in ventanas:
            v = (t >= t[i] - 0.1) & (t < fin)
            ax.plot(t[v] - t[i], res[v], lw=0.8, alpha=0.8)
        ax.axhline(3 * RUIDO_GZ_REPOSO, color="k", ls="--", lw=0.7)
        ax.axhline(-3 * RUIDO_GZ_REPOSO, color="k", ls="--", lw=0.7, label="±3 σ del ruido en reposo")
        ax.set_xlabel("tiempo desde la parada [s]")
        ax.set_ylabel("medido − predicho [°/s]")
        ax.set_title("Oscilación al detenerse (%d paradas, %d sin asentarse) · %.1f Hz · asentamiento %.2f s"
                     % (len(stops), rep["paradas_sin_asentar"], rep["oscilacion_frecuencia_hz"],
                        rep["asentamiento_s"]))
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(carpeta, "oscilacion_paradas.png"), dpi=120)

    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axs[0].plot(t, p_sensor.real, color="#8794A1", lw=1.2, label="predicha")
    axs[0].plot(t, m.real, color="#0E7C86", lw=0.6, label="medida")
    axs[0].set_ylabel("ax [m/s²]")
    axs[1].plot(t, p_sensor.imag, color="#8794A1", lw=1.2, label="predicha")
    axs[1].plot(t, m.imag, color="#0E7C86", lw=0.6, label="medida")
    axs[1].set_ylabel("ay [m/s²]")
    axs[2].plot(t, dif, color="#7D3C98", lw=0.8)
    axs[2].set_ylabel("rumbo − eslabón [°]")
    axs[2].set_xlabel("tiempo [s]")
    axs[0].set_title("Aceleración en el plano (montaje estimado %.0f°, r = %.0f mm) y rumbo de la fusión"
                     % (rep["angulo_montaje_deg"], rep["r_mm"]))
    for a in axs:
        a.grid(alpha=0.3)
    axs[0].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "aceleracion_y_rumbo.png"), dpi=120)
    plt.close("all")


# ============================================================================ prueba de golpes
FILAS_GOLPES = [
    ("Grabacion", "Nombre", "nombre", ""), ("Grabacion", "Fecha", "fecha", ""),
    ("Grabacion", "Duracion", "duracion_s", "s"), ("Grabacion", "Muestras de la IMU", "muestras", ""),
    ("Grabacion", "Frecuencia", "frecuencia_hz", "Hz"), ("Grabacion", "Huecos en el tiempo", "huecos", ""),
    ("Grabacion", "Muestras danadas reparadas", "muestras_reparadas", ""),
    ("Postura", "x del efector", "x_mm", "mm"), ("Postura", "y del efector", "y_mm", "mm"),
    ("Postura", "Angulo motor 1", "q1_deg", "°"), ("Postura", "Angulo motor 2", "q2_deg", "°"),
    ("Postura", "Eslabon distal de la IMU", "eslabon", ""),
    ("Postura", "Angulo del eslabon de la IMU", "phi_deg", "°"),
    ("Golpes", "Golpes detectados", "golpes", ""),
    ("Golpes", "Golpes analizados", "golpes_validos", ""),
    ("Golpes", "Medios ciclos usados", "medios_ciclos", ""),
    ("Golpes", "Frecuencia natural (mediana)", "fn_hz", "Hz"),
    ("Golpes", "Frecuencia natural (dispersion entre golpes)", "fn_std_hz", "Hz"),
    ("Golpes", "Amortiguamiento zeta (mediana)", "zeta", ""),
    ("Golpes", "Amortiguamiento zeta (dispersion entre golpes)", "zeta_std", ""),
    ("Por amplitud", "fn con amplitud grande (>= 20 °/s)", "fn_grande_hz", "Hz"),
    ("Por amplitud", "zeta con amplitud grande", "zeta_grande", ""),
    ("Por amplitud", "medios ciclos con amplitud grande", "n_grande", ""),
    ("Por amplitud", "fn con amplitud media (8-20 °/s)", "fn_media_hz", "Hz"),
    ("Por amplitud", "zeta con amplitud media", "zeta_media", ""),
    ("Por amplitud", "medios ciclos con amplitud media", "n_media", ""),
    ("Por amplitud", "fn con amplitud pequena (< 8 °/s)", "fn_pequena_hz", "Hz"),
    ("Por amplitud", "zeta con amplitud pequena", "zeta_pequena", ""),
    ("Por amplitud", "medios ciclos con amplitud pequena", "n_pequena", ""),
    ("Golpes", "Asentamiento desde el rebote (mediana, hasta 3 sigma del ruido)", "asentamiento_s", "s"),
    ("Golpes", "Pico de gz (mediana)", "pico_dps", "°/s"),
    ("Reposo", "Ruido de gz en reposo", "ruido_gz_dps", "°/s"),
    ("Reposo", "ax en reposo (gravedad colada)", "ax0_ms2", "m/s²"),
    ("Reposo", "ay en reposo (gravedad colada)", "ay0_ms2", "m/s²"),
    ("Reposo", "az en reposo", "az0_ms2", "m/s²"),
    ("Reposo", "Inclinacion total (de ax, ay)", "inclinacion_deg", "°"),
    ("Reposo", "Roll de la fusion", "roll_deg", "°"), ("Reposo", "Pitch de la fusion", "pitch_deg", "°"),
]

CLAVES_INCLINACION = ["x_mm", "y_mm", "phi_deg", "ax0_ms2", "ay0_ms2", "roll_deg", "pitch_deg"]


BANDAS = [("grande", 20.0, 1e9), ("media", 8.0, 20.0), ("pequena", 0.0, 8.0)]   # °/s, amplitud del medio ciclo


def extremos(e, t0, dt):
    """Extremos de la señal entre cruces por cero: lista de (t, |amplitud|), con el pico refinado por parábola."""
    s = np.sign(e)
    zc = np.where(s[:-1] * s[1:] < 0)[0]
    out = []
    for a, b in zip(np.r_[0, zc + 1], np.r_[zc + 1, len(e)]):
        if b - a < 2:
            continue
        j = a + int(np.argmax(np.abs(e[a:b])))
        tj, aj = t0 + j * dt, abs(e[j])
        if 0 < j < len(e) - 1:
            y0, y1, y2 = abs(e[j - 1]), abs(e[j]), abs(e[j + 1])
            den = y0 - 2 * y1 + y2
            if den < 0:
                dj = 0.5 * (y0 - y2) / den
                tj, aj = t0 + (j + dj) * dt, y1 - 0.25 * (y0 - y2) * dj
        out.append((tj, aj))
    return out


def ajustar_golpe(e, fs, ruido):
    """Decremento logarítmico entre extremos sucesivos (medios ciclos) de la oscilación libre.
    e empieza en el cruce del umbral. Se toma como inicio el rebote (el extremo más grande entre los
    3 primeros) y se descarta el primer medio ciclo después de él, en el que el dedo todavía influye.
    Devuelve dict con la lista de medios ciclos (t, amplitud, fn, zeta), el tiempo del rebote y el
    asentamiento, o None."""
    ext = extremos(e, 0.0, 1.0 / fs)
    ext = [q for q in ext if q[1] > 1.0]
    if len(ext) < 4:
        return None
    im = int(np.argmax([q[1] for q in ext[:3]]))
    t_reb = ext[im][0]
    ext = ext[im + 1:]
    piso = max(8 * ruido, 2.5)
    medios = []
    for (t1, a1), (t2, a2) in zip(ext[:-1], ext[1:]):
        if a2 < piso:
            break
        dh = math.log(a1 / a2)                       # decremento por medio ciclo
        dl = 2 * dh                                  # por ciclo
        z = dl / math.sqrt(4 * math.pi ** 2 + dl * dl)
        fd = 1.0 / (2 * (t2 - t1))
        if not (0 < z < 1 and 0.3 < fd < 40):
            continue
        medios.append((t1, a1, fd / math.sqrt(1 - z * z), z))
    if len(medios) < 2:
        return None
    # asentamiento: desde el rebote hasta que la amplitud RMS (0.15 s) queda bajo 3 sigma del ruido
    nv = max(3, int(0.15 * fs))
    env = np.sqrt(2 * np.convolve(e ** 2, np.ones(nv) / nv, mode="same"))
    k0 = int(round(t_reb * fs))
    fuera = np.where(env[k0:] > 3 * ruido)[0]
    ts = float((fuera[-1] + 1) / fs) if len(fuera) else 0.0
    return {"medios": medios, "t_rebote": t_reb, "asent": ts,
            "fn": float(np.median([m[2] for m in medios])), "zeta": float(np.median([m[3] for m in medios]))}


def analizar_golpes(carpeta, eslabon=None, raiz=None, graficas=True):
    info = leer_info(carpeta)
    d, reparadas = leer_imu(carpeta)
    nombre = os.path.basename(carpeta)
    if d.size < 200:
        print("  %s: grabación muy corta (%d muestras)." % (nombre, d.size))
        return None
    gm = info.get("geometria_mm") or {"L1": 120, "L2": 180, "L3": 180, "L4": 120, "D": 150}
    g = tuple(gm[k] / 1000.0 for k in ("L1", "L2", "L3", "L4", "D"))
    modo = int(info.get("modo_ensamblaje", 1))
    eslabon = eslabon or info.get("eslabon") or "der"

    t = d["t_s"]
    fs = 1.0 / np.median(np.diff(t))
    huecos = int(np.sum(np.diff(t) > 1.5 / fs))
    gz = d["gz"].astype(float)

    # --- postura (ángulos de los motores, constantes con el robot quieto)
    q1, q2 = float(np.median(d["q1_deg"])), float(np.median(d["q2_deg"]))
    p0 = None
    if "x_mm" in info and "y_mm" in info:
        p0 = (info["x_mm"] / 1000.0, info["y_mm"] / 1000.0)
    P = efector(np.radians([q1]), np.radians([q2]), modo, g, p0)[0]
    if eslabon == "izq":
        base = (g[0] * math.cos(math.radians(q1)), g[0] * math.sin(math.radians(q1)))
    else:
        base = (g[4] + g[3] * math.cos(math.radians(q2)), g[3] * math.sin(math.radians(q2)))
    phi = math.degrees(math.atan2(P[1] - base[1], P[0] - base[0])) if np.all(np.isfinite(P)) else float("nan")

    # --- ruido y reposo: estimación robusta (los golpes son una fracción pequeña del tiempo)
    sesgo = float(np.median(gz))
    x = gz - sesgo
    ruido = float(1.4826 * np.median(np.abs(x - np.median(x))))
    ruido = max(ruido, 0.03)
    umbral = max(10 * ruido, 2.0)

    # --- golpes: picos de |gz| sobre el umbral; el siguiente solo cuenta cuando la oscilación del
    #     anterior ya bajó del umbral durante 0.3 s (y al menos 0.8 s después)
    golpes = []
    i, n = 0, len(x)
    calma = int(0.3 * fs)
    while i < n:
        if abs(x[i]) > umbral:
            j = min(n, i + int(0.25 * fs))
            k = i + int(np.argmax(np.abs(x[i:j])))
            golpes.append(k)
            i = k + int(0.8 * fs)
            while i < n and np.any(np.abs(x[i:i + calma]) > umbral):
                i += 1
            i += calma
        else:
            i += 1

    # quieto: lejos (>0.3 s antes, >3 s después) de cualquier golpe
    quieto = np.ones(n, dtype=bool)
    for k in golpes:
        quieto &= ~((t > t[k] - 0.3) & (t < t[k] + 3.0))
    if quieto.sum() < int(0.5 * fs):
        quieto = np.abs(x) < 3 * ruido
    if quieto.sum() > int(0.5 * fs):
        ruido = max(float(np.std(x[quieto])), 0.03)

    fns, zetas, tss, picos, ajustes = [], [], [], [], []
    medios = []                                   # todos los medios ciclos de todos los golpes
    for a, k in enumerate(golpes):
        ini = max(0, k - int(0.3 * fs))
        while ini < k and abs(x[ini]) < umbral:   # desde el cruce del umbral
            ini += 1
        ini = max(0, ini - 3)
        fin = golpes[a + 1] - int(0.3 * fs) if a + 1 < len(golpes) else n
        fin = min(fin, k + int(3.0 * fs))
        e = x[ini:fin]
        picos.append(float(np.max(np.abs(e))) if len(e) else float("nan"))
        r = ajustar_golpe(e, fs, ruido)
        if r is not None:
            r["ini"] = ini
            fns.append(r["fn"])
            zetas.append(r["zeta"])
            tss.append(r["asent"])
            medios += r["medios"]
        ajustes.append(r)

    def banda(lo, hi, col):
        v = [m[col] for m in medios if lo <= m[1] < hi]
        return float(np.median(v)) if v else float("nan"), len(v)

    med = lambda v: float(np.median(v)) if v else float("nan")
    std = lambda v: float(np.std(v)) if len(v) > 1 else float("nan")
    ax0 = float(np.median(d["ax"][quieto]))
    ay0 = float(np.median(d["ay"][quieto]))
    az0 = float(np.median(d["az"][quieto]))
    rep = {
        "carpeta": nombre, "nombre": info.get("nombre", ""), "fecha": info.get("fecha", ""), "tipo": "grabacion",
        "duracion_s": float(t[-1] - t[0]), "muestras": int(d.size), "frecuencia_hz": float(fs),
        "huecos": huecos, "muestras_reparadas": reparadas,
        "x_mm": float(P[0] * 1000) if np.all(np.isfinite(P)) else float("nan"),
        "y_mm": float(P[1] * 1000) if np.all(np.isfinite(P)) else float("nan"),
        "q1_deg": q1, "q2_deg": q2, "eslabon": eslabon, "phi_deg": phi,
        "golpes": len(golpes), "golpes_validos": len(fns),
        "fn_hz": float(np.median([m[2] for m in medios])) if medios else float("nan"),
        "fn_std_hz": std(fns),
        "zeta": float(np.median([m[3] for m in medios])) if medios else float("nan"),
        "zeta_std": std(zetas), "medios_ciclos": len(medios),
        "asentamiento_s": med(tss), "pico_dps": med(picos),
        "ruido_gz_dps": ruido, "ax0_ms2": ax0, "ay0_ms2": ay0, "az0_ms2": az0,
        "inclinacion_deg": math.degrees(math.atan2(math.hypot(ax0, ay0), az0)),
        "roll_deg": float(np.median(d["roll"][quieto])), "pitch_deg": float(np.median(d["pitch"][quieto])),
    }

    for nb, lo, hi in BANDAS:
        rep["fn_%s_hz" % nb], rep["n_%s" % nb] = banda(lo, hi, 2)
        rep["zeta_%s" % nb], _ = banda(lo, hi, 3)

    guardar_csv(os.path.join(carpeta, "resultados.csv"), rep, FILAS_GOLPES)
    with open(os.path.join(carpeta, "resultados.json"), "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False, default=float)
    with open(os.path.join(carpeta, "medios_ciclos.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["golpe", "t_desde_cruce_s", "amplitud_dps", "fn_hz", "zeta"])
        for g, r in enumerate(ajustes, 1):
            for m in (r["medios"] if r else []):
                w.writerow([g, num(m[0]), num(m[1]), num(m[2]), num(m[3])])
    if raiz:
        agregar_resumen(raiz, rep, "resumen_golpes.csv", FILAS_GOLPES)
    if graficas:
        dibujar_golpes(carpeta, t, x, golpes, ajustes, fs, ruido, umbral, rep)

    def f(v, fmt):
        return "—" if v is None or (isinstance(v, float) and math.isnan(v)) else fmt % v
    print("\n=== %s (golpes) ===" % nombre)
    print("  Postura: x %s, y %s | motores %.1f° / %.1f° | eslabon %s a %s"
          % (f(rep["x_mm"], "%.0f mm"), f(rep["y_mm"], "%.0f mm"), q1, q2, eslabon, f(phi, "%.1f°")))
    print("  Golpes: %d detectados, %d analizados (%d medios ciclos) | fn %s | zeta %s | asentamiento %s"
          % (len(golpes), len(fns), len(medios), f(rep["fn_hz"], "%.2f Hz"), f(rep["zeta"], "%.3f"),
             f(rep["asentamiento_s"], "%.2f s")))
    print("  Por amplitud: " + " | ".join(
        "%s (%s) fn %s, zeta %s, n=%d" % (nb, ">=%g °/s" % lo if hi > 1e8 else "%g-%g °/s" % (lo, hi),
                                         f(rep["fn_%s_hz" % nb], "%.2f Hz"), f(rep["zeta_%s" % nb], "%.3f"),
                                         rep["n_%s" % nb]) for nb, lo, hi in BANDAS))
    print("  Reposo: ruido gz %.3f °/s | ax %.3f, ay %.3f m/s² | inclinacion %.2f° | roll %.2f°, pitch %.2f°"
          % (ruido, ax0, ay0, rep["inclinacion_deg"], rep["roll_deg"], rep["pitch_deg"]))
    if not golpes:
        print("  No se detectaron golpes: denlos un poco más fuertes, en el plano del robot.")
    if reparadas:
        print("  Muestras de la IMU danadas y reparadas: %d" % reparadas)
    return rep


def dibujar_golpes(carpeta, t, x, golpes, ajustes, fs, ruido, umbral, rep):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 9.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.2], width_ratios=[1.6, 1])
    a0 = fig.add_subplot(gs[0, :])
    a1 = fig.add_subplot(gs[1, 0])
    a2 = fig.add_subplot(gs[1, 1])
    a0.plot(t, x, color="#0E7C86", lw=0.7)
    for k in golpes:
        a0.axvline(t[k], color="#D97A1E", lw=0.7, alpha=0.6)
    a0.axhline(umbral, color="k", ls=":", lw=0.6)
    a0.axhline(-umbral, color="k", ls=":", lw=0.6, label="umbral de detección")
    a0.set_xlabel("tiempo [s]")
    a0.set_ylabel("gz [°/s]")
    a0.legend(loc="upper right")
    a0.set_title("%s · %d golpes · fn %.2f Hz · ζ %.3f · asentamiento %.2f s"
                 % (rep["nombre"] or rep["carpeta"], len(golpes), rep["fn_hz"], rep["zeta"], rep["asentamiento_s"]))
    # golpes superpuestos, alineados en el rebote
    for g, r in enumerate(ajustes):
        if r is None:
            continue
        k0 = r["ini"] + int(round(r["t_rebote"] * fs))
        fin = golpes[g + 1] - int(0.3 * fs) if g + 1 < len(golpes) else len(x)
        fin = min(fin, k0 + int(2.0 * fs))
        v = slice(max(0, k0 - int(0.3 * fs)), fin)
        a1.plot(t[v] - t[k0], x[v], lw=0.8, alpha=0.85,
                label="golpe %d: %.2f Hz, ζ %.3f" % (g + 1, r["fn"], r["zeta"]))
    a1.axhline(3 * ruido, color="k", ls="--", lw=0.6)
    a1.axhline(-3 * ruido, color="k", ls="--", lw=0.6)
    a1.axvline(0, color="k", lw=0.5)
    a1.set_xlabel("tiempo desde el rebote [s]")
    a1.set_ylabel("gz [°/s]")
    if any(r is not None for r in ajustes):
        a1.legend(fontsize=7.5, ncol=2, loc="upper right")
    # fn y zeta de cada medio ciclo frente a su amplitud
    med = [m for r in ajustes if r for m in r["medios"]]
    if med:
        amp = [m[1] for m in med]
        a2.scatter(amp, [m[2] for m in med], s=14, color="#0E7C86", label="fn [Hz]")
        a2.set_xscale("log")
        marcas = [m for m in (2, 3, 5, 10, 20, 30, 50, 100, 200) if min(amp) / 1.2 <= m <= max(amp) * 1.2]
        a2.set_xticks(marcas)
        a2.set_xticklabels([str(m) for m in marcas])
        a2.minorticks_off()
        a2.set_xlabel("amplitud del medio ciclo [°/s]")
        a2.set_ylabel("fn [Hz]", color="#0E7C86")
        b2 = a2.twinx()
        b2.scatter(amp, [m[3] for m in med], s=14, color="#D97A1E", marker="s", label="ζ")
        b2.set_ylabel("ζ", color="#D97A1E")
        b2.set_ylim(0, max(0.5, 1.1 * max(m[3] for m in med)))
        a2.set_title("fn y ζ según la amplitud")
    for a in (a0, a1, a2):
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "golpes.png"), dpi=120)
    plt.close("all")


def ajustar_inclinacion(raiz):
    """Con el robot quieto en varias posturas separa la gravedad que se cuela en ax, ay en:
       - una parte FIJA en los ejes del chip (la IMU o el eslabón inclinados en su montaje), y
       - una parte que GIRA con el eslabón (el plano del robot inclinado respecto a la horizontal).
    Modelo: ax = cx + p1 cos(phi) + p2 sin(phi),  ay = cy - p1 sin(phi) + p2 cos(phi)."""
    filas = []
    for c in corridas(raiz):
        ruta = os.path.join(c, "resultados.json")
        if not os.path.isfile(ruta):
            continue
        with open(ruta, encoding="utf-8") as f:
            r = json.load(f)
        if r.get("tipo") == "grabacion" and all(np.isfinite(r.get(k, float("nan"))) for k in CLAVES_INCLINACION):
            filas.append(r)
    if len(filas) < 2:
        return None
    eslabones = {r["eslabon"] for r in filas}
    if len(eslabones) > 1:
        print("\nInclinacion: hay grabaciones con la IMU en eslabones distintos; no se combinan.")
        return None
    phi = np.radians([r["phi_deg"] for r in filas])
    if np.ptp(np.unwrap(phi)) < math.radians(20):
        print("\nInclinacion: las posturas giran el eslabon menos de 20°; graben posturas mas distintas "
              "para separar la parte fija de la del plano.")
        return None
    A, b = [], []
    for r, p in zip(filas, phi):
        A.append([1, 0, math.cos(p), math.sin(p)]); b.append(r["ax0_ms2"])
        A.append([0, 1, -math.sin(p), math.cos(p)]); b.append(r["ay0_ms2"])
    sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    resid = np.array(b) - np.array(A) @ sol
    cx, cy, p1, p2 = (float(v) for v in sol)
    out = {
        "posturas": len(filas),
        "fija_ax_ms2": cx, "fija_ay_ms2": cy,
        "fija_deg": math.degrees(math.asin(min(1.0, math.hypot(cx, cy) / G0))),
        "plano_p1_ms2": p1, "plano_p2_ms2": p2,
        "plano_deg": math.degrees(math.asin(min(1.0, math.hypot(p1, p2) / G0))),
        "residuo_rms_ms2": float(np.sqrt(np.mean(resid ** 2))),
    }
    ruta = os.path.join(raiz, "resumen_inclinacion.csv")
    filas_csv = [
        ("Posturas usadas", "posturas", ""),
        ("Parte fija en ax (IMU en su montaje)", "fija_ax_ms2", "m/s²"),
        ("Parte fija en ay (IMU en su montaje)", "fija_ay_ms2", "m/s²"),
        ("Parte fija: inclinacion", "fija_deg", "°"),
        ("Parte que gira con el eslabon: p1", "plano_p1_ms2", "m/s²"),
        ("Parte que gira con el eslabon: p2", "plano_p2_ms2", "m/s²"),
        ("Plano del robot: inclinacion", "plano_deg", "°"),
        ("Residuo RMS del ajuste (flexion, ruido)", "residuo_rms_ms2", "m/s²"),
    ]
    try:
        with open(ruta, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f, delimiter=";")
            w.writerow(["Parametro", "Valor", "Unidad"])
            for nombre, clave, unidad in filas_csv:
                w.writerow([nombre, num(out[clave]), unidad])
    except PermissionError:
        print("  AVISO: no se pudo escribir %s (¿abierto en Excel?)." % ruta)
    print("\nInclinacion con %d posturas: parte fija %.2f° (ax %.3f, ay %.3f m/s²) | plano del robot %.2f° | "
          "residuo %.3f m/s²" % (out["posturas"], out["fija_deg"], cx, cy, out["plano_deg"], out["residuo_rms_ms2"]))
    if out["posturas"] == 2:
        print("  (con 2 posturas el ajuste es exacto y no hay residuo: con 3 o 4 se puede verificar)")
    return out


# ============================================================================ main
def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    p = argparse.ArgumentParser(description="Etapa 1: IMU frente a la cinemática")
    p.add_argument("carpeta", nargs="?", help="carpeta de una corrida (con imu.csv)")
    p.add_argument("--ultima", action="store_true", help="analizar la corrida más reciente")
    p.add_argument("--todas", action="store_true", help="analizar las corridas que aún no tienen resultados")
    p.add_argument("--raiz", help="carpeta etapa1 (por defecto Descargas/Resultados_BNO055/etapa1)")
    p.add_argument("--r_mm", type=float, help="distancia del eje del efector al chip, en mm (por defecto 0)")
    p.add_argument("--eslabon", choices=["izq", "der"], help="eslabón distal de la IMU (por defecto se detecta)")
    p.add_argument("--rehacer", action="store_true",
                   help="con --todas: vuelve a analizar también las que ya tienen resultados y rehace los resúmenes")
    p.add_argument("--k", type=float, default=1.0,
                   help="escala del giroscopio de la calibración (el gz medido se divide por k)")
    p.add_argument("--sin-graficas", action="store_true")
    a = p.parse_args()
    global ESCALA_GIRO
    ESCALA_GIRO = a.k

    raiz = os.path.abspath(a.raiz) if a.raiz else raiz_etapa1()
    if a.carpeta:
        lista = [os.path.abspath(a.carpeta)]
        raiz_resumen = os.path.dirname(lista[0])
    else:
        todas = corridas(raiz)
        if not todas:
            sys.exit("No hay corridas en %s. Ejecuten una figura con R5B Lab y la IMU conectada." % raiz)
        if a.ultima:
            lista = todas[-1:]
        elif a.todas and a.rehacer:
            lista = todas
            for nombre in ("resumen_etapa1.csv", "resumen_golpes.csv", "resumen_inclinacion.csv"):
                ruta = os.path.join(raiz, nombre)
                if os.path.isfile(ruta):
                    try:
                        os.remove(ruta)
                    except PermissionError:
                        sys.exit("Cierren %s en Excel y vuelvan a intentarlo." % ruta)
        elif a.todas:
            lista = [c for c in todas if not os.path.isfile(os.path.join(c, "resultados.csv"))]
            if not lista:
                print("Todas las corridas ya tienen resultados. Para volver a analizarlas: --todas --rehacer")
                return
        else:
            sys.exit("Indiquen una carpeta, --ultima o --todas.")
        raiz_resumen = raiz
    hubo_golpes = False
    for c in lista:
        try:
            rep = analizar(c, r_mm=a.r_mm, eslabon=a.eslabon, raiz=raiz_resumen, graficas=not a.sin_graficas)
        except Exception as exc:                  # una corrida dañada no detiene las demás
            print("\n=== %s ===\n  ERROR al analizar: %s" % (os.path.basename(c), exc))
            continue
        hubo_golpes |= bool(rep and rep.get("tipo") == "grabacion")
    if hubo_golpes:
        ajustar_inclinacion(raiz_resumen)
    print("\nResultados en cada carpeta (resultados.csv y gráficas) y resúmenes en %s" % raiz_resumen)


if __name__ == "__main__":
    main()
