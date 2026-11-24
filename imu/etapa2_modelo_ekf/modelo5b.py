"""
Modelo dinámico del robot de 5 barras (etapa 2) — biblioteca común para la identificación y el EKF.

Estados (por motor i = 1, 2):  q_i  ángulo real del motor [rad],  w_i  velocidad [rad/s]
Entrada:                       q_c,i(t)  ángulo comandado por la ESP32 (pulsos), con retardo tau

Dinámica (Lagrange, 2 grados de libertad):
    M(q) q'' + C(q, q') q' = K (q_c(t - tau) - q) + B (ff * q_c'(t - tau) - q')
- M(q): inercia que ven los motores en cada postura (proximales, distales, efector como masa
        puntual y rotores), con las masas medidas.
- Cada servo es su lazo de posición: rigidez K = Kp*Kv y amortiguamiento B = Kv, con
  feedforward de velocidad ff (P02-03 = 30 % en el driver). Se espera K/B ≈ Kp = 20 1/s.
Medición de la IMU (en el distal derecho):
    gz = k_g * w3,   w3 = dφ3/dt = J3(q) · q'      (k_g = 1.031, escala del giroscopio)

Para que la simulación sea rápida, M, ∂M/∂q y J3 se calculan sobre la trayectoria COMANDADA
(el robot real se aparta de ella solo unos grados, y M cambia poco en ese rango).
"""
import json
import math
import os

import numpy as np

# ------------------------------------------------------------------ parámetros físicos
GEOM = {"L1": 0.120, "L2": 0.180, "L3": 0.180, "L4": 0.120, "D": 0.150}
MASAS = {"prox": 0.115, "dist": 0.170, "total": 0.904}
MASAS["efector"] = MASAS["total"] - 2 * MASAS["prox"] - 2 * MASAS["dist"]      # 0.334 kg
CM = {"prox": 0.045, "dist": 0.090}          # centros de masa estimados (el modelo es poco sensible)
J_ROTOR = 0.3e-4                             # kg·m² (P00-04 = 0.3 kg·cm²)
K_GIRO = 1.031                               # escala del giroscopio (calibración cinemática)


# ------------------------------------------------------------------ cinemática
def fk(t1, t2, g=GEOM):
    L1, L2, L3, L4, D = g["L1"], g["L2"], g["L3"], g["L4"], g["D"]
    ax, ay = L1 * math.cos(t1), L1 * math.sin(t1)
    bx, by = D + L4 * math.cos(t2), L4 * math.sin(t2)
    dx, dy = bx - ax, by - ay
    d = math.hypot(dx, dy)
    a = (L2 * L2 - L3 * L3 + d * d) / (2 * d)
    h = math.sqrt(max(L2 * L2 - a * a, 0.0))
    mx, my = ax + a * dx / d, ay + a * dy / d
    nx, ny = -dy / d, dx / d
    p1, p2 = (mx + h * nx, my + h * ny), (mx - h * nx, my - h * ny)
    px, py = p1 if p1[1] >= p2[1] else p2          # en los modos de trabajo el efector va arriba
    return (ax, ay), (bx, by), (px, py)


def cuerpos(t1, t2):
    """Posiciones de los centros de masa (10) y ángulos (5) de los cuerpos."""
    (ax, ay), (bx, by), (px, py) = fk(t1, t2)
    D = GEOM["D"]
    rp, rd = CM["prox"], CM["dist"]
    a2 = math.atan2(py - ay, px - ax)
    a3 = math.atan2(py - by, px - bx)
    L1, L2, L3, L4 = GEOM["L1"], GEOM["L2"], GEOM["L3"], GEOM["L4"]
    c = [rp * ax / L1, rp * ay / L1,
         D + rp * (bx - D) / L4, rp * by / L4,
         ax + rd * (px - ax) / L2, ay + rd * (py - ay) / L2,
         bx + rd * (px - bx) / L3, by + rd * (py - by) / L3,
         px, py]
    return np.array(c), np.array([t1, t2, a2, a3, 0.0])


def jacobianos(t1, t2, e=1e-6):
    Jc = np.zeros((10, 2))
    Ja = np.zeros((5, 2))
    for j in range(2):
        d = [0.0, 0.0]
        d[j] = e
        c1, a1 = cuerpos(t1 + d[0], t2 + d[1])
        c2, a2 = cuerpos(t1 - d[0], t2 - d[1])
        Jc[:, j] = (c1 - c2) / (2 * e)
        Ja[:, j] = (np.unwrap(np.array([a2, a1]), axis=0)[1] - a2) / (2 * e)
    return Jc, Ja


def masa(t1, t2):
    """M(q) [kg·m²] y J3 = dφ3/dq (fila del distal derecho)."""
    Jc, Ja = jacobianos(t1, t2)
    m = [MASAS["prox"], MASAS["prox"], MASAS["dist"], MASAS["dist"], MASAS["efector"]]
    L = [GEOM["L1"], GEOM["L4"], GEOM["L2"], GEOM["L3"], 0.0]
    M = np.diag([J_ROTOR, J_ROTOR])
    for i in range(5):
        Jv = Jc[2 * i:2 * i + 2]
        M = M + m[i] * (Jv.T @ Jv) + (m[i] * L[i] ** 2 / 12.0) * np.outer(Ja[i], Ja[i])
    return M, Ja[3].copy()


def masa_y_derivadas(t1, t2, e=1e-4):
    M, J3 = masa(t1, t2)
    dM = []
    for j in range(2):
        d = [0.0, 0.0]
        d[j] = e
        Mp, _ = masa(t1 + d[0], t2 + d[1])
        Mm, _ = masa(t1 - d[0], t2 - d[1])
        dM.append((Mp - Mm) / (2 * e))
    return M, dM, J3


# ------------------------------------------------------------------ datos
def leer_corrida(carpeta):
    d = np.genfromtxt(os.path.join(carpeta, "imu.csv"), delimiter=",", names=True)
    info = {}
    if os.path.isfile(os.path.join(carpeta, "info.json")):
        with open(os.path.join(carpeta, "info.json"), encoding="utf-8") as f:
            info = json.load(f)
    t = d["t_s"].astype(float)
    # lecturas dañadas de la IMU (|a| imposible): se reemplazan por la anterior
    amag = np.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2)
    malas = (amag < 2) | (amag > 40) | ((d["cal_sis"] == 3) & (d["cal_mag"] == 3))
    for k in ("gz", "ax", "ay"):
        x = d[k].astype(float)
        for i in np.where(malas)[0]:
            if i > 0:
                x[i] = x[i - 1]
        d[k] = x
    return {"t": t, "q1c": np.radians(d["q1_deg"].astype(float)), "q2c": np.radians(d["q2_deg"].astype(float)),
            "gz": np.radians(d["gz"].astype(float)), "ax": d["ax"].astype(float), "ay": d["ay"].astype(float),
            "info": info, "nombre": os.path.basename(carpeta)}


def derivada(x, t):
    """Derivada suavizada (diferencia central sobre ±2 muestras): el comando viene cuantizado
    en pasos de 0.045° y una diferencia simple a 100 Hz mete 4.5 °/s de ruido."""
    v = np.zeros_like(x)
    v[2:-2] = (x[4:] - x[:-4]) / (t[4:] - t[:-4])
    v[:2], v[-2:] = v[2], v[-3]
    return v


def preparar(c):
    """Precalcula M, ∂M/∂q y J3 sobre la trayectoria comandada."""
    n = len(c["t"])
    c["M"] = np.zeros((n, 2, 2))
    c["dM1"] = np.zeros((n, 2, 2))
    c["dM2"] = np.zeros((n, 2, 2))
    c["J3"] = np.zeros((n, 2))
    cache = {}
    for i in range(n):
        key = (round(c["q1c"][i], 5), round(c["q2c"][i], 5))
        if key not in cache:
            cache[key] = masa_y_derivadas(c["q1c"][i], c["q2c"][i])
        M, dM, J3 = cache[key]
        c["M"][i], c["dM1"][i], c["dM2"][i], c["J3"][i] = M, dM[0], dM[1], J3
    c["v1c"] = derivada(c["q1c"], c["t"])
    c["v2c"] = derivada(c["q2c"], c["t"])
    return c


# ------------------------------------------------------------------ simulación
def simular(c, p, sub=2):
    """Simula la corrida con los parámetros p = {K, B, ff, tau}. Devuelve q, w (n×2) y w3 [rad/s]."""
    t = c["t"]
    n = len(t)
    K1 = p.get("K1", p["K"])
    K2 = p.get("K2", p["K"])
    B1 = p.get("B1", p["B"])
    B2 = p.get("B2", p["B"])
    ff, tau = p["ff"], p["tau"]
    td = t - tau
    q1r = np.interp(td, t, c["q1c"])
    q2r = np.interp(td, t, c["q2c"])
    v1r = ff * np.interp(td, t, c["v1c"])
    v2r = ff * np.interp(td, t, c["v2c"])
    M, D1, D2 = c["M"], c["dM1"], c["dM2"]
    q = np.zeros((n, 2))
    w = np.zeros((n, 2))
    x = [q1r[0], q2r[0], 0.0, 0.0]

    def f(x, i, a):
        # a ∈ [0, 1]: fracción del intervalo (entrada interpolada linealmente)
        j = min(i + 1, n - 1)
        r1 = q1r[i] + a * (q1r[j] - q1r[i])
        r2 = q2r[i] + a * (q2r[j] - q2r[i])
        s1 = v1r[i] + a * (v1r[j] - v1r[i])
        s2 = v2r[i] + a * (v2r[j] - v2r[i])
        m = M[i]
        w1, w2 = x[2], x[3]
        # Coriolis: C q' = Ṁ q' − ½ ∂(q'ᵀ M q')/∂q
        mdot = D1[i] * w1 + D2[i] * w2
        g1 = 0.5 * (w1 * (D1[i][0, 0] * w1 + D1[i][0, 1] * w2) + w2 * (D1[i][1, 0] * w1 + D1[i][1, 1] * w2))
        g2 = 0.5 * (w1 * (D2[i][0, 0] * w1 + D2[i][0, 1] * w2) + w2 * (D2[i][1, 0] * w1 + D2[i][1, 1] * w2))
        cor1 = mdot[0, 0] * w1 + mdot[0, 1] * w2 - g1
        cor2 = mdot[1, 0] * w1 + mdot[1, 1] * w2 - g2
        tau1 = K1 * (r1 - x[0]) + B1 * (s1 - w1) - cor1
        tau2 = K2 * (r2 - x[1]) + B2 * (s2 - w2) - cor2
        det = m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0]
        a1 = (m[1, 1] * tau1 - m[0, 1] * tau2) / det
        a2 = (-m[1, 0] * tau1 + m[0, 0] * tau2) / det
        return [w1, w2, a1, a2]

    for i in range(n):
        q[i] = x[0], x[1]
        w[i] = x[2], x[3]
        if i == n - 1:
            break
        h = (t[i + 1] - t[i]) / sub
        for s in range(sub):
            a0 = s / sub
            da = 1.0 / sub
            k1 = f(x, i, a0)
            k2 = f([x[k] + 0.5 * h * k1[k] for k in range(4)], i, a0 + 0.5 * da)
            k3 = f([x[k] + 0.5 * h * k2[k] for k in range(4)], i, a0 + 0.5 * da)
            k4 = f([x[k] + h * k3[k] for k in range(4)], i, a0 + da)
            x = [x[k] + h / 6 * (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]) for k in range(4)]
    w3 = np.einsum("ij,ij->i", c["J3"], w)
    return q, w, w3


def rigido(c, tau):
    """Predicción de la cinemática pura (robot rígido que sigue el comando con un retardo)."""
    t = c["t"]
    v1 = np.interp(t - tau, t, c["v1c"])
    v2 = np.interp(t - tau, t, c["v2c"])
    return c["J3"][:, 0] * v1 + c["J3"][:, 1] * v2
