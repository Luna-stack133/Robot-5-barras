"""
Etapa 4 — Simulación en lazo cerrado de la corrección activa con la IMU (antes de tocar el robot).

Planta:   modelo identificado (M(q), K, B, ff, τ) + opcional: error de parámetros y fricción de Coulomb.
Sensores: gz y acelerómetro generados con el estado verdadero + ruido medido en reposo.
Estimador: el EKF de ekf5b.py (o el estado verdadero, para ver el límite ideal).
Ley:      Δq = −G · K⁻¹ · D · (ŵ(t+Tp) − q̇_c(t+Tp−τ))     (amortiguación a través de la referencia)
          D_i = 2·α·√(K_i·M_ii)  ·  ŵ(t+Tp): el EKF predice Tp hacia adelante con el modelo
          |Δq| ≤ Δmax (recorte de seguridad)
Retardos: lectura de la IMU + planificador del firmware (d_io) y el retardo τ del servo (en el modelo).

Uso:  python lazo_sim.py <corrida con el comando> [opciones]  (ver --help)
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.linalg import expm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ekf5b as E  # noqa: E402
import lqr5b  # noqa: E402
import modelo5b as m  # noqa: E402
import video_real as vr  # noqa: E402

DT = 0.01                    # periodo de la IMU y del EKF
SIG_GZ = math.radians(0.15)  # ruido del giroscopio en reposo
SIG_ACC = 0.03               # ruido del acelerómetro en reposo (m/s²)


def preparar_uniforme(carpeta, recorte=None):
    d = m.leer_corrida(carpeta)
    t0, t1 = d["t"][0], d["t"][-1]
    if recorte:
        t1 = min(t1, t0 + recorte)
    t = np.arange(t0, t1, DT)
    c = {"t": t, "q1c": np.interp(t, d["t"], d["q1c"]), "q2c": np.interp(t, d["t"], d["q2c"]),
         "gz": np.zeros_like(t), "ax": np.zeros_like(t), "ay": np.zeros_like(t), "info": d["info"],
         "nombre": d["nombre"]}
    return m.preparar(c)


def kb(p):
    return (np.diag([p.get("K1", p.get("K")), p.get("K2", p.get("K"))]),
            np.diag([p.get("B1", p.get("B")), p.get("B2", p.get("B"))]))


def simular_lazo(c, p_mod, p_real, gp, a):
    """Devuelve q, w verdaderos (n×2), Δq aplicado, estimaciones."""
    t = c["t"]
    n = len(t)
    Kr, Br = kb(p_real)
    Km, Bm = kb(p_mod)
    tau_r, ff_r = p_real["tau"], p_real["ff"]
    tau_m = p_mod["tau"]
    n_io = int(round(a.d_io / DT))
    Delta = np.zeros((n, 2))                  # corrección calculada en cada paso
    q = np.zeros((n, 2))
    w = np.zeros((n, 2))
    X_est = np.zeros((n, 7))
    xr = np.array([c["q1c"][0], c["q2c"][0], 0.0, 0.0])
    psi = math.radians(a.psi)
    rng = np.random.default_rng(1)

    def ref_aplicada(tt, Dl):
        """Referencia que recibe el driver en el tiempo tt: comando + corrección (ya con d_io), y su derivada."""
        qc = np.array([np.interp(tt, t, c["q1c"]), np.interp(tt, t, c["q2c"])])
        vc = np.array([np.interp(tt, t, c["v1c"]), np.interp(tt, t, c["v2c"])])
        k = (tt - t[0]) / DT - n_io
        k0 = int(math.floor(k))
        if k0 < 0:
            return qc, vc
        k0 = min(k0, n - 2)
        fr = min(max(k - k0, 0.0), 1.0)
        dq = Dl[k0] + fr * (Dl[k0 + 1] - Dl[k0])
        ddq = (Dl[k0 + 1] - Dl[k0]) / DT
        return qc + dq, vc + ddq

    def deriv_real(x, i, tt):
        qq, ww = x[:2], x[2:]
        r, rv = ref_aplicada(tt - tau_r, Delta)
        M, D1, D2 = c["M"][i], c["dM1"][i], c["dM2"][i]
        mdot = D1 * ww[0] + D2 * ww[1]
        g = 0.5 * np.array([ww @ D1 @ ww, ww @ D2 @ ww])
        tq = Kr @ (r - qq) + Br @ (ff_r * rv - ww) - (mdot @ ww - g)
        if a.friccion > 0:
            tq = tq - a.friccion * np.tanh(ww / 0.02)
        return np.r_[ww, np.linalg.solve(M, tq)]

    # EKF (modelo del controlador): la referencia que "cree" el EKF incluye la corrección que él mismo aplicó
    ekf = E.EKF(c, p_mod, gp, psi, usar_acc=not a.sin_acc,
                sig={"gz": math.radians(a.r_gz), "acc": a.r_acc})
    s = ekf.s
    xe = np.array([c["q1c"][0], c["q2c"][0], 0, 0, 0, 0, 0.0])
    P = np.diag([1e-6, 1e-6, 1e-4, 1e-4, s["sd_d"] ** 2, s["sd_d"] ** 2, 1e-4])
    nz = 1 if a.sin_acc else 3
    Rm = np.diag([s["gz"] ** 2] + ([s["acc"] ** 2] * 2 if nz == 3 else []))
    Mdiag = np.array([[c["M"][i][0, 0], c["M"][i][1, 1]] for i in range(n)])
    cache_F = {}
    lav_lp = np.zeros(2)
    lav_out = np.zeros(2)

    for i in range(n):
        tt = t[i]
        # ---- planta: integrar de t[i-1] a t[i]
        if i > 0:
            h = DT / 4
            for k in range(4):
                ts = t[i - 1] + k * h
                k1 = deriv_real(xr, i - 1, ts)
                k2 = deriv_real(xr + 0.5 * h * k1, i - 1, ts + 0.5 * h)
                k3 = deriv_real(xr + 0.5 * h * k2, i - 1, ts + 0.5 * h)
                k4 = deriv_real(xr + h * k3, i - 1, ts + h)
                xr = xr + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        q[i], w[i] = xr[:2], xr[2:]
        # ---- mediciones
        wd = deriv_real(xr, i, tt)[2:]
        acc, J = E.acel_sensor(xr[:2], xr[2:], wd)
        f = E.sensor(*xr[:2])[1]
        gz = m.K_GIRO * (J[2] @ xr[2:]) + rng.normal(0, SIG_GZ)
        ang = -(f + psi)
        R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        al = R @ acc + E.g_eslabon(f, gp) + rng.normal(0, SIG_ACC, 2) + np.array(a.sesgo_acc)
        z = np.array([gz, al[0], al[1]])[:nz]
        # ---- estimador
        if a.ideal:
            xe = np.r_[xr, 0, 0, 0]
        else:
            # referencia que conoce el EKF (comando + su propia corrección, con su τ)
            for j in (i, max(i - 1, 0)):
                ekf.r[j], rv = ref_aplicada(t[j] - tau_m, Delta)
                ekf.rd[j] = p_mod["ff"] * rv
            if i > 0:
                hh = DT / 2
                for _ in range(2):
                    def fx(xx):
                        return np.r_[xx[2:4], ekf.deriv(xx, i - 1), -xx[4:6] / s["tau_d"], 0]
                    k1 = fx(xe)
                    k2 = fx(xe + 0.5 * hh * k1)
                    k3 = fx(xe + 0.5 * hh * k2)
                    k4 = fx(xe + hh * k3)
                    xe = xe + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
                Minv = np.linalg.inv(c["M"][i - 1])
                A = np.zeros((7, 7))
                A[0:2, 2:4] = np.eye(2)
                A[2:4, 0:2] = -Minv @ Km
                A[2:4, 2:4] = -Minv @ Bm
                A[2:4, 4:6] = Minv
                A[4:6, 4:6] = -np.eye(2) / s["tau_d"]
                Phi = expm(A * DT)
                qd = 2 * s["sd_d"] ** 2 / s["tau_d"]
                Q = np.diag([s["q"] ** 2, s["q"] ** 2, s["w"] ** 2, s["w"] ** 2, qd, qd, s["b"] ** 2]) * DT
                P = Phi @ P @ Phi.T + Q
            hx = ekf.medir(xe, i)[:nz]
            H = np.zeros((nz, 7))
            for j in range(7):
                e = 1e-6 if j < 4 else 1e-4
                dx = np.zeros(7)
                dx[j] = e
                H[:, j] = (ekf.medir(xe + dx, i)[:nz] - ekf.medir(xe - dx, i)[:nz]) / (2 * e)
            S = H @ P @ H.T + Rm
            Kg = P @ H.T @ np.linalg.inv(S)
            xe = xe + Kg @ (z - hx)
            P = (np.eye(7) - Kg @ H) @ P
        X_est[i] = xe
        # ---- ley de corrección
        if a.ley == "lqr":
            qkey = (round(c["q1c"][i], 2), round(c["q2c"][i], 2))
            if qkey not in cache_F:
                cache_F[qkey] = lqr5b.ganancia(c["M"][i], p_mod, td=a.d_io + tau_m, ep=a.ep, ev=a.ev, du=a.du)[:2]
            F, N = cache_F[qkey]
            tq = tt - tau_m
            qd = np.array([np.interp(tq, t, c["q1c"]), np.interp(tq, t, c["q2c"])])
            vd = np.array([np.interp(tq, t, c["v1c"]), np.interp(tq, t, c["v2c"])])
            eq = xe[0:2] - qd
            if a.lavado > 0:          # filtro de lavado: quita la parte lenta (sesgo) del error de posición
                lav_lp[:] = lav_lp + (DT / a.lavado) * (eq - lav_lp)
                eq = eq - lav_lp
            z = np.r_[eq, xe[2:4] - vd] if not a.solo_vel else np.r_[0, 0, xe[2:4] - vd]
            hist = [Delta[i - j] if i - j >= 0 else np.zeros(2) for j in range(1, N + 1)]
            z = np.r_[z, np.concatenate(hist)]
            dlt = -F @ z
            if a.lavado_salida > 0:   # lavado de la salida: la corrección no puede sostener un valor constante
                lav_out[:] = lav_out + (DT / a.lavado_salida) * (dlt - lav_out)
                dlt = dlt - lav_out
            Delta[i] = np.clip(dlt, -math.radians(a.dmax), math.radians(a.dmax))
        elif a.alfa > 0:
            # predicción Tp hacia adelante con el modelo (velocidad ŵ(t+Tp))
            xp = xe.copy()
            npred = int(round(a.tp / DT))
            for k in range(npred):
                j = min(i + k, n - 1)
                if not a.ideal:
                    ekf.r[j], rv = ref_aplicada(t[j] - tau_m, Delta)
                    ekf.rd[j] = p_mod["ff"] * rv
                    xp = xp + DT * np.r_[xp[2:4], ekf.deriv(xp, j), -xp[4:6] / s["tau_d"], 0]
                else:
                    xp4 = deriv_real(xp[:4], j, t[j])
                    xp = np.r_[xp[:4] + DT * xp4, xp[4:]]
            tq = tt + a.tp - tau_m
            v_des = np.array([np.interp(tq, t, c["v1c"]), np.interp(tq, t, c["v2c"])])
            Dd = 2 * a.alfa * np.sqrt(np.diag(Km) * Mdiag[i])
            ew = xp[2:4] - v_des
            ew = np.sign(ew) * np.maximum(np.abs(ew) - math.radians(a.zona), 0.0)   # zona muerta
            dlt = -(Dd / np.diag(Km)) * ew
            if a.lavado_salida > 0:
                lav_out[:] = lav_out + (DT / a.lavado_salida) * (dlt - lav_out)
                dlt = dlt - lav_out
            dlt = np.clip(dlt, -math.radians(a.dmax), math.radians(a.dmax))
            Delta[i] = dlt
    return q, w, Delta, X_est


def metricas(c, q):
    t = c["t"]
    P = np.array([m.fk(a_, b_)[2] for a_, b_ in zip(c["q1c"], c["q2c"])]) * 1000
    Pr = np.array([m.fk(a_, b_)[2] for a_, b_ in q]) * 1000
    mov = np.hypot(*np.gradient(P, axis=0).T) > 1e-4
    # llegadas: fin de cada movimiento seguido de ≥0.5 s quieto (o el final)
    out = []
    i = 1
    while i < len(t):
        if mov[i - 1] and not mov[i]:
            j = i
            while j < len(t) and not mov[j]:
                j += 1
            if t[min(j, len(t) - 1)] - t[i] >= 0.5:
                s = slice(i, j)
                dev = np.hypot(*(Pr[s] - P[i]).T)
                fuera = np.where(dev > 0.5)[0]
                ts = 0.0 if len(fuera) == 0 else (t[s][fuera[-1]] - t[i] if fuera[-1] < len(dev) - 2 else float("nan"))
                fuera1 = np.where(dev > 1.0)[0]
                ts1 = 0.0 if len(fuera1) == 0 else (t[s][fuera1[-1]] - t[i] if fuera1[-1] < len(dev) - 2 else float("nan"))
                out.append((dev.max(), ts1, ts, float(np.median(dev[-20:]))))
            i = j
        else:
            i += 1
    e = vr.dist_polilinea(Pr[mov][::2], P[mov])
    A = np.array(out) if out else np.full((1, 4), np.nan)
    return {"contorno_rms": float(np.sqrt(np.mean(e ** 2))), "contorno_max": float(e.max()),
            "llegadas": len(out), "dev_max": float(np.nanmedian(A[:, 0])), "t_1mm": float(np.nanmedian(A[:, 1])),
            "t_05mm": float(np.nanmedian(A[:, 2])),
            "error_reposo": float(np.nanmedian(A[:, 3]))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corrida")
    ap.add_argument("--modelo", required=True)
    ap.add_argument("--grav", required=True)
    ap.add_argument("--psi", type=float, default=-177.0)
    ap.add_argument("--alfa", type=float, default=0.0, help="amortiguamiento agregado (fracción del crítico)")
    ap.add_argument("--ley", choices=["alfa", "lqr"], default="alfa")
    ap.add_argument("--ep", type=float, default=0.5, help="LQR: error de posición tolerable [°]")
    ap.add_argument("--ev", type=float, default=5.0, help="LQR: error de velocidad tolerable [°/s]")
    ap.add_argument("--du", type=float, default=1.0, help="LQR: corrección tolerable [°]")
    ap.add_argument("--solo-vel", action="store_true", help="LQR sin realimentar el error de posición")
    ap.add_argument("--lavado", type=float, default=0.0, help="LQR: constante del filtro de lavado del error de posición [s]")
    ap.add_argument("--lavado-salida", type=float, default=0.0, help="constante del filtro de lavado de la corrección [s]")
    ap.add_argument("--sesgo-acc", type=float, nargs=2, default=[0.0, 0.0], help="sesgo del acelerómetro real [m/s²]")
    ap.add_argument("--tp", type=float, default=0.0, help="horizonte de predicción [s]")
    ap.add_argument("--d-io", type=float, default=0.020, help="retardo lectura IMU + planificador [s]")
    ap.add_argument("--dmax", type=float, default=0.5, help="recorte de la corrección [°]")
    ap.add_argument("--zona", type=float, default=0.0, help="zona muerta del error de velocidad [°/s]")
    ap.add_argument("--ideal", action="store_true", help="usar el estado verdadero en vez del EKF")
    ap.add_argument("--sin-acc", action="store_true")
    ap.add_argument("--r-gz", type=float, default=0.8)
    ap.add_argument("--r-acc", type=float, default=0.35)
    ap.add_argument("--err-kb", type=float, default=0.0, help="error relativo de K y B de la planta real")
    ap.add_argument("--err-tau", type=float, default=0.0, help="error del retardo de la planta real [s]")
    ap.add_argument("--friccion", type=float, default=0.0, help="fricción de Coulomb en la planta [N·m]")
    ap.add_argument("--recorte", type=float, default=None, help="simular solo los primeros N s")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    with open(a.modelo, encoding="utf-8") as f:
        p = json.load(f)["parametros"]
    gp = json.load(open(a.grav))
    c = preparar_uniforme(a.corrida, a.recorte)
    pr = dict(p)
    for k in ("K1", "K2", "B1", "B2", "K", "B"):
        if k in pr:
            pr[k] = pr[k] * (1 + a.err_kb)
    pr["tau"] = p["tau"] + a.err_tau
    q, w, Dl, X = simular_lazo(c, p, pr, gp, a)
    r = metricas(c, q)
    r.update({"ley": a.ley, "lavado": a.lavado, "lavado_salida": a.lavado_salida, "solo_vel": a.solo_vel, "sesgo_acc": a.sesgo_acc, "alfa": a.alfa, "tp": a.tp, "ideal": a.ideal, "acc": not a.sin_acc, "d_io": a.d_io,
              "err_kb": a.err_kb, "err_tau": a.err_tau, "friccion": a.friccion,
              "delta_max_deg": float(np.degrees(np.abs(Dl).max())),
              "recortado_pct": float(100 * np.mean(np.abs(Dl) >= math.radians(a.dmax) * 0.999))})
    if a.ley == "lqr":
        print("LQR ep=%.2f° ev=%.1f°/s du=%.1f°%s" % (a.ep, a.ev, a.du, " (solo vel)" if a.solo_vel else ""), end=" ")
    print("α=%.2f Tp=%3.0f ms %s %s | contorno RMS %.2f máx %.2f mm | llegada: desv %.2f mm, ±1 mm %.2f s, "
          "±0.5 mm %.2f s, error en reposo %.2f mm | Δ máx %.2f° (recorte %.0f %%)"
          % (a.alfa, a.tp * 1000, "ideal" if a.ideal else "EKF  ", "gz+acc" if not a.sin_acc else "gz",
             r["contorno_rms"], r["contorno_max"], r["dev_max"], r["t_1mm"], r["t_05mm"], r["error_reposo"], r["delta_max_deg"],
             r["recortado_pct"]))
    if a.json:
        with open(a.json, "a") as f:
            f.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    main()
