"""
EKF offline del robot de 5 barras con la IMU del distal derecho (etapa 3, primera parte).

Estados (7):  q1, q2 [rad]  ·  w1, w2 [rad/s]  ·  d1, d2 [N·m] (par de perturbación)  ·  b [rad/s] (sesgo del giro)
Entradas:     ángulos comandados por la ESP32 (pulsos emitidos), con el retardo τ del modelo identificado
Modelo:       M(q)·w' = K·(q_c(t−τ) − q) + B·(ff·q_c'(t−τ) − w) − C(q,w)·w + d
              d' = ruido (paseo aleatorio): la fricción, el pegue-deslice y la rigidez no lineal que
              el modelo no explica se estiman como un par desconocido en cada motor.
Mediciones:   gz = k·φ3'(q, w) + b                       (giroscopio, rad/s)
              ax, ay = aceleración del chip en ejes del eslabón + gravedad por la inclinación (m/s²)

Uso:
  python ekf5b.py grav <corridas con reposos...> --salida g.json        ajusta la gravedad/inclinación
  python ekf5b.py run <corrida> --modelo modelo.json --grav g.json [--video real.csv] [--sin-acc]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.linalg import expm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402

R_IMU = 0.0335          # m: del eje del efector al chip, sobre el distal derecho
G = 9.80665


# ------------------------------------------------------------------ geometría del sensor
def sensor(q1, q2):
    """Posición del chip (m) y ángulo del distal derecho φ3 (rad)."""
    _, B, P = m.fk(q1, q2)
    f = math.atan2(P[1] - B[1], P[0] - B[0])
    return np.array([P[0] - R_IMU * math.cos(f), P[1] - R_IMU * math.sin(f)]), f


def jac_sensor(q, e=1e-6):
    """J (3×2): derivadas de (x_s, y_s, φ3) respecto a q."""
    J = np.zeros((3, 2))
    for j in range(2):
        d = np.zeros(2)
        d[j] = e
        sp, fp = sensor(*(q + d))
        sm, fm = sensor(*(q - d))
        J[:2, j] = (sp - sm) / (2 * e)
        J[2, j] = (fp - fm) / (2 * e)
    return J


def acel_sensor(q, w, wd, e=1e-5):
    """Aceleración del chip (m/s², marco del robot) = J·q'' + (dJ/dt)·q'."""
    J = jac_sensor(q)
    n = np.linalg.norm(w)
    if n > 1e-9:
        u = w / n
        Jd = (jac_sensor(q + u * e) - jac_sensor(q - u * e)) / (2 * e) * n
    else:
        Jd = np.zeros((3, 2))
    return J[:2] @ wd + Jd[:2] @ w, J


# ------------------------------------------------------------------ gravedad / inclinación
def g_eslabon(f, gp):
    """Componente de la gravedad en los ejes x, y del chip según el ángulo del distal (ajuste empírico)."""
    cx, cy, p1, p2 = gp["cx"], gp["cy"], gp["p1"], gp["p2"]
    return np.array([cx + p1 * math.cos(f) + p2 * math.sin(f), cy - p1 * math.sin(f) + p2 * math.cos(f)])


def ajustar_gravedad(carpetas):
    """Usa los reposos (comando quieto ≥1 s, desde 0.6 s después de parar) para ajustar cx, cy, p1, p2."""
    filas = []
    for c in carpetas:
        d = m.leer_corrida(c)
        t, q1, q2 = d["t"], d["q1c"], d["q2c"]
        mov = np.r_[True, (np.abs(np.diff(q1)) + np.abs(np.diff(q2))) > 1e-9]
        i0 = None
        for i in range(len(t)):
            if not mov[i] and i0 is None:
                i0 = i
            if (mov[i] or i == len(t) - 1) and i0 is not None:
                sel = (t >= t[i0] + 0.6) & (t <= t[i - 1])
                if sel.sum() >= 20:
                    _, f = sensor(q1[i0], q2[i0])
                    filas.append((f, np.median(d["ax"][sel]), np.median(d["ay"][sel])))
                i0 = None
    F = np.array(filas)
    f, ax, ay = F[:, 0], F[:, 1], F[:, 2]
    A = np.zeros((2 * len(f), 4))
    A[0::2] = np.column_stack([np.ones_like(f), np.zeros_like(f), np.cos(f), np.sin(f)])
    A[1::2] = np.column_stack([np.zeros_like(f), np.ones_like(f), -np.sin(f), np.cos(f)])
    y = np.empty(2 * len(f))
    y[0::2], y[1::2] = ax, ay
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ sol
    return {"cx": sol[0], "cy": sol[1], "p1": sol[2], "p2": sol[3], "n_reposos": len(f),
            "residuo_rms": float(np.sqrt(np.mean(res ** 2)))}


# ------------------------------------------------------------------ EKF
class EKF:
    def __init__(self, c, p, gp, psi, usar_acc=True, sig=None):
        self.c, self.p, self.gp, self.psi, self.usar_acc = c, p, gp, psi, usar_acc
        # d: proceso de Gauss-Markov (constante de tiempo TAU_D) con desviación estacionaria SD_D [N·m]
        s = {"q": 1e-5, "w": 0.05, "sd_d": 0.03, "tau_d": 0.3, "b": 0.002, "gz": math.radians(0.8), "acc": 0.35}
        s.update(sig or {})
        self.s = s
        t = c["t"]
        self.K = np.diag([p.get("K1", p.get("K")), p.get("K2", p.get("K"))])
        self.B = np.diag([p.get("B1", p.get("B")), p.get("B2", p.get("B"))])
        td = t - p["tau"]
        self.r = np.column_stack([np.interp(td, t, c["q1c"]), np.interp(td, t, c["q2c"])])
        self.rd = p["ff"] * np.column_stack([np.interp(td, t, c["v1c"]), np.interp(td, t, c["v2c"])])

    def deriv(self, x, i):
        q, w, d = x[0:2], x[2:4], x[4:6]
        M, D1, D2 = self.c["M"][i], self.c["dM1"][i], self.c["dM2"][i]
        mdot = D1 * w[0] + D2 * w[1]
        g = 0.5 * np.array([w @ D1 @ w, w @ D2 @ w])
        cor = mdot @ w - g
        tau = self.K @ (self.r[i] - q) + self.B @ (self.rd[i] - w) - cor + d
        wd = np.linalg.solve(M, tau)
        return wd

    def medir(self, x, i):
        q, w = x[0:2], x[2:4]
        wd = self.deriv(x, i)
        a, J = acel_sensor(q, w, wd)
        f = sensor(*q)[1]
        gz = m.K_GIRO * (J[2] @ w) + x[6]
        if not self.usar_acc:
            return np.array([gz])
        # aceleración del chip en ejes del eslabón (giro −φ) y montaje ψ, más la gravedad
        ang = -(f + self.psi)
        R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        al = R @ a + g_eslabon(f, self.gp)
        return np.array([gz, al[0], al[1]])

    def correr(self):
        c = self.c
        t = c["t"]
        n = len(t)
        x = np.array([self.r[0, 0], self.r[0, 1], 0, 0, 0, 0, 0.0])
        P = np.diag([1e-6, 1e-6, 1e-4, 1e-4, self.s["sd_d"] ** 2, self.s["sd_d"] ** 2, 1e-4])
        s = self.s
        nz = 3 if self.usar_acc else 1
        Rm = np.diag([s["gz"] ** 2] + ([s["acc"] ** 2] * 2 if self.usar_acc else []))
        z_all = np.column_stack([c["gz"], c["ax"], c["ay"]])[:, :nz]
        out = np.zeros((n, 7))
        inov = np.zeros((n, nz))
        for i in range(n):
            if i > 0:
                dt = t[i] - t[i - 1]
                # predicción: RK4 del modelo (2 subpasos)
                h = dt / 2
                for _ in range(2):
                    def f(xx):
                        return np.r_[xx[2:4], self.deriv(xx, i - 1), -xx[4:6] / s["tau_d"], 0]
                    k1 = f(x)
                    k2 = f(x + 0.5 * h * k1)
                    k3 = f(x + 0.5 * h * k2)
                    k4 = f(x + h * k3)
                    x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
                # covarianza: modelo linealizado (sin Coriolis) discretizado exactamente
                Minv = np.linalg.inv(c["M"][i - 1])
                A = np.zeros((7, 7))
                A[0:2, 2:4] = np.eye(2)
                A[2:4, 0:2] = -Minv @ self.K
                A[2:4, 2:4] = -Minv @ self.B
                A[2:4, 4:6] = Minv
                A[4:6, 4:6] = -np.eye(2) / s["tau_d"]
                Phi = expm(A * dt)
                qd = 2 * s["sd_d"] ** 2 / s["tau_d"]          # intensidad que da la desviación estacionaria sd_d
                Q = np.diag([s["q"] ** 2, s["q"] ** 2, s["w"] ** 2, s["w"] ** 2, qd, qd, s["b"] ** 2]) * dt
                P = Phi @ P @ Phi.T + Q
            # corrección
            z = z_all[i]
            hx = self.medir(x, i)
            H = np.zeros((nz, 7))
            for j in range(7):
                e = 1e-6 if j < 4 else 1e-4
                dx = np.zeros(7)
                dx[j] = e
                H[:, j] = (self.medir(x + dx, i) - self.medir(x - dx, i)) / (2 * e)
            y = z - hx
            S = H @ P @ H.T + Rm
            Kg = P @ H.T @ np.linalg.inv(S)
            x = x + Kg @ y
            P = (np.eye(7) - Kg @ H) @ P
            out[i] = x
            inov[i] = y
        return out, inov


# ------------------------------------------------------------------ evaluación contra el video
def alinear_gz(tv, Pv, t, P_cmd, gz_dps, lag=0.06):
    """Giro/reflexión + desplazamiento del video que hacen que el giro del distal derecho visto en el
    video coincida con el giroscopio."""
    import video_real as vr
    Pc = np.column_stack([np.interp(tv - lag, t, P_cmd[:, k]) for k in range(2)])
    g = np.interp(tv, t, gz_dps)
    s = (tv > t[0] + 0.1) & (tv < t[-1] - 0.1)

    def prueba(th, refl):
        Q = Pv * [1, -1] if refl else Pv
        c_, sn = math.cos(th), math.sin(th)
        A = np.column_stack([c_ * Q[:, 0] - sn * Q[:, 1], sn * Q[:, 0] + c_ * Q[:, 1]])
        A = A - A.mean(0) + Pc.mean(0)
        q = np.array([vr.ik(x / 1000, y / 1000) for x, y in A])
        if not np.all(np.isfinite(q)):
            return -1, A
        w = np.degrees(np.gradient(vr.phi3(q[:, 0], q[:, 1]), tv))
        w = np.convolve(w, np.ones(3) / 3, "same")
        return np.corrcoef(w[s], g[s])[0, 1], A
    best = None
    for refl in (False, True):
        for th in np.radians(np.arange(0, 360, 10)):
            r, A = prueba(th, refl)
            if best is None or r > best[0]:
                best = (r, th, refl, A)
    r, th, refl, A = best
    for step in (2, 0.5, 0.1):
        th0 = th
        for dth in np.radians(np.arange(-5 * step, 5.01 * step, step)):
            r2, A2 = prueba(th0 + dth, refl)
            if r2 > r:
                r, A, th = r2, A2, th0 + dth
    return A, r


def efector(q):
    return np.array([m.fk(a, b)[2] for a, b in q]) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("accion", choices=["grav", "run"])
    ap.add_argument("corridas", nargs="+")
    ap.add_argument("--modelo")
    ap.add_argument("--grav")
    ap.add_argument("--psi", type=float, default=-177.0, help="ángulo de montaje del chip [°]")
    ap.add_argument("--video")
    ap.add_argument("--sin-acc", action="store_true")
    ap.add_argument("--salida", default=".")
    a = ap.parse_args()
    if a.accion == "grav":
        g = ajustar_gravedad(a.corridas)
        print(json.dumps(g, indent=2))
        with open(os.path.join(a.salida, "gravedad.json"), "w") as f:
            json.dump(g, f, indent=2)
        return
    with open(a.modelo, encoding="utf-8") as f:
        p = json.load(f)["parametros"]
    gp = json.load(open(a.grav)) if a.grav else {"cx": 0, "cy": 0, "p1": 0, "p2": 0}
    c = m.preparar(m.leer_corrida(a.corridas[0]))
    t = c["t"]
    ekf = EKF(c, p, gp, math.radians(a.psi), usar_acc=not a.sin_acc)
    X, inov = ekf.correr()
    qm, wm, _ = m.simular(c, p)
    res = {"corrida": os.path.basename(a.corridas[0]), "con_acc": not a.sin_acc,
           "inov_gz_rms_dps": float(np.degrees(np.sqrt(np.mean(inov[:, 0] ** 2))))}
    if a.video:
        v = np.genfromtxt(a.video, delimiter=",", names=True)
        tv = v["t_s"]
        Pv = np.column_stack([v["x_mm"], v["y_mm"]])
        sel = (tv >= t[0]) & (tv <= t[-1])
        tv, Pv = tv[sel], Pv[sel]
        P_cmd = efector(np.column_stack([c["q1c"], c["q2c"]]))
        # El video se sincronizó con la rapidez del COMANDO, así que su reloj quedó corrido el retardo del robot.
        # Se re-sincroniza y se alinea con el GIROSCOPIO (mide el movimiento real): giro/reflexión +
        # desplazamiento que maximizan la correlación del giro del distal visto en el video con gz.
        import video_real as vr
        gz_dps = np.degrees(c["gz"]) / m.K_GIRO
        Pv0 = Pv.copy()
        dt_v = 0.0
        for _ in range(2):
            Pv, rr = alinear_gz(tv + dt_v, Pv0, t, P_cmd, gz_dps)
            q_v = np.array([vr.ik(x / 1000, y / 1000) for x, y in Pv])
            w_v = np.degrees(np.gradient(vr.phi3(q_v[:, 0], q_v[:, 1]), tv))
            mejor = max(((np.corrcoef(w_v[5:-5], np.interp(tv[5:-5] + dt_v + dd, t, gz_dps))[0, 1], dd)
                         for dd in np.arange(-0.15, 0.151, 0.005)))
            dt_v += mejor[1]
        tv = tv + dt_v
        res["video_resinc_s"] = float(dt_v)
        res["video_corr_gz"] = float(mejor[0])
        print("video re-sincronizado con el giroscopio: %+.3f s, correlación %.3f" % (dt_v, mejor[0]))
        sel = (tv >= t[0]) & (tv <= t[-1])
        tv, Pv = tv[sel], Pv[sel]
        P_mod = efector(qm)
        P_ekf = efector(X[:, :2])
        interp = lambda Pa: np.column_stack([np.interp(tv, t, Pa[:, k]) for k in range(2)])
        Pc, Pm, Pe = interp(P_cmd), interp(P_mod), interp(P_ekf)
        # EKF calculado a bordo por el firmware 1.3 (columnas q*_ekf_deg de imu.csv), si existe
        dfw = np.genfromtxt(os.path.join(a.corridas[0], "imu.csv"), delimiter=",", names=True)
        Pf = None
        if "q1_ekf_deg" in dfw.dtype.names:
            okf = dfw["ekf_valido"] > 0
            qf = np.radians(np.column_stack([dfw["q1_ekf_deg"][okf], dfw["q2_ekf_deg"][okf]]))
            Pf = np.column_stack([np.interp(tv, dfw["t_s"][okf], efector(qf)[:, k]) for k in range(2)])
        # el video está alineado sobre el contorno: se quita el desfase medio (sesgo de alineación ~1–2 mm)
        # para comparar la parte DINÁMICA (retardo, vibración), que es lo que debe estimar el EKF
        def err(Pa):
            dlt = Pv - Pa
            dlt = dlt - np.median(dlt, axis=0)
            e = np.hypot(*dlt.T)
            return float(np.sqrt(np.mean(e ** 2))), float(np.percentile(e, 95)), e
        for nombre, Pa in (("comando", Pc), ("modelo", Pm), ("ekf", Pe)) + ((("esp32", Pf),) if Pf is not None else ()):
            r_, p95, e = err(Pa)
            res["err_%s_rms_mm" % nombre] = r_
            res["err_%s_p95_mm" % nombre] = p95
        if Pf is not None:
            print("  EKF de la ESP32 frente al video: RMS %.2f mm (p95 %.2f)" % (res["err_esp32_rms_mm"], res["err_esp32_p95_mm"]))
        print("%s %s | posición del efector frente al video (sin el desfase medio): comando RMS %.2f mm | "
              "modelo %.2f | EKF %.2f mm  (p95: %.2f / %.2f / %.2f)  | innovación gz %.2f °/s"
              % (res["corrida"], "gz+acc" if not a.sin_acc else "gz   ", res["err_comando_rms_mm"],
                 res["err_modelo_rms_mm"], res["err_ekf_rms_mm"], res["err_comando_p95_mm"],
                 res["err_modelo_p95_mm"], res["err_ekf_p95_mm"], res["inov_gz_rms_dps"]))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        for k, lab in enumerate("xy"):
            ref = Pc[:, k]
            off = np.median(Pv[:, k] - ref)
            axs[k].plot(tv, Pv[:, k] - ref - off, color="#D9480F", lw=1.1, label="real (video) − comando")
            axs[k].plot(tv, Pm[:, k] - ref, color="#8794A1", lw=0.9, ls="--", label="modelo − comando")
            axs[k].plot(tv, Pe[:, k] - ref, color="#0E7C86", lw=1.0, label="EKF (PC) − comando")
            if Pf is not None:
                axs[k].plot(tv, Pf[:, k] - ref, color="#7048E8", lw=0.9, ls=":", label="EKF (ESP32) − comando")
            axs[k].set_ylabel("%s [mm]" % lab)
            axs[k].grid(alpha=0.3)
        axs[0].legend(loc="upper right", fontsize=8)
        axs[0].set_title("%s · EKF (%s): RMS %.2f mm · modelo %.2f · comando %.2f"
                         % (res["corrida"], "gz + acelerómetro" if not a.sin_acc else "solo gz",
                            res["err_ekf_rms_mm"], res["err_modelo_rms_mm"], res["err_comando_rms_mm"]))
        axs[1].set_xlabel("tiempo [s]")
        fig.tight_layout()
        os.makedirs(a.salida, exist_ok=True)
        fig.savefig(os.path.join(a.salida, "ekf_%s_%s.png" % (res["corrida"][11:19], "acc" if not a.sin_acc else "gz")),
                    dpi=100)
    else:
        print(res)
    os.makedirs(a.salida, exist_ok=True)
    with open(os.path.join(a.salida, "ekf_%s_%s.json" % (res["corrida"], "acc" if not a.sin_acc else "gz")), "w") as f:
        json.dump(res, f, indent=2)


if __name__ == "__main__":
    main()
