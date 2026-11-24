"""
Posición REAL del efector a partir del video, SIN usar el modelo dinámico.

Entrada: el CSV de video_seguir.py (marcador con el movimiento de la cámara ya descontado) y la
carpeta de la corrida (imu.csv con el comando y el giroscopio).

Pasos:
 1. Rectificación con el propio marcador: se busca la transformación imagen → plano (perspectiva de
    la cámara) que hace que el cuadro mida 20 × 20 mm en todos los cuadros del video.
 2. Alineación al marco del robot: giro + desplazamiento (3 parámetros) que mejor llevan la
    trayectoria del video sobre el contorno ideal de la figura. No usa tiempos ni dinámica.
 3. Cinemática inversa: posición real → ángulos reales de los motores y del distal derecho.
 4. Sincronización: se correlaciona el giro del distal derecho visto en el video con el giroscopio.

Salida: <corrida>/video_real.csv  (t_s de la corrida, x_mm, y_mm, q1_deg, q2_deg)

Uso:  python video_real.py <video_marcador.csv> <carpeta de la corrida>
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo5b as m  # noqa: E402

LADO_MM = 20.0


def ik(x, y, g=m.GEOM):
    """Cinemática inversa, modo 1 (codos afuera). x, y en metros → (q1, q2) en rad."""
    L1, L2, L3, L4, D = g["L1"], g["L2"], g["L3"], g["L4"], g["D"]
    d1 = math.hypot(x, y)
    d2 = math.hypot(x - D, y)
    c1 = (L1 * L1 + d1 * d1 - L2 * L2) / (2 * L1 * d1)
    c2 = (L4 * L4 + d2 * d2 - L3 * L3) / (2 * L4 * d2)
    if abs(c1) > 1 or abs(c2) > 1:
        return float("nan"), float("nan")
    return math.atan2(y, x) + math.acos(c1), math.atan2(y, x - D) - math.acos(c2)


def phi3(q1, q2):
    """Ángulo del distal derecho (rad) para arreglos de ángulos de motor."""
    out = np.empty(len(q1))
    for i, (a, b) in enumerate(zip(q1, q2)):
        if not (np.isfinite(a) and np.isfinite(b)):
            out[i] = np.nan
            continue
        _, B, P = m.fk(a, b)
        out[i] = math.atan2(P[1] - B[1], P[0] - B[0])
    return np.unwrap(out)


# ------------------------------------------------------------------ 1. rectificación con el marcador
def a_plano(p, xy):
    s, a, b, g, h = p
    x = (xy[..., 0] - 540.0) / 1000.0
    y = (xy[..., 1] - 960.0) / 1000.0
    w = 1 + g * x + h * y
    return np.stack([s * (x + a * y) / w, s * b * y / w], axis=-1)


def _res_cuadro(p, E):
    P = a_plano(p, E)
    r = [np.linalg.norm(P[:, i] - P[:, (i + 1) % 4], axis=1) - LADO_MM for i in range(4)]
    r.append((np.linalg.norm(P[:, 0] - P[:, 2], axis=1) - LADO_MM * math.sqrt(2)) / math.sqrt(2))
    r.append((np.linalg.norm(P[:, 1] - P[:, 3], axis=1) - LADO_MM * math.sqrt(2)) / math.sqrt(2))
    return np.concatenate(r)


def rectificar(v):
    E = np.stack([np.column_stack([v["f%dx" % k], v["f%dy" % k]]) for k in range(1, 5)], axis=1)
    c = np.column_stack([v["fcx"], v["fcy"]])
    t = v["t_s"]
    rap = np.hypot(np.gradient(c[:, 0], t), np.gradient(c[:, 1], t)) / 4.3
    sel = (rap < 150) & np.all(np.isfinite(E.reshape(len(E), -1)), axis=1)     # sin los cuadros más borrosos
    sol = least_squares(_res_cuadro, [1000 / 4.3, 0, 1, 0, 0], args=(E[sel],), loss="soft_l1", f_scale=0.3)
    rms = float(np.sqrt(np.mean(_res_cuadro(sol.x, E[sel]) ** 2)))
    return sol.x, a_plano(sol.x, c), rms


# ------------------------------------------------------------------ 2. alineación al marco del robot
def dist_polilinea(p, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(p))
    for i, x in enumerate(p):
        k = np.clip(((x - a) * ab).sum(1) / L2, 0, 1)
        out[i] = np.sqrt((((a + k[:, None] * ab) - x) ** 2).sum(1)).min()
    return out


def mover(par, P, espejo):
    th, tx, ty = par
    Q = P * [1, -1] if espejo else P
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([c * Q[:, 0] - s * Q[:, 1] + tx, s * Q[:, 0] + c * Q[:, 1] + ty])


def alinear(Pv, poly, dentro, tv, t, P_cmd, d0):
    """Giro + desplazamiento (y si hace falta espejo) que llevan el video sobre el contorno ideal.
    Un cuadrado tiene 8 simetrías: la correcta es la que además recorre la figura en el mismo orden que
    el comando (se prueba con el mejor desfase de tiempo). Eso solo sirve para elegir la simetría; el
    ajuste fino usa únicamente la distancia al contorno."""
    def orden(Pal):
        best = np.inf
        for d in np.arange(d0 - 2.0, d0 + 2.0, 0.05):
            tc = tv[dentro] - d
            ok = (tc >= t[0]) & (tc <= t[-1])
            if ok.sum() < 0.7 * dentro.sum():
                continue
            Pc = np.column_stack([np.interp(tc[ok], t, P_cmd[:, k]) for k in range(2)])
            best = min(best, float(np.mean(np.hypot(*(Pal[ok] - Pc).T))))
        return best
    cen_v = Pv[dentro].mean(axis=0)
    cen_p = poly.mean(axis=0)
    cands = []
    for espejo in (False, True):
        for th0 in np.radians(np.arange(0, 360, 15)):
            ini = mover([th0, 0, 0], cen_v[None, :], espejo)[0]
            x0 = [th0, cen_p[0] - ini[0], cen_p[1] - ini[1]]
            f = lambda par: dist_polilinea(mover(par, Pv[dentro][::3], espejo), poly)
            sol = least_squares(f, x0, loss="soft_l1", f_scale=3.0)
            if sol.cost > 3 * min([c[0] for c in cands] + [sol.cost]):
                continue
            cands.append((sol.cost, sol.x, espejo))
    cmin = min(c[0] for c in cands)
    cands = [c for c in cands if c[0] < 1.5 * cmin]            # las 8 simetrías dan casi el mismo costo
    _, par, espejo = min(cands, key=lambda c: c[0])
    f = lambda q: dist_polilinea(mover(q, Pv[dentro], espejo), poly)
    par = least_squares(f, par, loss="soft_l1", f_scale=3.0).x
    return par, espejo


def simetrias(P, centro):
    """Las 8 simetrías de un cuadrado alrededor de su centro (4 giros × con/sin espejo)."""
    out = []
    Q0 = P - centro
    for esp in (False, True):
        Q = Q0 * [-1, 1] if esp else Q0
        for k in range(4):
            c, s_ = round(math.cos(k * math.pi / 2)), round(math.sin(k * math.pi / 2))
            out.append((k, esp, np.column_stack([c * Q[:, 0] - s_ * Q[:, 1], s_ * Q[:, 0] + c * Q[:, 1]]) + centro))
    return out


# ------------------------------------------------------------------ todo junto
def reconstruir(video_csv, carpeta, verbose=True):
    v = np.genfromtxt(video_csv, delimiter=",", names=True)
    c = m.preparar(m.leer_corrida(carpeta))
    t = c["t"]
    P_cmd = np.array([m.fk(a, b)[2] for a, b in zip(c["q1c"], c["q2c"])]) * 1000.0
    G, Pv, rms_cuadro = rectificar(v)
    tv = v["t_s"]

    # tramo del video donde el efector recorre la figura (rapidez sostenida), con margen
    rap = np.hypot(np.gradient(Pv[:, 0], tv), np.gradient(Pv[:, 1], tv))
    rs = np.convolve(rap, np.ones(15) / 15, mode="same")
    dur = t[-1] - t[0]
    # la corrida es el intervalo de duración `dur` con más recorrido
    paso = np.gradient(tv) * rs
    acum = np.concatenate([[0], np.cumsum(paso)])
    mejor = (0, 0)
    for i in range(len(tv)):
        j = np.searchsorted(tv, tv[i] + dur)
        if j >= len(tv):
            j = len(tv) - 1
        if acum[j] - acum[i] > mejor[0]:
            mejor = (acum[j] - acum[i], i)
    i0 = mejor[1]
    dentro = (tv >= tv[i0]) & (tv <= tv[i0] + dur)

    # Un cuadrado tiene 8 simetrías: la correcta es la que hace coincidir el giro del distal derecho
    # visto en el video con el giroscopio (se prueban todas y se sincroniza cada una).
    gz = c["gz"] / m.K_GIRO
    rej = np.arange(tv[0], tv[-1], 0.002)
    mejor = None
    par, espejo = alinear(Pv, P_cmd, dentro, tv, t, P_cmd, tv[i0] - t[0])
    centro = 0.5 * (P_cmd.min(axis=0) + P_cmd.max(axis=0))
    for ksim, esp2, Pr in simetrias(mover(par, Pv, espejo), centro):
        q = np.array([ik(x / 1000.0, y / 1000.0) for x, y in Pr])
        if not np.all(np.isfinite(q)):
            continue
        f3 = phi3(q[:, 0], q[:, 1])
        w3v = np.gradient(f3, tv)                                  # rad/s, del video
        w3g = np.interp(rej, tv, np.convolve(w3v, np.ones(3) / 3, mode="same"))
        for d in np.arange(tv[0] - t[0] - 1.0, tv[-1] - t[-1] + 1.0, 0.004):
            a = np.interp(t + d, rej, w3g, left=np.nan, right=np.nan)
            ok = np.isfinite(a)
            if ok.sum() < 0.9 * len(t):
                continue
            r = np.corrcoef(a[ok], gz[ok])[0, 1]
            if mejor is None or r > mejor[0]:
                mejor = (r, d, (par, ksim), espejo != esp2, Pr, q, f3, w3v)
    r_sinc, desfase, par, espejo, Pr, q, f3, w3v = mejor
    tr = tv - desfase                                              # tiempo de la corrida
    info = {"rms_cuadro_mm": rms_cuadro, "giro_deg": math.degrees(par[0][0]) + 90 * par[1], "espejo": bool(espejo),
            "desfase_s": float(desfase), "r_sinc": float(r_sinc),
            "cam_mov_px": float(np.nanmax(np.hypot(v["cam_dx"], v["cam_dy"]))),
            "escala_px_mm": float(np.median(v["lado_px"]) / LADO_MM)}
    if verbose:
        print("  cuadro de 20 mm tras rectificar: error RMS %.2f mm | cámara se movió hasta %.0f px (descontado)"
              % (rms_cuadro, info["cam_mov_px"]))
        print("  sincronización con el giroscopio: r = %.3f, desfase %.3f s" % (r_sinc, desfase))
    return {"t": tr, "P": Pr, "q": q, "phi3": f3, "w3": w3v, "info": info, "corrida": c, "P_cmd": P_cmd}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("marcador_csv")
    ap.add_argument("corrida")
    a = ap.parse_args()
    r = reconstruir(a.marcador_csv, a.corrida)
    out = os.path.join(a.corrida, "video_real.csv")
    np.savetxt(out, np.column_stack([r["t"], r["P"], np.degrees(r["q"])]), delimiter=",", fmt="%.5f",
               header="t_s,x_mm,y_mm,q1_deg,q2_deg", comments="")
    print("  guardado: %s" % out)


if __name__ == "__main__":
    main()
