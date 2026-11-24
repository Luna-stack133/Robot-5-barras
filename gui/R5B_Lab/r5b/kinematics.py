"""Cinemática del robot de 5 barras: traducción directa de kinematics_5bar.h.

Diferencias con el firmware solo de unidades: aquí mm y grados (el firmware usa m y rad).

Convención:
    O1 = (0, 0) motor izquierdo, O2 = (D, 0) motor derecho.
    Modo de ensamblaje m: th_i = phi_i + sgn_i * beta_i   (ver config.MODE_SIGNS)
        1: codos afuera   2: codos adentro   3: izq. afuera / der. adentro   4: izq. adentro / der. afuera

La geometría y el modo activo son variables del módulo: la app las actualiza cuando
el firmware responde "GEOM ..." o "MODE n".
"""
import math

from . import config

GEOM = {"L1": config.L1, "L2": config.L2, "L3": config.L3, "L4": config.L4, "D": config.D}
STATE = {"mode": config.DEFAULT_MODE, "version": 0}   # version cambia con geometría o modo
TOL = 1e-6


def set_geometry(l1, l2, l3, l4, d):
    """Geometría en mm. Devuelve True si cambió."""
    new = {"L1": l1, "L2": l2, "L3": l3, "L4": l4, "D": d}
    if all(abs(new[k] - GEOM[k]) < 1e-6 for k in new):
        return False
    GEOM.update(new)
    STATE["version"] += 1
    return True


def set_mode(mode):
    if mode in config.MODE_SIGNS and mode != STATE["mode"]:
        STATE["mode"] = mode
        STATE["version"] += 1
        return True
    return False


def mode():
    return STATE["mode"]


def _rem(a):
    """remainderf(a, 2*pi) en radianes."""
    return math.remainder(a, 2 * math.pi)


# ------------------------------------------------------------------ IK
def ik(x, y, m=None):
    """(θ1, θ2) en grados para el punto (x, y) mm en el modo m, o None si no es alcanzable."""
    s1, s2 = config.MODE_SIGNS[m or STATE["mode"]]
    L1, L2, L3, L4, D = (GEOM[k] for k in ("L1", "L2", "L3", "L4", "D"))
    r1 = math.hypot(x, y)
    if r1 < TOL or r1 > L1 + L2 + TOL or r1 < abs(L1 - L2) - TOL:
        return None
    phi1 = math.atan2(y, x)
    b1 = math.acos(max(-1.0, min(1.0, (r1 * r1 + L1 * L1 - L2 * L2) / (2 * r1 * L1))))
    dx = x - D
    r2 = math.hypot(dx, y)
    if r2 < TOL or r2 > L4 + L3 + TOL or r2 < abs(L4 - L3) - TOL:
        return None
    phi2 = math.atan2(y, dx)
    b2 = math.acos(max(-1.0, min(1.0, (r2 * r2 + L4 * L4 - L3 * L3) / (2 * r2 * L4))))
    return math.degrees(phi1 + s1 * b1), math.degrees(phi2 + s2 * b2)


# ------------------------------------------------------------------ FK
def elbows(t1, t2):
    """Codos A (izq.) y B (der.) en mm para θ1, θ2 en grados."""
    L1, L4, D = GEOM["L1"], GEOM["L4"], GEOM["D"]
    r1, r2 = math.radians(t1), math.radians(t2)
    return (L1 * math.cos(r1), L1 * math.sin(r1)), (D + L4 * math.cos(r2), L4 * math.sin(r2))


def fk_both(t1, t2):
    """Las dos soluciones [(x, y) rama +h, (x, y) rama -h] o None si la cadena no cierra."""
    L2, L3 = GEOM["L2"], GEOM["L3"]
    (ax, ay), (bx, by) = elbows(t1, t2)
    ex, ey = bx - ax, by - ay
    d = math.hypot(ex, ey)
    if d > L2 + L3 + TOL or d < abs(L2 - L3) - TOL or d < TOL:
        return None
    a = (L2 * L2 - L3 * L3 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, L2 * L2 - a * a))
    mx, my = ax + a * ex / d, ay + a * ey / d
    px, py = -ey / d, ex / d
    return [(mx + h * px, my + h * py), (mx - h * px, my - h * py)]


def angles_match(x, y, t1, t2, m, tol_deg=0.06):
    s = ik(x, y, m)
    return (s is not None and abs(math.degrees(_rem(math.radians(s[0] - t1)))) < tol_deg
            and abs(math.degrees(_rem(math.radians(s[1] - t2)))) < tol_deg)


def detect_mode(x, y, t1, t2):
    for m in config.MODE_SIGNS:
        if angles_match(x, y, t1, t2, m):
            return m
    return 0


def fk_mode(t1, t2, m=None):
    """Punto del efector coherente con el modo (como fk5barMode). Si ninguna rama coincide,
    devuelve la de efector más alto (como la GUI anterior). None si la cadena no cierra."""
    c = fk_both(t1, t2)
    if c is None:
        return None
    m = m or STATE["mode"]
    for p in c:
        if angles_match(p[0], p[1], t1, t2, m):
            return p
    return max(c, key=lambda p: p[1])


def fk_near(t1, t2, ref):
    """Solución más cercana a ref (seguimiento por continuidad, como fk5barNear)."""
    c = fk_both(t1, t2)
    if c is None:
        return None
    return min(c, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)


def points(t1, t2, p):
    """Puntos del mecanismo para dibujar: A=O1, B=O2, C=codo izq., D=codo der., P=efector."""
    c, d = elbows(t1, t2)
    return {"A": (0.0, 0.0), "B": (GEOM["D"], 0.0), "C": c, "D": d, "P": p}


# ------------------------------------------------------------------ singularidad
def sing_measure(x, y, t1, t2):
    """|sin(a1 - a2)| ~ |det(J_P)|: 0 cuando los distales se alinean (= firmware)."""
    (ax, ay), (bx, by) = elbows(t1, t2)
    a1 = math.atan2(y - ay, x - ax)
    a2 = math.atan2(y - by, x - bx)
    return abs(math.sin(a1 - a2))


def joint_limits_ok(t1, t2):
    return (config.TH1_LIMITS[0] <= t1 <= config.TH1_LIMITS[1]
            and config.TH2_LIMITS[0] <= t2 <= config.TH2_LIMITS[1])


def quality_xy(x, y, m=None):
    """None si (x, y) no es alcanzable o está más cerca de la singularidad que SIN_MIN
    (el firmware lo rechazaría); si no, la medida de singularidad (0..1)."""
    q = ik(x, y, m)
    if q is None:
        return None
    s = sing_measure(x, y, q[0], q[1])
    return s if s >= config.SIN_MIN else None


def reachable_safe(x, y, m=None):
    return quality_xy(x, y, m) is not None


def home_point(t1=None, t2=None):
    """Efector con los brazos en los ángulos del utillaje (por defecto config.HOME_DEG)."""
    t1 = config.HOME_DEG[0] if t1 is None else t1
    t2 = config.HOME_DEG[1] if t2 is None else t2
    c = fk_both(t1, t2)
    if c is None:
        return None
    return max(c, key=lambda p: p[1])
