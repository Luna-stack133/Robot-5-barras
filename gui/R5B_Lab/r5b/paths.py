"""Recorridos cartesianos (mm): figuras, esquinas donde frena el robot y validación.

Las reglas de esquinas y de reducción de puntos son las mismas del firmware y de la
GUI anterior (prueba.py): el robot solo frena donde la dirección gira más de CORNER_DEG.
"""
import math

from .config import CORNER_DEG, MAX_WP_HOST
from .kinematics import quality_xy

FIGURES = [
    ("circle", "Círculo"),
    ("square", "Cuadrado"),
    ("triangle", "Triángulo"),
    ("star", "Estrella"),
    ("infinity", "Infinito"),
    ("line", "Línea"),
]


def densify(points, step=1.0, closed=False):
    """Inserta puntos intermedios cada `step` mm en un polígono."""
    pts = list(points)
    if closed and pts:
        pts = pts + [pts[0]]
    out = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
        for k in range(n):
            t = k / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    if pts:
        out.append(pts[-1])
    return out


def figure_points(kind, cx, cy, size, repetitions=1):
    """Puntos (x, y) en mm de una figura centrada en (cx, cy) con tamaño total `size`."""
    r = size / 2.0
    if kind == "circle":
        n = 120
        base = [(cx + r * math.cos(math.pi / 2 + 2 * math.pi * k / n),
                 cy + r * math.sin(math.pi / 2 + 2 * math.pi * k / n)) for k in range(n + 1)]
    elif kind == "square":
        base = densify([(cx - r, cy + r), (cx + r, cy + r), (cx + r, cy - r), (cx - r, cy - r)],
                       step=size / 12.0, closed=True)
    elif kind == "triangle":
        v = [(cx + r * math.cos(math.radians(90 + 120 * k)), cy + r * math.sin(math.radians(90 + 120 * k)))
             for k in range(3)]
        base = densify(v, step=size / 14.0, closed=True)
    elif kind == "star":
        v = []
        for k in range(10):
            rr = r if k % 2 == 0 else 0.45 * r
            a = math.radians(90 + 36 * k)
            v.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
        base = densify(v, step=size / 16.0, closed=True)
    elif kind == "infinity":
        n = 120
        base = []
        for k in range(n + 1):
            t = 2 * math.pi * k / n
            d = 1 + math.sin(t) ** 2
            base.append((cx + r * math.cos(t) / d, cy + r * math.sin(t) * math.cos(t) / d))
    elif kind == "line":
        base = [(cx - r, cy - r / 2), (cx + r, cy + r / 2)]
    else:
        raise ValueError("Figura desconocida: %s" % kind)
    out = list(base)
    for _ in range(max(1, int(repetitions)) - 1):
        out += base[1:]
    return out


def corner_indices(pts):
    """Índices donde el robot se detiene: extremos y giros > CORNER_DEG (= isStopPoint del firmware)."""
    if len(pts) < 2:
        return list(range(len(pts)))
    cmin = math.cos(math.radians(CORNER_DEG))
    idx = [0]
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        la, lb = math.hypot(ax, ay), math.hypot(bx, by)
        if la < 1e-9 or lb < 1e-9:
            continue
        if (ax * bx + ay * by) / (la * lb) < cmin:
            idx.append(i)
    idx.append(len(pts) - 1)
    return idx


def limit_points(pts, max_n=MAX_WP_HOST):
    """Reduce la lista a max_n puntos sin perder las esquinas (el firmware acepta 130)."""
    if len(pts) <= max_n:
        return list(pts)
    keep = set(corner_indices(pts))
    others = [i for i in range(len(pts)) if i not in keep]
    room = max(0, max_n - len(keep))
    if room and others:
        step = len(others) / room
        keep.update(others[int(k * step)] for k in range(room))
    return [pts[i] for i in sorted(keep)][:max_n]


def check_path(points, warn_quality=0.30, check_step=2.0):
    """Valida un recorrido (también entre puntos, cada `check_step` mm).

    Devuelve dict(ok, bad_index, bad_point, min_quality, length, warning).
    """
    min_q = 1.0
    length = 0.0
    for i, (x, y) in enumerate(points):
        seg = [(x, y)]
        if i:
            x0, y0 = points[i - 1]
            dl = math.hypot(x - x0, y - y0)
            length += dl
            n = int(dl // check_step)
            seg = [(x0 + (x - x0) * k / (n + 1), y0 + (y - y0) * k / (n + 1)) for k in range(1, n + 1)] + seg
        for px, py in seg:
            q = quality_xy(px, py)
            if q is None:
                return {"ok": False, "bad_index": i, "bad_point": (px, py), "min_quality": None,
                        "length": length, "warning": False}
            min_q = min(min_q, q)
    return {"ok": True, "bad_index": None, "bad_point": None, "min_quality": min_q, "length": length,
            "warning": min_q < warn_quality}


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points[:-1], points[1:]))
