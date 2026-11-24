"""
Seguimiento del marcador del efector en el video (cuadro rojo/blanco sobre la tapa azul de la torre Z).

Para cada cuadro: posición del centro del cuadro (intersección de las diagonales) y del agujero
central, en píxeles, y el tamaño del cuadro (para la escala local mm/px).

Uso:  python video_seguir.py video.mp4 [--motor auto|ffmpeg|opencv]
Salida: <video>_marcador.csv  (t_s, cx_px, cy_px, hx_px, hy_px, lado_px, angulo_deg, ok y las
        4 esquinas e1..e4 en píxeles, en orden alrededor del centro; cam_* = movimiento de la cámara
        medido con el fondo; fcx, fcy y f1..f4 = centro y esquinas con ese movimiento descontado,
        en las coordenadas del primer cuadro: son las que se usan en el análisis)
"""
import argparse
import json
import math
import subprocess
import sys

import cv2
import numpy as np


def _cuadros_ffmpeg(video):
    """Decodifica con ffmpeg (aplica la rotación del celular). Entrega (t, cuadro BGR)."""
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "frame=best_effort_timestamp_time", "-of", "json", video],
                         capture_output=True, text=True, check=True).stdout
    t = np.array([float(f["best_effort_timestamp_time"]) for f in json.loads(out)["frames"]])
    info = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                           "stream=width,height:stream_side_data=rotation", "-of", "json", video],
                          capture_output=True, text=True, check=True).stdout
    s = json.loads(info)["streams"][0]
    w, h = s["width"], s["height"]
    rot = 0
    for sd in s.get("side_data_list", []):
        rot = int(sd.get("rotation", 0))
    if rot % 180 != 0:
        w, h = h, w
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-i", video, "-fps_mode", "passthrough", "-f", "rawvideo",
                          "-pix_fmt", "bgr24", "-"], stdout=subprocess.PIPE)
    n = w * h * 3
    i = 0
    while True:
        buf = p.stdout.read(n)
        if len(buf) < n or i >= len(t):
            break
        yield t[i], np.frombuffer(buf, np.uint8).reshape(h, w, 3)
        i += 1


def _cuadros_opencv(video):
    """Sin ffmpeg instalado: OpenCV lee el video, aplica la rotación del celular y da el tiempo de cada cuadro."""
    cap = cv2.VideoCapture(video)
    if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 1)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir " + video)
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        yield cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0, fr
    cap.release()


def cuadros(video, motor="auto"):
    """Entrega (t_s, cuadro BGR) de cada cuadro. Usa ffmpeg si está instalado; si no, OpenCV."""
    if motor in ("auto", "ffmpeg"):
        try:
            subprocess.run(["ffprobe", "-version"], capture_output=True, check=True)
            gen = _cuadros_ffmpeg(video)
            primero = next(gen)
            yield primero
            yield from gen
            return
        except Exception:
            if motor == "ffmpeg":
                raise
    yield from _cuadros_opencv(video)


def mascara_marcador(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    azul = (H >= 95) & (H <= 135) & (S > 70)
    m = (~azul) & (V > 90)
    m = m.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return m


def afinar_esquinas(poly, pts):
    """Esquinas con precisión de subpíxel: ajusta una recta a los puntos del contorno de cada lado
    (sin los extremos) y corta las rectas vecinas."""
    cen = poly.mean(axis=0)
    orden = np.argsort(np.arctan2(poly[:, 1] - cen[1], poly[:, 0] - cen[0]))
    poly = poly[orden]
    rectas = []
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        ab = b - a
        L = np.linalg.norm(ab)
        if L < 1:
            return poly
        u = ab / L
        n = np.array([-u[1], u[0]])
        s_ = (pts - a) @ u
        d_ = (pts - a) @ n
        sel = (s_ > 0.15 * L) & (s_ < 0.85 * L) & (np.abs(d_) < 5)
        if sel.sum() < 8:
            return poly
        vx, vy, x0, y0 = cv2.fitLine(pts[sel].astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        rectas.append((np.array([x0, y0]), np.array([vx, vy])))
    out = []
    for i in range(4):
        (p1, d1), (p2, d2) = rectas[i - 1], rectas[i]      # la esquina i está entre el lado i-1 y el lado i
        A = np.array([d1, -d2]).T
        try:
            k = np.linalg.solve(A, p2 - p1)
        except np.linalg.LinAlgError:
            return poly
        out.append(p1 + k[0] * d1)
    out = np.array(out)
    return out if np.all(np.linalg.norm(out - poly, axis=1) < 8) else poly


def encontrar(frame, prev, radio=160, lado_ref=None):
    h, w = frame.shape[:2]
    if prev is None:
        # primera vez: la mancha azul más grande (tapa de la torre) y el marcador dentro
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        azul = cv2.inRange(hsv, (100, 120, 60), (130, 255, 255))
        cnts, _ = cv2.findContours(azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(c)
        prev = (x + bw / 2, y + bh / 2)
    x0 = int(max(0, prev[0] - radio)); x1 = int(min(w, prev[0] + radio))
    y0 = int(max(0, prev[1] - radio)); y1 = int(min(h, prev[1] + radio))
    roi = frame[y0:y1, x0:x1]
    m = mascara_marcador(roi)
    cnts, _ = cv2.findContours(m, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)   # el marcador queda anidado dentro del fondo
    mejor = None
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 1500:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bx <= 1 or by <= 1 or bx + bw >= roi.shape[1] - 1 or by + bh >= roi.shape[0] - 1:
            continue                              # toca el borde de la ventana: no es el marcador
        hull = cv2.convexHull(c)
        rect = cv2.minAreaRect(hull)
        (rw, rh) = rect[1]
        if min(rw, rh) == 0 or max(rw, rh) / min(rw, rh) > 1.35:
            continue
        if lado_ref and not (0.8 * lado_ref < 0.5 * (rw + rh) < 1.25 * lado_ref):
            continue                              # tamaño muy distinto al del marcador
        llenado = a / (rw * rh)
        if llenado < 0.75:
            continue
        d = np.hypot(rect[0][0] + x0 - prev[0], rect[0][1] + y0 - prev[1])
        if mejor is None or d < mejor[0]:
            mejor = (d, hull, rect, c)
    if mejor is None:
        return None
    _, hull, rect, cont = mejor
    peri = cv2.arcLength(hull, True)
    poly = cv2.approxPolyDP(hull, 0.04 * peri, True).reshape(-1, 2).astype(float)
    if len(poly) != 4:
        poly = cv2.boxPoints(rect)
    poly = afinar_esquinas(poly, cont.reshape(-1, 2).astype(float))
    poly = poly + [x0, y0]
    # orden de las esquinas alrededor del centro
    cen = poly.mean(axis=0)
    ang = np.arctan2(poly[:, 1] - cen[1], poly[:, 0] - cen[0])
    poly = poly[np.argsort(ang)]
    # centro = intersección de las diagonales (correcto bajo perspectiva)
    p1, p3 = poly[0], poly[2]
    p2, p4 = poly[1], poly[3]
    A = np.array([p3 - p1, -(p4 - p2)]).T
    try:
        s_, _ = np.linalg.solve(A, p2 - p1)
        c = p1 + s_ * (p3 - p1)
    except np.linalg.LinAlgError:
        c = cen
    lados = [np.linalg.norm(poly[i] - poly[(i + 1) % 4]) for i in range(4)]
    # agujero: la mancha oscura más cercana al centro, dentro del cuadro
    hx = hy = np.nan
    r = int(0.25 * np.mean(lados))
    cx, cy = int(round(c[0])), int(round(c[1]))
    sub = frame[max(0, cy - r):cy + r, max(0, cx - r):cx + r]
    if sub.size:
        g = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
        _, dark = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        M = cv2.moments(dark)
        if M["m00"] > 0 and dark.mean() < 80:
            hx = M["m10"] / M["m00"] + max(0, cx - r)
            hy = M["m01"] / M["m00"] + max(0, cy - r)
    return {"c": c, "poly": poly, "lado": float(np.mean(lados)), "ang": float(np.degrees(rect[2])),
            "hx": hx, "hy": hy}


class Estabilizador:
    """Mide el movimiento de la CÁMARA con puntos fijos del fondo y lo descuenta.
    Para cada cuadro estima una semejanza (giro + desplazamiento + escala) entre el primer cuadro y el
    actual, con flujo óptico sobre puntos del fondo y RANSAC (los brazos, que se mueven, quedan fuera).
    transformar() lleva puntos del cuadro actual a las coordenadas del primer cuadro."""

    def __init__(self, frame):
        self.g0 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.p0 = cv2.goodFeaturesToTrack(self.g0, maxCorners=1500, qualityLevel=0.01, minDistance=12, blockSize=7)
        self.A = np.array([[1, 0, 0], [0, 1, 0]], dtype=float)       # primer cuadro → actual
        self.inl = len(self.p0)
        self.res = 0.0

    def actualizar(self, frame, marcador=None):
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ini = cv2.transform(self.p0, self.A).astype(np.float32)
        lk = dict(winSize=(31, 31), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.g0, g, self.p0, ini.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
        pb, st2, _ = cv2.calcOpticalFlowPyrLK(g, self.g0, p1, self.p0.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
        ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((pb - self.p0).reshape(-1, 2), axis=1) < 0.7)
        if marcador is not None:                       # nada cerca del efector
            ok &= np.linalg.norm(p1.reshape(-1, 2) - np.asarray(marcador), axis=1) > 220
        if ok.sum() >= 12:
            A, inl = cv2.estimateAffinePartial2D(self.p0[ok], p1[ok], method=cv2.RANSAC, ransacReprojThreshold=1.0,
                                                 maxIters=3000, confidence=0.999)
            if A is not None and inl.sum() >= 10:
                self.A = A
                sel = inl.ravel() == 1
                pred = cv2.transform(self.p0[ok][sel], A).reshape(-1, 2)
                self.res = float(np.sqrt(np.mean(np.sum((pred - p1[ok][sel].reshape(-1, 2)) ** 2, axis=1))))
                self.inl = int(sel.sum())
        return self.A

    def transformar(self, pts):
        Ai = cv2.invertAffineTransform(self.A)
        return cv2.transform(np.asarray(pts, dtype=float).reshape(-1, 1, 2), Ai).reshape(-1, 2)

    def parametros(self):
        a, b = self.A[0, 0], self.A[1, 0]
        return self.A[0, 2], self.A[1, 2], math.degrees(math.atan2(b, a)), math.hypot(a, b)


def seguir(video, out=None, motor="auto", progreso=None):
    """Sigue el marcador en todo el video y guarda <video>_marcador.csv (o `out`). Devuelve (out, fracción ok)."""
    filas = []
    prev = None
    lados = []
    est = None
    for i, (ti, fr) in enumerate(cuadros(video, motor)):
        lado_ref = float(np.median(lados[-30:])) if len(lados) >= 5 else None
        r = encontrar(fr, prev, lado_ref=lado_ref)
        if r is None and prev is not None:
            # se perdió (movimiento rápido o salió de la ventana): buscar de nuevo en todo el cuadro
            try:
                r = encontrar(fr, None, lado_ref=lado_ref)
            except ValueError:
                r = None
        if est is None:
            est = Estabilizador(fr)
        est.actualizar(fr, None if r is None else r["c"])
        cam = est.parametros() + (est.inl, est.res)
        if progreso and i % 600 == 0:
            progreso(i, ti)
        if r is None:
            filas.append((ti,) + (np.nan,) * 6 + (0,) + (np.nan,) * 8 + cam + (np.nan,) * 10)
            continue
        prev = tuple(r["c"])
        lados.append(r["lado"])
        fijo = est.transformar(np.vstack([r["c"], r["poly"]]))          # en coordenadas del primer cuadro
        filas.append((ti, r["c"][0], r["c"][1], r["hx"], r["hy"], r["lado"], r["ang"], 1)
                     + tuple(r["poly"].ravel()) + cam + tuple(fijo.ravel()))
    out = out or video.rsplit(".", 1)[0] + "_marcador.csv"
    np.savetxt(out, np.array(filas), delimiter=",", fmt="%.4f",
               header="t_s,cx_px,cy_px,hx_px,hy_px,lado_px,angulo_deg,ok,"
                      "e1x,e1y,e2x,e2y,e3x,e3y,e4x,e4y,"
                      "cam_dx,cam_dy,cam_giro_deg,cam_escala,cam_puntos,cam_res_px,"
                      "fcx,fcy,f1x,f1y,f2x,f2y,f3x,f3y,f4x,f4y", comments="")
    ok = np.array([f[7] for f in filas])
    return out, len(filas), float(ok.mean()) if len(ok) else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--motor", default="auto", choices=["auto", "ffmpeg", "opencv"])
    a = ap.parse_args()
    out, n, fr = seguir(a.video, motor=a.motor)
    print("%s: %d cuadros, marcador encontrado en %.1f %%" % (out, n, 100 * fr))


if __name__ == "__main__":
    main()
