#!/usr/bin/env python3
# =============================================================================
#  ROBOT PARALELO 5 BARRAS  -  INTERFAZ DE CONTROL (Python / Tkinter)
#
#  Novedades de esta version:
#   - VISTA DE ESLABONES: dibuja la posicion de los brazos en 3 capas ->
#       * pose de CERO/utillaje (gris claro, referencia)
#       * VISTA PREVIA al pasar el raton o fijar un punto (naranja punteado)
#       * pose REAL del robot por telemetria cuando hay conexion (azul/verde)
#   - La GUI puede PEDIR LA GEOMETRIA AL FIRMWARE al conectar (comando GEOM):
#       editas solo kinematics_5bar.h y la interfaz se sincroniza sola.
#   - Cero por UTILLAJE (SETHOME) + JOG manual.
#   - Cero por UTILLAJE (SETHOME) + JOG manual.  Sin ENABLE (solo PUL/DIR).
#
#  REQUISITOS:   pip install pyserial matplotlib numpy
#  EJECUTAR:     python robot_5bar_gui.py
# =============================================================================
import threading
import queue
import math
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    import serial.tools.list_ports as list_ports
    HAVE_SERIAL = True
except ImportError:
    serial = None
    list_ports = None
    HAVE_SERIAL = False

# ============================================================================
#   GEOMETRIA  (EDITA AQUI si trabajas sin conexion)
#   DEBE coincidir con kinematics_5bar.h.  Si conectas y el firmware soporta el
#   comando GEOM, estos valores se sobrescriben con los del firmware al conectar.
# ============================================================================
L1, L2, L3, L4, D = 0.25, 0.30, 0.30, 0.25, 0.30
SIN_MIN = 0.15            # guarda de singularidad (= firmware)
BAUD = 115200

# ============================ CINEMATICA =====================================
def ik5bar(x, y):
    """IK config 3. Devuelve (th1, th2) en rad o None si no alcanzable."""
    r1 = math.hypot(x, y)
    if r1 > L1 + L2 + 1e-9 or r1 < abs(L1 - L2) - 1e-9:
        return None
    phi1 = math.atan2(y, x)
    b1 = math.acos(max(-1.0, min(1.0, (r1*r1 + L1*L1 - L2*L2) / (2*r1*L1))))
    r2 = math.hypot(x - D, y)
    if r2 > L4 + L3 + 1e-9 or r2 < abs(L4 - L3) - 1e-9:
        return None
    phi2 = math.atan2(y, x - D)
    b2 = math.acos(max(-1.0, min(1.0, (r2*r2 + L4*L4 - L3*L3) / (2*r2*L4))))
    return (phi1 + b1, phi2 + b2)

def fk5bar(th1, th2):
    """FK config 1 (corresponde a IK config 3). Devuelve (x, y) o None."""
    Ax, Ay = L1*math.cos(th1), L1*math.sin(th1)
    Bx, By = D + L4*math.cos(th2), L4*math.sin(th2)
    ex, ey = Bx - Ax, By - Ay
    dAB = math.hypot(ex, ey)
    if dAB > L2 + L3 + 1e-9 or dAB < abs(L2 - L3) - 1e-9 or dAB < 1e-9:
        return None
    a = (L2*L2 - L3*L3 + dAB*dAB) / (2*dAB)
    h = math.sqrt(max(0.0, L2*L2 - a*a))
    Mx, My = Ax + a*ex/dAB, Ay + a*ey/dAB
    px, py = -ey/dAB, ex/dAB
    return (Mx + h*px, My + h*py)

def sing_measure(x, y, th1, th2):
    Ax, Ay = L1*math.cos(th1), L1*math.sin(th1)
    Bx, By = D + L4*math.cos(th2), L4*math.sin(th2)
    a1 = math.atan2(y - Ay, x - Ax)
    a2 = math.atan2(y - By, x - Bx)
    return abs(math.sin(a1 - a2))

def reachable_safe(x, y):
    s = ik5bar(x, y)
    if s is None:
        return False
    return sing_measure(x, y, s[0], s[1]) >= SIN_MIN

def arm_points(th1, th2, x, y):
    """Devuelve los segmentos de los dos brazos para dibujar el robot."""
    Ax, Ay = L1*math.cos(th1), L1*math.sin(th1)        # codo izquierdo
    Bx, By = D + L4*math.cos(th2), L4*math.sin(th2)    # codo derecho
    left = ([0, Ax, x], [0, Ay, y])                    # O1 -> A -> P
    right = ([D, Bx, x], [0, By, y])                   # O2 -> B -> P
    return left, right

# ============================ FIGURAS ========================================
def fig_circle(cx, cy, r, n=60):
    return [(cx + r*math.cos(2*math.pi*i/n), cy + r*math.sin(2*math.pi*i/n))
            for i in range(n + 1)]

def fig_square(cx, cy, side, n_side=12):
    h = side / 2.0
    corners = [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]
    pts = []
    for k in range(len(corners) - 1):
        x0, y0 = corners[k]; x1, y1 = corners[k+1]
        for j in range(n_side):
            t = j / n_side
            pts.append((cx + x0 + (x1-x0)*t, cy + y0 + (y1-y0)*t))
    pts.append((cx + corners[0][0], cy + corners[0][1]))
    return pts

def fig_triangle(cx, cy, size, n_side=14):
    R = size / 2.0
    verts = [(cx + R*math.cos(math.radians(90 + 120*k)),
              cy + R*math.sin(math.radians(90 + 120*k))) for k in range(3)]
    verts.append(verts[0])
    pts = []
    for k in range(3):
        x0, y0 = verts[k]; x1, y1 = verts[k+1]
        for j in range(n_side):
            t = j / n_side
            pts.append((x0 + (x1-x0)*t, y0 + (y1-y0)*t))
    pts.append(verts[0])
    return pts

def fig_star(cx, cy, r, points=5):
    pts = []
    n = points * 2
    for i in range(n + 1):
        rad = r if i % 2 == 0 else r * 0.45
        ang = math.pi/2 + math.pi * i / points
        pts.append((cx + rad*math.cos(ang), cy + rad*math.sin(ang)))
    return pts

FIGURES = {
    "Circulo":   lambda cx, cy, s: fig_circle(cx, cy, s/2.0),
    "Cuadrado":  lambda cx, cy, s: fig_square(cx, cy, s),
    "Triangulo": lambda cx, cy, s: fig_triangle(cx, cy, s),
    "Estrella":  lambda cx, cy, s: fig_star(cx, cy, s/2.0),
}

# ============================ ENLACE SERIE ===================================
class SerialLink:
    def __init__(self, rx_queue):
        self.ser = None
        self.rx = rx_queue
        self.reader = None
        self.running = False

    def open(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _read_loop(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.rx.put(line.decode(errors="replace").strip())
            except Exception as e:
                self.rx.put(f"ERR serial: {e}")
                break

    def send(self, line):
        if self.ser and self.ser.is_open:
            self.ser.write((line + "\n").encode())

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

# ============================ APLICACION =====================================
class App:
    def __init__(self, root):
        self.root = root
        root.title("Robot Paralelo 5 Barras - Control")
        self.rx = queue.Queue()
        self.link = SerialLink(self.rx)
        self.target = None
        self.path_preview = []
        self._last_hover = None

        self._build_ui()
        self._draw_workspace()
        self._update_home_pose()
        self.root.after(50, self._poll_serial)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Puerto:").pack(side=tk.LEFT)
        self.port_cb = ttk.Combobox(top, width=22, values=self._ports())
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="\u21bb", width=3, command=self._refresh_ports).pack(side=tk.LEFT)
        self.btn_conn = ttk.Button(top, text="Conectar", command=self._toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=6)
        self.lbl_conn = ttk.Label(top, text="desconectado", foreground="#a33")
        self.lbl_conn.pack(side=tk.LEFT, padx=4)
        if not HAVE_SERIAL:
            self.lbl_conn.config(text="pyserial no instalado (pip install pyserial)")

        body = ttk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- grafica ---
        self.fig = Figure(figsize=(6.4, 6.4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=body)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self._on_click)
        self.canvas.mpl_connect("motion_notify_event", self._on_hover)

        # --- panel de control ---
        panel = ttk.Frame(body, padding=8)
        panel.pack(side=tk.RIGHT, fill=tk.Y)

        g1 = ttk.LabelFrame(panel, text="Robot", padding=6)
        g1.pack(fill=tk.X, pady=4)
        stop = tk.Button(g1, text="\u23f9  PARO (STOP)", bg="#c0392b", fg="white",
                         font=("TkDefaultFont", 11, "bold"), command=lambda: self._send("STOP"))
        stop.pack(fill=tk.X, pady=3, ipady=4)

        gz = ttk.LabelFrame(panel, text="Calibrar cero (utillaje)", padding=6)
        gz.pack(fill=tk.X, pady=4)
        ttk.Label(gz, text="Angulos del utillaje (grados):").pack(anchor=tk.W)
        zr = ttk.Frame(gz); zr.pack(fill=tk.X)
        ttk.Label(zr, text="\u03b81:").pack(side=tk.LEFT)
        self.z1 = ttk.Entry(zr, width=7); self.z1.insert(0, "90"); self.z1.pack(side=tk.LEFT, padx=2)
        ttk.Label(zr, text="\u03b82:").pack(side=tk.LEFT)
        self.z2 = ttk.Entry(zr, width=7); self.z2.insert(0, "90"); self.z2.pack(side=tk.LEFT, padx=2)
        ttk.Button(zr, text="ver", width=4, command=self._update_home_pose).pack(side=tk.LEFT, padx=2)
        jr = ttk.Frame(gz); jr.pack(fill=tk.X, pady=3)
        ttk.Label(jr, text="Jog paso:").pack(side=tk.LEFT)
        self.jog_step = ttk.Entry(jr, width=5); self.jog_step.insert(0, "2"); self.jog_step.pack(side=tk.LEFT, padx=2)
        ttk.Label(jr, text="\u00b0").pack(side=tk.LEFT)
        jb = ttk.Frame(gz); jb.pack(fill=tk.X)
        ttk.Button(jb, text="\u03b81 \u2212", width=5, command=lambda: self._jog(1, -1)).grid(row=0, column=0, padx=1)
        ttk.Button(jb, text="\u03b81 +", width=5, command=lambda: self._jog(1, +1)).grid(row=0, column=1, padx=1)
        ttk.Button(jb, text="\u03b82 \u2212", width=5, command=lambda: self._jog(2, -1)).grid(row=0, column=2, padx=1)
        ttk.Button(jb, text="\u03b82 +", width=5, command=lambda: self._jog(2, +1)).grid(row=0, column=3, padx=1)
        ttk.Button(gz, text="Fijar CERO (utillaje puesto)", command=self._set_home).pack(fill=tk.X, pady=3)
        ttk.Label(gz, text="1) poner utillaje 2) Fijar CERO 3) retirar",
                  foreground="#666").pack(anchor=tk.W)

        g2 = ttk.LabelFrame(panel, text="Velocidad del efector", padding=6)
        g2.pack(fill=tk.X, pady=4)
        self.var_v = tk.DoubleVar(value=0.10)
        self.var_a = tk.DoubleVar(value=0.20)
        self._slider(g2, "Vmax", self.var_v, 0.02, 0.30, "m/s")
        self._slider(g2, "Amax", self.var_a, 0.05, 0.40, "m/s\u00b2")

        g3 = ttk.LabelFrame(panel, text="Ir a coordenada", padding=6)
        g3.pack(fill=tk.X, pady=4)
        row = ttk.Frame(g3); row.pack(fill=tk.X)
        ttk.Label(row, text="X:").pack(side=tk.LEFT)
        self.ent_x = ttk.Entry(row, width=7); self.ent_x.pack(side=tk.LEFT, padx=2)
        ttk.Label(row, text="Y:").pack(side=tk.LEFT)
        self.ent_y = ttk.Entry(row, width=7); self.ent_y.pack(side=tk.LEFT, padx=2)
        ttk.Button(g3, text="Mover a (X,Y)", command=self._goto_entry).pack(fill=tk.X, pady=2)
        ttk.Label(g3, text="(o haz click en el area)", foreground="#666").pack(anchor=tk.W)

        g4 = ttk.LabelFrame(panel, text="Trazar figura", padding=6)
        g4.pack(fill=tk.X, pady=4)
        self.fig_cb = ttk.Combobox(g4, width=12, values=list(FIGURES.keys()), state="readonly")
        self.fig_cb.current(0)
        self.fig_cb.pack(fill=tk.X, pady=2)
        fr = ttk.Frame(g4); fr.pack(fill=tk.X)
        ttk.Label(fr, text="Centro X:").grid(row=0, column=0, sticky=tk.W)
        self.fc_x = ttk.Entry(fr, width=6); self.fc_x.insert(0, "0.15"); self.fc_x.grid(row=0, column=1)
        ttk.Label(fr, text="Y:").grid(row=0, column=2, sticky=tk.W)
        self.fc_y = ttk.Entry(fr, width=6); self.fc_y.insert(0, "0.25"); self.fc_y.grid(row=0, column=3)
        ttk.Label(fr, text="Tamano:").grid(row=1, column=0, sticky=tk.W)
        self.fc_s = ttk.Entry(fr, width=6); self.fc_s.insert(0, "0.12"); self.fc_s.grid(row=1, column=1)
        ttk.Button(g4, text="Previsualizar", command=self._preview_figure).pack(fill=tk.X, pady=2)
        ttk.Button(g4, text="Ejecutar figura", command=self._run_figure).pack(fill=tk.X, pady=2)

        g5 = ttk.LabelFrame(panel, text="Estado", padding=6)
        g5.pack(fill=tk.BOTH, expand=True, pady=4)
        self.lbl_pose = ttk.Label(g5, text="pose: ---", font=("TkFixedFont", 9))
        self.lbl_pose.pack(anchor=tk.W)
        self.log = tk.Text(g5, width=34, height=8, font=("TkFixedFont", 8))
        self.log.pack(fill=tk.BOTH, expand=True, pady=2)

    def _slider(self, parent, name, var, lo, hi, unit):
        fr = ttk.Frame(parent); fr.pack(fill=tk.X, pady=1)
        lbl = ttk.Label(fr, text=f"{name}: {var.get():.3f} {unit}", width=22)
        lbl.pack(side=tk.LEFT)
        def on_move(_v):
            lbl.config(text=f"{name}: {var.get():.3f} {unit}")
        def on_release(_e):
            self._send(f"SETV {self.var_v.get():.4f} {self.var_a.get():.4f}")
        s = ttk.Scale(parent, from_=lo, to=hi, variable=var, command=on_move)
        s.pack(fill=tk.X)
        s.bind("<ButtonRelease-1>", on_release)

    # ---------------------------------------------------------- workspace plot
    def _draw_workspace(self):
        self.ax.clear()
        nx, ny = 180, 180
        xlim = (-0.20, max(0.50, D + 0.20))
        ylim = (-0.10, max(0.50, L1 + L2 + 0.05))
        xs = np.linspace(xlim[0], xlim[1], nx)
        ys = np.linspace(ylim[0], ylim[1], ny)
        Z = np.full((ny, nx), np.nan)
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                s = ik5bar(x, y)
                if s is None:
                    continue
                m = sing_measure(x, y, s[0], s[1])
                if m >= SIN_MIN:
                    Z[j, i] = m
        self.ax.contourf(xs, ys, Z, levels=20, cmap="viridis", alpha=0.50)
        self.ax.contour(xs, ys, np.isfinite(Z).astype(float), levels=[0.5],
                        colors="#2c3e50", linewidths=1.0)
        # bases y motores
        self.ax.plot([0, D], [0, 0], "k-", lw=3, zorder=5)
        self.ax.plot(0, 0, "ks", ms=10, zorder=6)
        self.ax.plot(D, 0, "ks", ms=10, zorder=6)
        self.ax.text(0, -0.03, "O1", ha="center")
        self.ax.text(D, -0.03, "O2", ha="center")

        # capa 1: pose de CERO / utillaje (referencia, gris)
        (self.hm_left,)  = self.ax.plot([], [], "-", color="#b0b0b0", lw=2, zorder=6)
        (self.hm_right,) = self.ax.plot([], [], "-", color="#b0b0b0", lw=2, zorder=6)
        # capa 2: VISTA PREVIA (raton / destino, naranja punteado)
        (self.gh_left,)  = self.ax.plot([], [], "--", color="#e67e22", lw=2, zorder=7)
        (self.gh_right,) = self.ax.plot([], [], "--", color="#e67e22", lw=2, zorder=7)
        (self.gh_ef,)    = self.ax.plot([], [], "o", color="#e67e22", ms=7, zorder=8)
        # capa 3: pose REAL (telemetria)
        (self.ln_left,)  = self.ax.plot([], [], "-o", color="#2980b9", lw=3, ms=5, zorder=9)
        (self.ln_right,) = self.ax.plot([], [], "-o", color="#27ae60", lw=3, ms=5, zorder=9)
        (self.ln_ef,)    = self.ax.plot([], [], "ro", ms=8, zorder=10)
        # destino fijado y figura
        (self.ln_target,)= self.ax.plot([], [], "x", color="#c0392b", ms=12, mew=3, zorder=9)
        (self.ln_path,)  = self.ax.plot([], [], ":", color="#8e44ad", lw=1.5, zorder=4)

        # leyenda compacta
        from matplotlib.lines import Line2D
        leg = [Line2D([0],[0], color="#b0b0b0", lw=2, label="cero/utillaje"),
               Line2D([0],[0], color="#e67e22", lw=2, ls="--", label="vista previa"),
               Line2D([0],[0], color="#2980b9", lw=3, label="pose real")]
        self.ax.legend(handles=leg, loc="upper right", fontsize=8, framealpha=0.8)

        self.ax.set_aspect("equal")
        self.ax.set_xlim(*xlim); self.ax.set_ylim(*ylim)
        self.ax.set_xlabel("X [m]"); self.ax.set_ylabel("Y [m]")
        self.ax.set_title(f"Area de trabajo  (L1={L1:.3f} L2={L2:.3f} "
                          f"L3={L3:.3f} L4={L4:.3f} D={D:.3f})", fontsize=9)
        self.ax.grid(alpha=0.2)
        self.canvas.draw()

    # ---------------------------------------------------------- dibujo de pose
    def _draw_arm(self, lleft, lright, lef, th1, th2, x, y):
        left, right = arm_points(th1, th2, x, y)
        lleft.set_data(*left)
        lright.set_data(*right)
        if lef is not None:
            lef.set_data([x], [y])

    def _update_home_pose(self):
        """Dibuja la pose de cero (angulos del utillaje) como referencia gris."""
        try:
            t1 = math.radians(float(self.z1.get()))
            t2 = math.radians(float(self.z2.get()))
        except ValueError:
            return
        p = fk5bar(t1, t2)
        if p is None:
            self.hm_left.set_data([], []); self.hm_right.set_data([], [])
        else:
            self._draw_arm(self.hm_left, self.hm_right, None, t1, t2, p[0], p[1])
        self.canvas.draw_idle()

    def _on_hover(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        x, y = round(event.xdata, 3), round(event.ydata, 3)
        if (x, y) == self._last_hover:
            return
        self._last_hover = (x, y)
        s = ik5bar(x, y)
        if s is not None and sing_measure(x, y, s[0], s[1]) >= SIN_MIN:
            self._draw_arm(self.gh_left, self.gh_right, self.gh_ef, s[0], s[1], x, y)
        else:
            self.gh_left.set_data([], []); self.gh_right.set_data([], [])
            self.gh_ef.set_data([], [])
        self.canvas.draw_idle()

    # ---------------------------------------------------------------- acciones
    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        if not reachable_safe(x, y):
            self._log(f"({x:.3f},{y:.3f}) fuera de alcance / singular")
            return
        self.target = (x, y)
        self.ent_x.delete(0, tk.END); self.ent_x.insert(0, f"{x:.3f}")
        self.ent_y.delete(0, tk.END); self.ent_y.insert(0, f"{y:.3f}")
        self.ln_target.set_data([x], [y])
        self.canvas.draw_idle()
        self._send(f"GOTO {x:.4f} {y:.4f}")

    def _goto_entry(self):
        try:
            x = float(self.ent_x.get()); y = float(self.ent_y.get())
        except ValueError:
            messagebox.showerror("Error", "Coordenadas invalidas"); return
        if not reachable_safe(x, y):
            messagebox.showwarning("Fuera de alcance",
                                   "Ese punto no es alcanzable o esta en singularidad."); return
        self.target = (x, y)
        self.ln_target.set_data([x], [y]); self.canvas.draw_idle()
        self._send(f"GOTO {x:.4f} {y:.4f}")

    def _jog(self, axis, direction):
        try:
            step = float(self.jog_step.get())
        except ValueError:
            step = 2.0
        self._send(f"JOG {axis} {direction * step:.2f}")

    def _set_home(self):
        try:
            t1 = float(self.z1.get()); t2 = float(self.z2.get())
        except ValueError:
            messagebox.showerror("Error", "Angulos del utillaje invalidos"); return
        self._send(f"SETHOME {t1:.3f} {t2:.3f}")
        self._update_home_pose()

    def _gen_figure(self):
        try:
            cx = float(self.fc_x.get()); cy = float(self.fc_y.get()); s = float(self.fc_s.get())
        except ValueError:
            messagebox.showerror("Error", "Parametros de figura invalidos"); return None
        pts = FIGURES[self.fig_cb.get()](cx, cy, s)
        bad = [(i, p) for i, p in enumerate(pts) if not reachable_safe(*p)]
        if bad:
            messagebox.showwarning("Figura fuera de alcance",
                f"{len(bad)} de {len(pts)} puntos no son alcanzables.\n"
                "Reduce el tamano o mueve el centro.")
            return None
        return pts

    def _preview_figure(self):
        pts = self._gen_figure()
        if not pts:
            self.path_preview = []
            self.ln_path.set_data([], [])
        else:
            self.path_preview = pts
            self.ln_path.set_data([p[0] for p in pts], [p[1] for p in pts])
        self.canvas.draw_idle()

    def _run_figure(self):
        pts = self.path_preview or self._gen_figure()
        if not pts:
            return
        if len(pts) > 130:
            step = math.ceil(len(pts) / 130)
            pts = pts[::step] + [pts[-1]]
        self.ln_path.set_data([p[0] for p in pts], [p[1] for p in pts]); self.canvas.draw_idle()
        flat = " ".join(f"{x:.4f} {y:.4f}" for x, y in pts)
        self._send(f"PATH {len(pts)} {flat}")

    # ---------------------------------------------------------------- serial
    def _ports(self):
        if not HAVE_SERIAL:
            return []
        return [p.device for p in list_ports.comports()]

    def _refresh_ports(self):
        self.port_cb["values"] = self._ports()

    def _toggle_conn(self):
        if not HAVE_SERIAL:
            messagebox.showinfo("pyserial", "Instala pyserial: pip install pyserial"); return
        if self.link.ser and self.link.ser.is_open:
            self.link.close()
            self.lbl_conn.config(text="desconectado", foreground="#a33")
            self.btn_conn.config(text="Conectar")
            return
        port = self.port_cb.get().strip()
        if not port:
            messagebox.showwarning("Puerto", "Selecciona un puerto"); return
        try:
            self.link.open(port)
        except Exception as e:
            messagebox.showerror("Conexion", str(e)); return
        self.lbl_conn.config(text=f"conectado {port}", foreground="#2a7")
        self.btn_conn.config(text="Desconectar")
        # secuencia de arranque: ping y pedir geometria al firmware
        self.root.after(300, lambda: self._send("PING"))
        self.root.after(600, lambda: self._send("GEOM"))

    def _send(self, cmd):
        if self.link.ser and self.link.ser.is_open:
            self.link.send(cmd)
            self._log(">> " + (cmd if len(cmd) < 60 else cmd[:57] + "..."))
        else:
            self._log("(sin conexion) " + cmd[:50])

    def _poll_serial(self):
        try:
            while True:
                self._handle_rx(self.rx.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self._poll_serial)

    def _handle_rx(self, line):
        if line.startswith("STATE"):
            parts = line.split()
            if len(parts) >= 7:
                try:
                    _, st, x, y, t1, t2, mov = parts[:7]
                    x = float(x); y = float(y); t1 = float(t1); t2 = float(t2)
                    self._draw_arm(self.ln_left, self.ln_right, self.ln_ef,
                                   math.radians(t1), math.radians(t2), x, y)
                    self.canvas.draw_idle()
                    self.lbl_pose.config(
                        text=f"{st}  x={x:+.3f} y={y:+.3f}\n"
                             f"th1={t1:6.1f}  th2={t2:6.1f}")
                except ValueError:
                    pass
            return
        if line.startswith("GEOM"):
            parts = line.split()
            if len(parts) >= 6:
                try:
                    self._set_geometry(*[float(v) for v in parts[1:6]])
                    self._log(f"geometria del firmware aplicada")
                except ValueError:
                    pass
            return
        self._log(line)

    def _set_geometry(self, l1, l2, l3, l4, d):
        global L1, L2, L3, L4, D
        L1, L2, L3, L4, D = l1, l2, l3, l4, d
        self._draw_workspace()
        self._update_home_pose()

    def _log(self, msg):
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)
        if int(self.log.index("end-1c").split(".")[0]) > 200:
            self.log.delete("1.0", "50.0")

    def _on_close(self):
        try:
            self._send("STOP")
        except Exception:
            pass
        self.link.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()