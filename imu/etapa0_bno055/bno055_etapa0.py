"""Etapa 0 - BNO055: registrar datos de la IMU y caracterizar su ruido y su sesgo.

Uso:
    python bno055_etapa0.py puertos
    python bno055_etapa0.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_off.csv [--modo imu]
    python bno055_etapa0.py analizar reposo_servos_off.csv

Dependencias:  pip install pyserial numpy matplotlib

El programa de la ESP32 (bno055_etapa0.ino) envía enteros crudos de la BNO055;
este script los convierte a unidades físicas:
    acelerómetro, aceleración lineal y gravedad: 100 LSB = 1 m/s²
    giroscopio: 16 LSB = 1 °/s        magnetómetro: 16 LSB = 1 µT
    ángulos de Euler: 16 LSB = 1°     cuaternión: 16384 LSB = 1
"""
import argparse
import csv
import json
import math
import os
import sys
import time

G0 = 9.80665
COLUMNAS = ["n", "t_s",
            "ax", "ay", "az", "mx", "my", "mz", "gx", "gy", "gz",
            "rumbo", "roll", "pitch", "qw", "qx", "qy", "qz",
            "lx", "ly", "lz", "vx", "vy", "vz",
            "temp_c", "cal_sis", "cal_gir", "cal_acc", "cal_mag", "errores_i2c"]
ESCALAS = [100.0] * 3 + [16.0] * 3 + [16.0] * 3 + [16.0] * 3 + [16384.0] * 4 + [100.0] * 3 + [100.0] * 3
MODOS = {"imu": "i", "ndof": "n", "amg": "r"}

# Valores de referencia de la hoja de datos de la BNO055 (sensores BMA280 / BMG160)
REF_RUIDO_ACC_UG = 150.0      # µg/√Hz
REF_RUIDO_GIR = 0.014         # °/s/√Hz
REF_SESGO_GIR = 1.0           # ±°/s


# ============================================================================ serial
def cmd_puertos(_args):
    from serial.tools import list_ports
    ps = list(list_ports.comports())
    if not ps:
        print("No se encontraron puertos. Revise el cable USB.")
    for p in ps:
        vidpid = "%04X:%04X" % (p.vid, p.pid) if p.vid is not None else "----:----"
        print("%-10s %s  [%s]" % (p.device, p.description, vidpid))


def convertir(campos, t0_us, vuelta):
    """Línea 'D,...' ya separada -> fila en unidades físicas."""
    n = int(campos[1])
    t_us = int(campos[2]) + vuelta * 2 ** 32
    crudos = [int(v) for v in campos[3:25]]
    temp = int(campos[25])
    cal = int(campos[26])
    err = int(campos[27])
    fila = [n, (t_us - t0_us) * 1e-6]
    fila += [c / e for c, e in zip(crudos, ESCALAS)]
    fila += [temp, (cal >> 6) & 3, (cal >> 4) & 3, (cal >> 2) & 3, cal & 3, err]
    return fila


def cmd_registrar(args):
    import serial
    try:
        ser = serial.Serial(args.puerto, args.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit("No se pudo abrir %s: %s\n¿Está abierto el monitor serie del Arduino IDE?" % (args.puerto, e))
    time.sleep(2.0)                     # la placa puede reiniciarse al abrir el puerto
    ser.reset_input_buffer()
    ser.write(b"q\n")
    time.sleep(0.2)
    ser.write((MODOS[args.modo] + "\n").encode())
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"d\n")

    duracion = args.minutos * 60.0
    print("Registrando %.1f min en modo %s -> %s" % (args.minutos, args.modo.upper(), args.salida))
    print("No toquen la mesa ni el robot. Ctrl+C para terminar antes.")
    filas = 0
    perdidas = 0
    n_ant = None
    t0_us = None
    t_ant = None
    vuelta = 0
    inicio = time.time()
    ultimo_aviso = inicio
    with open(args.salida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNAS)
        try:
            while time.time() - inicio < duracion:
                linea = ser.readline().decode("ascii", errors="ignore").strip()
                if not linea.startswith("D,"):
                    if linea and not linea.startswith("#"):
                        print("  ESP32:", linea)
                    continue
                campos = linea.split(",")
                if len(campos) != 28:
                    continue
                t_raw = int(campos[2])
                if t_ant is not None and t_raw < t_ant:
                    vuelta += 1          # micros() dio la vuelta (cada 71 min)
                t_ant = t_raw
                if t0_us is None:
                    t0_us = t_raw
                fila = convertir(campos, t0_us, vuelta)
                if n_ant is not None and fila[0] != n_ant + 1:
                    perdidas += max(0, fila[0] - n_ant - 1)
                n_ant = fila[0]
                w.writerow(["%.6g" % v if isinstance(v, float) else v for v in fila])
                filas += 1
                if time.time() - ultimo_aviso >= 10:
                    ultimo_aviso = time.time()
                    print("  %5.0f s | %7d muestras | perdidas %d | T=%d C | calibracion sis/gir/acc/mag = %d/%d/%d/%d"
                          % (time.time() - inicio, filas, perdidas, fila[24], fila[25], fila[26], fila[27], fila[28]))
        except KeyboardInterrupt:
            print("\nDetenido por el usuario.")
    ser.write(b"q\n")
    ser.close()
    print("Listo: %d muestras, %d perdidas. Ahora: python %s analizar %s"
          % (filas, perdidas, os.path.basename(sys.argv[0]), args.salida))


# ============================================================================ análisis
def allan(x, fs, puntos=60):
    """Desviación de Allan con solapamiento. Devuelve (tau, sigma)."""
    import numpy as np
    x = np.asarray(x, float)
    n = len(x)
    m_max = max(1, n // 9)
    ms = np.unique(np.logspace(0, math.log10(m_max), puntos).astype(int))
    theta = np.concatenate(([0.0], np.cumsum(x))) / fs
    taus, sig = [], []
    for m in ms:
        tau = m / fs
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        if len(d) < 2:
            continue
        taus.append(tau)
        sig.append(math.sqrt(np.sum(d * d) / (2 * tau * tau * len(d))))
    return np.array(taus), np.array(sig)


def psd_welch(x, fs, nseg=1024):
    """Densidad espectral de potencia unilateral (Welch, ventana Hann)."""
    import numpy as np
    x = np.asarray(x, float) - np.mean(x)
    nseg = min(nseg, len(x))
    paso = nseg // 2
    win = np.hanning(nseg)
    escala = 1.0 / (fs * np.sum(win ** 2))
    acc = None
    k = 0
    for i in range(0, len(x) - nseg + 1, paso):
        seg = x[i:i + nseg] - np.mean(x[i:i + nseg])
        p = np.abs(np.fft.rfft(seg * win)) ** 2 * escala
        acc = p if acc is None else acc + p
        k += 1
    p = acc / k
    p[1:-1] *= 2
    f = np.fft.rfftfreq(nseg, 1 / fs)
    return f, p


def valor_en_tau(taus, sig, tau0):
    import numpy as np
    if tau0 < taus[0] or tau0 > taus[-1]:
        return float("nan")
    return float(np.exp(np.interp(math.log(tau0), np.log(taus), np.log(sig))))


def cmd_analizar(args):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    datos = np.genfromtxt(args.archivo, delimiter=",", names=True)
    if datos.size < 200:
        sys.exit("El archivo tiene muy pocas muestras para analizar.")
    base = os.path.splitext(args.archivo)[0]
    t = datos["t_s"]
    dt = np.diff(t)
    fs = 1.0 / np.median(dt)
    duracion = t[-1] - t[0]
    n_ids = datos["n"]
    perdidas = int(np.sum(np.clip(np.diff(n_ids) - 1, 0, None)))
    huecos = int(np.sum(dt > 1.5 / fs))
    repetidas = int(np.sum(np.all(np.diff(np.column_stack([datos[c] for c in ("ax", "ay", "az", "gx", "gy", "gz")]),
                                          axis=0) == 0, axis=1)))
    errores_i2c = int(datos["errores_i2c"][-1])

    rep = {"archivo": os.path.basename(args.archivo), "muestras": int(datos.size), "duracion_s": round(float(duracion), 1),
           "frecuencia_hz": round(float(fs), 2), "muestras_perdidas": perdidas, "huecos": huecos,
           "muestras_repetidas": repetidas, "errores_i2c": errores_i2c}

    print("\n=== %s ===" % rep["archivo"])
    print("Muestras: %d en %.1f s  (%.2f Hz)" % (datos.size, duracion, fs))
    print("Perdidas: %d | huecos: %d | repetidas: %d (%.1f %%) | errores I2C: %d"
          % (perdidas, huecos, repetidas, 100.0 * repetidas / datos.size, errores_i2c))

    # --- acelerómetro: magnitud e inclinación
    ax, ay, az = datos["ax"], datos["ay"], datos["az"]
    mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
    roll = math.degrees(math.atan2(np.mean(ay), np.mean(az)))
    pitch = math.degrees(math.atan2(-np.mean(ax), math.hypot(np.mean(ay), np.mean(az))))
    rep["acel_magnitud_ms2"] = round(float(np.mean(mag)), 4)
    rep["inclinacion_deg"] = {"roll": round(roll, 2), "pitch": round(pitch, 2)}
    print("\nAcelerometro: |a| = %.3f m/s2 (%.3f g) | inclinacion roll %.2f deg, pitch %.2f deg"
          % (np.mean(mag), np.mean(mag) / G0, roll, pitch))

    # --- por eje
    ejes = {"gx": "°/s", "gy": "°/s", "gz": "°/s", "ax": "m/s²", "ay": "m/s²", "az": "m/s²"}
    rep["ejes"] = {}
    allans = {}
    psds = {}
    print("\n%-4s %14s %14s %16s %18s %16s" % ("eje", "media", "desv. est.", "dens. ruido", "ref. hoja datos", "inestab. sesgo"))
    for c, u in ejes.items():
        x = datos[c]
        media = float(np.mean(x))
        sd = float(np.std(x - media))
        tau, sig = allan(x, fs)
        allans[c] = (tau, sig)
        f, p = psd_welch(x, fs)
        psds[c] = (f, p)
        banda = (f >= 0.5) & (f <= min(10.0, fs / 4))
        dens_psd = float(np.sqrt(np.median(p[banda]))) if np.any(banda) else float("nan")
        dens_allan = valor_en_tau(tau, sig, 1.0)          # σ(τ=1 s) ≈ densidad de ruido blanco
        inestab = float(np.min(sig) / 0.664)
        tau_min = float(tau[np.argmin(sig)])
        if c.startswith("g"):
            texto_dens = "%.4f °/s/√Hz" % dens_psd
            ref = "%.3f °/s/√Hz" % REF_RUIDO_GIR
            texto_inest = "%.4f °/s" % inestab
        else:
            texto_dens = "%.0f µg/√Hz" % (dens_psd / G0 * 1e6)
            ref = "%.0f µg/√Hz" % REF_RUIDO_ACC_UG
            texto_inest = "%.0f µg" % (inestab / G0 * 1e6)
        print("%-4s %9.4f %-4s %9.4f %-4s %16s %18s %16s" % (c, media, u, sd, u, texto_dens, ref, texto_inest))
        rep["ejes"][c] = {"unidad": u, "media": media, "desv_est": sd, "varianza": sd * sd,
                          "densidad_ruido_psd": dens_psd, "allan_tau1s": dens_allan,
                          "inestabilidad_sesgo": inestab, "tau_inestabilidad_s": tau_min}

    # --- matriz R para el EKF (SI)
    d2r = math.pi / 180.0
    rep["R_ekf"] = {
        "giroscopio_rad2_s2": [rep["ejes"][c]["varianza"] * d2r * d2r for c in ("gx", "gy", "gz")],
        "acelerometro_m2_s4": [rep["ejes"][c]["varianza"] for c in ("ax", "ay", "az")],
        "nota": "Varianzas a %.0f Hz con el sensor quieto. Para otra frecuencia de muestreo escalen con la densidad de ruido." % fs,
    }
    rep["sesgo_giroscopio_dps"] = [rep["ejes"][c]["media"] for c in ("gx", "gy", "gz")]

    # --- deriva térmica del giroscopio
    temp = datos["temp_c"]
    rep["temperatura_c"] = {"min": float(np.min(temp)), "max": float(np.max(temp))}
    if np.ptp(temp) >= 2:
        k = np.polyfit(temp, datos["gz"], 1)[0]
        rep["deriva_termica_gz_dps_por_c"] = float(k)
        print("\nDeriva termica de gz: %.4f °/s por °C (temperatura %.0f a %.0f C)" % (k, np.min(temp), np.max(temp)))
    else:
        print("\nTemperatura casi constante (%.0f a %.0f C): no se puede estimar la deriva termica."
              % (np.min(temp), np.max(temp)))

    # --- rumbo de la fusión (solo modos IMU/NDOF)
    if np.any(datos["qw"] != 0):
        rumbo = np.degrees(np.unwrap(np.radians(datos["rumbo"])))
        pend = np.polyfit(t / 60.0, rumbo, 1)[0]
        rep["deriva_rumbo_deg_por_min"] = float(pend)
        print("Deriva del rumbo de la fusion en reposo: %.3f grados/min" % pend)

    # --- calibración
    fin = {k: int(datos[k][-1]) for k in ("cal_sis", "cal_gir", "cal_acc", "cal_mag")}
    rep["calibracion_final"] = fin
    print("Calibracion al final (0-3): sistema %d, giroscopio %d, acelerometro %d, magnetometro %d"
          % (fin["cal_sis"], fin["cal_gir"], fin["cal_acc"], fin["cal_mag"]))

    with open(base + "_calibracion.json", "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)

    # --- gráficas
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for c in ("gx", "gy", "gz"):
        axs[0].plot(t, datos[c], lw=0.5, label=c)
    axs[0].set_ylabel("°/s")
    for c in ("ax", "ay"):
        axs[1].plot(t, datos[c], lw=0.5, label=c)
    axs[1].set_ylabel("m/s²")
    axs[2].plot(t, datos["az"], lw=0.5, label="az")
    axs[2].set_ylabel("m/s²")
    axs[2].set_xlabel("tiempo [s]")
    for a in axs:
        a.legend(loc="upper right")
        a.grid(alpha=0.3)
    fig.suptitle("Series en reposo · " + rep["archivo"])
    fig.tight_layout()
    fig.savefig(base + "_series.png", dpi=120)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    for c in ("gx", "gy", "gz"):
        f, p = psds[c]
        axs[0].loglog(f[1:], np.sqrt(p[1:]), lw=0.7, label=c)
    axs[0].axhline(REF_RUIDO_GIR, color="k", ls="--", lw=0.8, label="hoja de datos")
    axs[0].set_title("Giroscopio")
    axs[0].set_ylabel("°/s/√Hz")
    for c in ("ax", "ay", "az"):
        f, p = psds[c]
        axs[1].loglog(f[1:], np.sqrt(p[1:]) / G0 * 1e6, lw=0.7, label=c)
    axs[1].axhline(REF_RUIDO_ACC_UG, color="k", ls="--", lw=0.8, label="hoja de datos")
    axs[1].set_title("Acelerómetro")
    axs[1].set_ylabel("µg/√Hz")
    for a in axs:
        a.set_xlabel("frecuencia [Hz]")
        a.legend()
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(base + "_psd.png", dpi=120)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    for c in ("gx", "gy", "gz"):
        tau, sig = allans[c]
        axs[0].loglog(tau, sig, "o-", ms=2.5, lw=0.8, label=c)
    axs[0].set_title("Desviación de Allan · giroscopio")
    axs[0].set_ylabel("°/s")
    for c in ("ax", "ay", "az"):
        tau, sig = allans[c]
        axs[1].loglog(tau, sig / G0 * 1e6, "o-", ms=2.5, lw=0.8, label=c)
    axs[1].set_title("Desviación de Allan · acelerómetro")
    axs[1].set_ylabel("µg")
    for a in axs:
        a.set_xlabel("τ [s]")
        a.legend()
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(base + "_allan.png", dpi=120)

    fig, ax1 = plt.subplots(figsize=(11, 4))
    ax1.plot(t, datos["gz"], lw=0.4, color="tab:blue")
    ax1.set_ylabel("gz [°/s]", color="tab:blue")
    ax2 = ax1.twinx()
    ax2.plot(t, temp, color="tab:red")
    ax2.set_ylabel("temperatura [°C]", color="tab:red")
    ax1.set_xlabel("tiempo [s]")
    fig.tight_layout()
    fig.savefig(base + "_temperatura.png", dpi=120)

    print("\nGuardado: %s_calibracion.json y las graficas _series, _psd, _allan, _temperatura (.png)" % base)


def main():
    try:  # consolas de Windows sin UTF-8: no fallar por los símbolos °, µ, √
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    p = argparse.ArgumentParser(description="Etapa 0 de la BNO055")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("puertos", help="listar puertos serie")
    r = sub.add_parser("registrar", help="registrar datos de la ESP32 en un CSV")
    r.add_argument("--puerto", required=True)
    r.add_argument("--minutos", type=float, default=10.0)
    r.add_argument("--salida", required=True)
    r.add_argument("--modo", choices=sorted(MODOS), default="imu")
    r.add_argument("--baud", type=int, default=921600)
    a = sub.add_parser("analizar", help="analizar un CSV registrado")
    a.add_argument("archivo")
    args = p.parse_args()
    {"puertos": cmd_puertos, "registrar": cmd_registrar, "analizar": cmd_analizar}[args.cmd](args)


if __name__ == "__main__":
    main()
