#!/usr/bin/env python3
"""
Etapa 0 - Registro y analisis de la MPU6050 en reposo
Robot paralelo de 5 barras - Tecnicas de Control

Usos:
  python etapa0_imu.py puertos
      Lista los puertos seriales disponibles.

  python etapa0_imu.py registrar --puerto COM5 --minutos 10 --salida reposo_servos_off.csv
      Le pide a la ESP32 el modo CSV (comando 'c') y guarda los datos.
      Se puede detener antes con Ctrl+C: lo registrado hasta ese momento se guarda.

  python etapa0_imu.py analizar reposo_servos_off.csv [--descartar 30] [--mostrar]
      Calcula sesgos, ruido, densidad espectral, desviacion de Allan y deriva
      con la temperatura. Guarda un JSON con la calibracion y las graficas.

Requisitos:  pip install pyserial numpy matplotlib
"""

import argparse
import io
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

# Valores tipicos de la hoja de datos de la MPU6050 (para comparar)
REF_GIRO_MDPS_RTHZ = 5.0     # 0.005 deg/s/sqrt(Hz)
REF_ACEL_UG_RTHZ = 400.0     # 400 ug/sqrt(Hz)
G = 9.80665
COLUMNAS = ["n", "t_us", "ax", "ay", "az", "temp", "gx", "gy", "gz", "perdidas", "err_i2c"]


# =============================================================================
# Utilidades
# =============================================================================
def ok(cond):
    return "[OK]     " if cond else "[REVISAR]"


def listar_puertos():
    from serial.tools import list_ports
    puertos = list(list_ports.comports())
    if not puertos:
        print("No se encontraron puertos seriales. Conecten la ESP32 por USB.")
        return
    for p in puertos:
        print(f"  {p.device:12s} {p.description}")


# =============================================================================
# Registro por serial
# =============================================================================
def registrar(puerto, baudios, segundos, salida):
    import serial

    salida = Path(salida)
    print(f"Abriendo {puerto} a {baudios} baudios...")
    ser = serial.Serial(puerto, baudios, timeout=1)

    # Al abrir el puerto, algunas placas se reinician. Se muestran los mensajes de
    # arranque (lineas con '#') hasta que el puerto quede en silencio.
    print("Esperando el arranque de la ESP32...")
    limite = time.time() + 8
    ultimo = time.time()
    while time.time() < limite:
        linea = ser.readline().decode("ascii", errors="replace").strip()
        if linea:
            ultimo = time.time()
            if linea.startswith("#"):
                print("  " + linea)
        elif time.time() - ultimo > 1.5:
            break

    ser.write(b"s\n")
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"c\n")

    # Esperar la linea de configuracion y el encabezado
    lineas_cfg = []
    limite = time.time() + 5
    encabezado = None
    while time.time() < limite:
        linea = ser.readline().decode("ascii", errors="replace").strip()
        if linea.startswith("# cfg"):
            lineas_cfg.append(linea)
        elif linea.startswith("n,t_us"):
            encabezado = linea
            break
    if encabezado is None:
        ser.close()
        sys.exit("No llego el encabezado CSV. Verifiquen que el sketch imu_etapa0 este cargado, "
                 "el puerto y los baudios (921600).")

    print(f"Registrando {segundos/60:.1f} min en {salida} (Ctrl+C para detener antes)...")
    n_ok = n_malas = 0
    ultimo_campos = None
    t0 = time.time()
    proximo_reporte = t0 + 5
    try:
        with open(salida, "w", newline="") as f:
            f.write(f"# registro {datetime.now().isoformat(timespec='seconds')} puerto={puerto}\n")
            for c in lineas_cfg:
                f.write(c + "\n")
            f.write(encabezado + "\n")
            while time.time() - t0 < segundos:
                linea = ser.readline().decode("ascii", errors="replace").strip()
                if not linea:
                    continue
                campos = linea.split(",")
                if len(campos) != len(COLUMNAS) or not all(c.lstrip("-").isdigit() for c in campos):
                    n_malas += 1
                    continue
                f.write(linea + "\n")
                n_ok += 1
                ultimo_campos = campos
                if time.time() >= proximo_reporte:
                    dt = time.time() - t0
                    print(f"  t={dt:6.0f} s | muestras={n_ok:7d} | fs~{n_ok/dt:6.1f} Hz | "
                          f"perdidas={ultimo_campos[9]} | errores I2C={ultimo_campos[10]}")
                    proximo_reporte += 5
    except KeyboardInterrupt:
        print("\nDetenido por el usuario. Se guarda lo registrado.")
    finally:
        try:
            ser.write(b"s\n")
        except Exception:
            pass
        ser.close()

    print(f"\nListo: {n_ok} muestras guardadas en {salida}")
    if n_malas:
        print(f"  Aviso: {n_malas} lineas descartadas por estar incompletas o corruptas.")
    if ultimo_campos is not None and (int(ultimo_campos[9]) > 0 or int(ultimo_campos[10]) > 0):
        print(f"  Aviso: perdidas={ultimo_campos[9]}, errores I2C={ultimo_campos[10]}. "
              "Revisen el cableado y que no haya otros programas usando el puerto.")
    print(f"Siguiente paso:  python {Path(__file__).name} analizar {salida}")


# =============================================================================
# Carga del CSV
# =============================================================================
def cargar_csv(archivo):
    cfg = {}
    filas = []
    with open(archivo, "r", errors="replace") as f:
        for linea in f:
            linea = linea.strip()
            if not linea:
                continue
            if linea.startswith("# cfg"):
                for par in linea[5:].split():
                    if "=" in par:
                        k, v = par.split("=", 1)
                        cfg[k] = v
                continue
            if linea.startswith("#") or linea.startswith("n,"):
                continue
            filas.append(linea)
    if not filas:
        sys.exit(f"{archivo} no tiene datos.")
    datos = np.loadtxt(io.StringIO("\n".join(filas)), delimiter=",", dtype=np.int64)
    if datos.ndim == 1:
        datos = datos[None, :]
    d = {nombre: datos[:, i] for i, nombre in enumerate(COLUMNAS)}

    fs_cfg = float(cfg.get("fs_hz", 200.0))
    giro_lsb = float(cfg.get("giro_lsb_dps", 131.0))
    acel_lsb = float(cfg.get("acel_lsb_g", 16384.0))
    who = int(cfg.get("who_am_i", "0x68"), 16)
    familia6500 = who in (0x70, 0x71, 0x73)

    # Tiempo en segundos (se corrige el desborde de micros() cada ~71 minutos)
    dt_us = np.diff(d["t_us"])
    dt_us[dt_us < 0] += 2**32
    t = np.concatenate([[0], np.cumsum(dt_us)]) / 1e6

    out = {
        "t": t,
        "acel_g": np.column_stack([d["ax"], d["ay"], d["az"]]) / acel_lsb,
        "giro_dps": np.column_stack([d["gx"], d["gy"], d["gz"]]) / giro_lsb,
        "temp_c": d["temp"] / 333.87 + 21.0 if familia6500 else d["temp"] / 340.0 + 36.53,
        "perdidas": int(d["perdidas"][-1] - d["perdidas"][0]),
        "err_i2c": int(d["err_i2c"][-1] - d["err_i2c"][0]),
        "fs_cfg": fs_cfg,
        "cfg": cfg,
        "who": who,
    }
    return out


# =============================================================================
# Herramientas de analisis
# =============================================================================
def psd_welch(x, fs, nperseg):
    """Densidad espectral de potencia unilateral (metodo de Welch, ventana de Hann)."""
    x = np.asarray(x, dtype=float)
    nperseg = min(nperseg, len(x))
    paso = nperseg // 2
    w = np.hanning(nperseg)
    escala = 1.0 / (fs * np.sum(w**2))
    acumulado = np.zeros(nperseg // 2 + 1)
    k = 0
    for ini in range(0, len(x) - nperseg + 1, paso):
        seg = x[ini:ini + nperseg]
        seg = seg - seg.mean()
        espectro = np.fft.rfft(seg * w)
        acumulado += np.abs(espectro) ** 2
        k += 1
    psd = acumulado / max(k, 1) * escala
    psd[1:-1] *= 2.0
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return f, psd


def densidad_banda(f, psd, f_min, f_max):
    sel = (f >= f_min) & (f <= f_max)
    return float(np.sqrt(np.median(psd[sel])))


def allan(y, fs, n_puntos=60):
    """Desviacion de Allan solapada de una senal de velocidad (p. ej. giroscopio)."""
    y = np.asarray(y, dtype=float)
    N = len(y)
    theta = np.cumsum(y) / fs
    m_max = max(1, N // 9)
    ms = np.unique(np.logspace(0, np.log10(m_max), n_puntos).astype(int))
    taus, adev = [], []
    for m in ms:
        if 2 * m >= N:
            break
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        tau = m / fs
        avar = np.sum(d**2) / (2 * tau**2 * (N - 2 * m))
        taus.append(tau)
        adev.append(np.sqrt(avar))
    return np.array(taus), np.array(adev)


# =============================================================================
# Analisis
# =============================================================================
def analizar(archivo, descartar, mostrar, sin_graficas):
    archivo = Path(archivo)
    d = cargar_csv(archivo)
    t, acel, giro, temp = d["t"], d["acel_g"], d["giro_dps"], d["temp_c"]

    dt_total = np.diff(t)
    fs = 1.0 / np.median(dt_total)
    huecos = int(np.sum(dt_total > 1.5 / fs))
    jitter_us = float(np.std(dt_total) * 1e6)

    if descartar > 0:
        sel = t >= t[0] + descartar
        t, acel, giro, temp = t[sel], acel[sel], giro[sel], temp[sel]
        t = t - t[0]
    dur = float(t[-1] - t[0])
    if dur < 30:
        sys.exit("El registro es muy corto para analizarlo (menos de 30 s). Registren al menos 5 minutos.")

    ejes = ["x", "y", "z"]
    media_a = acel.mean(axis=0)
    std_a = acel.std(axis=0)
    media_g = giro.mean(axis=0)
    std_g = giro.std(axis=0)
    norma = float(np.mean(np.linalg.norm(acel, axis=1)))
    roll = float(np.degrees(np.arctan2(media_a[1], media_a[2])))
    pitch = float(np.degrees(np.arctan2(-media_a[0], np.hypot(media_a[1], media_a[2]))))

    # Densidad espectral de ruido entre 0.5 y 20 Hz (por debajo del filtro de ~44 Hz)
    nperseg = int(2 ** np.floor(np.log2(min(4096, len(t) // 8))))
    psd_a, psd_g = [], []
    dens_a, dens_g = [], []
    for i in range(3):
        f, p = psd_welch(acel[:, i], fs, nperseg)
        psd_a.append(p)
        dens_a.append(densidad_banda(f, p, 0.5, 20.0) * 1e6)   # ug/sqrt(Hz)
        f, p = psd_welch(giro[:, i], fs, nperseg)
        psd_g.append(p)
        dens_g.append(densidad_banda(f, p, 0.5, 20.0) * 1e3)   # mdps/sqrt(Hz)

    # Desviacion de Allan del giroscopio
    allan_res = []
    for i in range(3):
        taus, adev = allan(giro[:, i], fs)
        sigma1 = float(np.interp(0.0, np.log10(taus), np.log10(adev)))   # en tau = 1 s
        sigma1 = 10 ** sigma1
        i_min = int(np.argmin(adev))
        allan_res.append({
            "taus": taus, "adev": adev,
            "arw_deg_rth": sigma1 * 60.0,                       # deg/sqrt(h)
            "bi_deg_h": float(adev[i_min] / 0.664 * 3600.0),     # deg/h
            "tau_min_s": float(taus[i_min]),
        })

    # Deriva del sesgo con la temperatura
    rango_temp = float(temp.max() - temp.min())
    deriva = None
    if rango_temp >= 0.5:
        deriva = [float(np.polyfit(temp, giro[:, i], 1)[0]) for i in range(3)]

    # ------------------------------------------------------------------ reporte
    print()
    print("=" * 72)
    print(f" Analisis de {archivo.name}")
    print("=" * 72)
    print(f" Duracion analizada: {dur/60:.1f} min   |  Muestreo efectivo: {fs:.1f} Hz "
          f"(configurado {d['fs_cfg']:.0f} Hz)")
    print(f" WHO_AM_I: 0x{d['who']:02X}   |  Lectura: {d['cfg'].get('lectura', '?')}   |  "
          f"Jitter del muestreo: {jitter_us:.0f} us")
    print(f" {ok(d['perdidas'] == 0 and huecos == 0)} Muestras perdidas: {d['perdidas']}  |  "
          f"huecos en el tiempo: {huecos}  |  errores I2C: {d['err_i2c']}")
    print(f" Temperatura: {temp[0]:.1f} -> {temp[-1]:.1f} C (rango {rango_temp:.1f} C)")
    print()
    print(" ACELEROMETRO")
    print(f"   Media [g]:                 x={media_a[0]:+.4f}  y={media_a[1]:+.4f}  z={media_a[2]:+.4f}")
    print(f"   Desv. estandar [mg]:       x={std_a[0]*1e3:.2f}   y={std_a[1]*1e3:.2f}   z={std_a[2]*1e3:.2f}")
    print(f"   Densidad de ruido [ug/rtHz]: x={dens_a[0]:.0f}  y={dens_a[1]:.0f}  z={dens_a[2]:.0f}"
          f"   (hoja de datos: {REF_ACEL_UG_RTHZ:.0f})")
    print(f" {ok(0.92 < norma < 1.08)} Magnitud media |a| = {norma:.4f} g (debe ser cercana a 1)")
    print(f" {ok(abs(roll) < 3 and abs(pitch) < 3)} Inclinacion: roll={roll:+.2f} deg, pitch={pitch:+.2f} deg "
          "(cerca de 0 si la IMU esta nivelada)")
    if media_a[2] < -0.5:
        print("   Nota: az es negativa: la IMU esta montada boca abajo. No es un error, "
              "pero hay que tenerlo en cuenta en los signos.")
    print()
    print(" GIROSCOPIO")
    print(f"   Sesgo en reposo [deg/s]:   x={media_g[0]:+.3f}  y={media_g[1]:+.3f}  z={media_g[2]:+.3f}")
    print(f"   Desv. estandar [deg/s]:    x={std_g[0]:.4f}  y={std_g[1]:.4f}  z={std_g[2]:.4f}")
    print(f"   Densidad de ruido [mdps/rtHz]: x={dens_g[0]:.1f}  y={dens_g[1]:.1f}  z={dens_g[2]:.1f}"
          f"   (hoja de datos: {REF_GIRO_MDPS_RTHZ:.0f})")
    print("   Allan:  ARW [deg/rt h]      " + "  ".join(f"{e}={r['arw_deg_rth']:.3f}" for e, r in zip(ejes, allan_res)))
    print("           Inestab. sesgo [deg/h] " + "  ".join(f"{e}={r['bi_deg_h']:.1f}" for e, r in zip(ejes, allan_res)))
    if deriva is not None:
        print("   Deriva termica [deg/s/C]:  " + "  ".join(f"{e}={v:+.4f}" for e, v in zip(ejes, deriva)))
    else:
        print("   Deriva termica: la temperatura vario menos de 0.5 C; no se puede estimar.")
    peor_a = max(dens_a) / REF_ACEL_UG_RTHZ
    peor_g = max(dens_g) / REF_GIRO_MDPS_RTHZ
    print()
    print(f" {ok(peor_a < 3)} Ruido del acelerometro: {peor_a:.1f} veces el valor de la hoja de datos (peor eje)")
    print(f" {ok(peor_g < 3)} Ruido del giroscopio:   {peor_g:.1f} veces el valor de la hoja de datos (peor eje)")
    if peor_a >= 3 or peor_g >= 3:
        print("   Un ruido mucho mayor al de la hoja de datos suele indicar vibracion (servos energizados,")
        print("   bomba, mesa) o ruido electrico en la alimentacion. Comparen con un registro con todo apagado.")

    # ------------------------------------------------------------------ JSON
    var_g_rad = (np.radians(std_g)) ** 2
    var_a_ms2 = (std_a * G) ** 2
    calib = {
        "archivo": archivo.name,
        "fecha_analisis": datetime.now().isoformat(timespec="seconds"),
        "duracion_s": round(dur, 1),
        "fs_hz": round(float(fs), 2),
        "who_am_i": f"0x{d['who']:02X}",
        "dlpf_cfg": d["cfg"].get("dlpf_cfg"),
        "muestras": int(len(t)),
        "muestras_perdidas": d["perdidas"],
        "huecos_tiempo": huecos,
        "jitter_us": round(jitter_us, 1),
        "temperatura_c": {"inicio": round(float(temp[0]), 2), "fin": round(float(temp[-1]), 2),
                          "min": round(float(temp.min()), 2), "max": round(float(temp.max()), 2)},
        "giroscopio": {
            "sesgo_dps": [round(float(v), 5) for v in media_g],
            "sesgo_rad_s": [round(float(v), 7) for v in np.radians(media_g)],
            "desv_est_dps": [round(float(v), 5) for v in std_g],
            "R_varianza_rad2_s2": [float(f"{v:.4e}") for v in var_g_rad],
            "densidad_ruido_mdps_rtHz": [round(v, 2) for v in dens_g],
            "allan_arw_deg_rth": [round(r["arw_deg_rth"], 4) for r in allan_res],
            "allan_inestabilidad_sesgo_deg_h": [round(r["bi_deg_h"], 2) for r in allan_res],
            "allan_tau_minimo_s": [round(r["tau_min_s"], 2) for r in allan_res],
            "deriva_termica_dps_por_C": None if deriva is None else [round(v, 5) for v in deriva],
        },
        "acelerometro": {
            "media_g": [round(float(v), 5) for v in media_a],
            "desv_est_g": [round(float(v), 6) for v in std_a],
            "R_varianza_m2_s4": [float(f"{v:.4e}") for v in var_a_ms2],
            "densidad_ruido_ug_rtHz": [round(v, 1) for v in dens_a],
            "norma_media_g": round(norma, 5),
            "inclinacion_deg": {"roll": round(roll, 3), "pitch": round(pitch, 3)},
        },
        "referencia_hoja_datos": {"giro_mdps_rtHz": REF_GIRO_MDPS_RTHZ, "acel_ug_rtHz": REF_ACEL_UG_RTHZ},
    }
    ruta_json = archivo.with_name(archivo.stem + "_calibracion.json")
    with open(ruta_json, "w", encoding="utf-8") as fj:
        json.dump(calib, fj, indent=2, ensure_ascii=False)
    print()
    print(f" Calibracion guardada en: {ruta_json}")

    # ------------------------------------------------------------------ graficas
    if sin_graficas:
        return
    import matplotlib
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    base = archivo.with_name(archivo.stem)
    paso = max(1, len(t) // 20000)   # se diezma solo para graficar

    fig, ax = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for i, e in enumerate(ejes):
        ax[i, 0].plot(t[::paso], (acel[::paso, i] - media_a[i]) * 1e3, lw=0.5)
        ax[i, 0].set_ylabel(f"a{e} - media [mg]")
        ax[i, 1].plot(t[::paso], giro[::paso, i], lw=0.5, color="tab:orange")
        ax[i, 1].set_ylabel(f"g{e} [deg/s]")
        ax[i, 0].grid(alpha=0.3)
        ax[i, 1].grid(alpha=0.3)
    ax[2, 0].set_xlabel("tiempo [s]")
    ax[2, 1].set_xlabel("tiempo [s]")
    ax[0, 0].set_title("Acelerometro (sin la media)")
    ax[0, 1].set_title("Giroscopio")
    fig.suptitle(f"Series en reposo - {archivo.name}")
    fig.tight_layout()
    fig.savefig(f"{base}_series.png", dpi=130)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    for i, e in enumerate(ejes):
        ax[0].loglog(f[1:], np.sqrt(psd_a[i][1:]) * 1e6, lw=0.8, label=f"a{e}")
        ax[1].loglog(f[1:], np.sqrt(psd_g[i][1:]) * 1e3, lw=0.8, label=f"g{e}")
    ax[0].axhline(REF_ACEL_UG_RTHZ, color="k", ls="--", lw=1, label="hoja de datos")
    ax[1].axhline(REF_GIRO_MDPS_RTHZ, color="k", ls="--", lw=1, label="hoja de datos")
    for a, u in zip(ax, ["ug/sqrt(Hz)", "mdps/sqrt(Hz)"]):
        a.axvline(44, color="gray", ls=":", lw=1)
        a.set_xlabel("frecuencia [Hz]")
        a.set_ylabel(u)
        a.grid(alpha=0.3, which="both")
        a.legend(fontsize=8)
    ax[0].set_title("Densidad de ruido - acelerometro")
    ax[1].set_title("Densidad de ruido - giroscopio")
    fig.tight_layout()
    fig.savefig(f"{base}_psd.png", dpi=130)

    fig, ax = plt.subplots(figsize=(7, 5))
    for e, r in zip(ejes, allan_res):
        ax.loglog(r["taus"], r["adev"], marker=".", lw=1, label=f"g{e}")
    ax.set_xlabel("tau [s]")
    ax.set_ylabel("desviacion de Allan [deg/s]")
    ax.set_title("Desviacion de Allan - giroscopio")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"{base}_allan.png", dpi=130)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    ax[0].plot(t[::paso], temp[::paso], lw=0.8, color="tab:red")
    ax[0].set_xlabel("tiempo [s]")
    ax[0].set_ylabel("temperatura [C]")
    ax[0].set_title("Temperatura del sensor")
    ax[0].grid(alpha=0.3)
    ax[1].plot(temp[::paso], giro[::paso, 2], ".", ms=1, alpha=0.4)
    if deriva is not None:
        c = np.polyfit(temp, giro[:, 2], 1)
        tt = np.linspace(temp.min(), temp.max(), 10)
        ax[1].plot(tt, np.polyval(c, tt), "k", lw=1.5, label=f"{c[0]:+.4f} deg/s/C")
        ax[1].legend()
    ax[1].set_xlabel("temperatura [C]")
    ax[1].set_ylabel("gz [deg/s]")
    ax[1].set_title("Sesgo del giroscopio Z vs temperatura")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{base}_temperatura.png", dpi=130)

    print(f" Graficas guardadas: {base.name}_series.png, _psd.png, _allan.png, _temperatura.png")
    if mostrar:
        plt.show()


# =============================================================================
def main():
    p = argparse.ArgumentParser(description="Etapa 0: registro y analisis de la MPU6050")
    sub = p.add_subparsers(dest="comando", required=True)

    sub.add_parser("puertos", help="listar puertos seriales")

    r = sub.add_parser("registrar", help="registrar datos de la ESP32 en un CSV")
    r.add_argument("--puerto", required=True, help="p. ej. COM5 en Windows o /dev/ttyACM0 en Linux")
    r.add_argument("--baudios", type=int, default=921600)
    r.add_argument("--minutos", type=float, default=10.0)
    r.add_argument("--salida", default="reposo.csv")

    a = sub.add_parser("analizar", help="analizar un CSV registrado en reposo")
    a.add_argument("archivo")
    a.add_argument("--descartar", type=float, default=0.0, help="segundos iniciales a descartar")
    a.add_argument("--mostrar", action="store_true", help="abrir las graficas en ventanas")
    a.add_argument("--sin-graficas", action="store_true")

    args = p.parse_args()
    if args.comando == "puertos":
        listar_puertos()
    elif args.comando == "registrar":
        registrar(args.puerto, args.baudios, args.minutos * 60.0, args.salida)
    elif args.comando == "analizar":
        analizar(args.archivo, args.descartar, args.mostrar, args.sin_graficas)


if __name__ == "__main__":
    main()
