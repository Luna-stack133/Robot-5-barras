"""Grabación de las corridas con IMU (etapa 1).

Cada corrida del robot real con la BNO055 se guarda sola en
    Descargas/Resultados_BNO055/etapa1/<fecha>_<hora>_<nombre>/
        imu.csv          una fila por muestra de la IMU (100 Hz) con los ángulos de los motores
        telemetria.csv   la telemetría STATE (posición del efector según los pasos)
        info.json        geometría, modo, velocidad, aceleración, la referencia de la figura y la
                         configuración de la corrección con IMU (firmware 1.3)
Esa carpeta es la que lee el script etapa1_analizar.py.

Con el firmware 1.3, imu.csv agrega las columnas del EKF y de la corrección (COLUMNAS_EKF).
Con el 1.4 agrega además el plan y lo que se sumó a los pulsos (COLUMNAS_PLAN).

El nombre de la carpeta termina según la condición:
    (nada) apagada · _ff prealimentación · _conIMU corrección activa · _ffIMU las dos ·
    _sombra / _ffsombra EKF en sombra;  y _c150g si hay una carga de 150 g en el efector.
"""
import csv
import json
import os
import re
import time

from . import config

COLUMNAS_IMU = ["t_s", "q1_deg", "q2_deg", "ax", "ay", "az", "gx", "gy", "gz",
                "rumbo", "roll", "pitch", "cal_sis", "cal_gir", "cal_acc", "cal_mag"]
# firmware 1.3: estimación del EKF y corrección calculada por la ley en cada muestra
COLUMNAS_EKF = ["q1_ekf_deg", "q2_ekf_deg", "w1_ekf_dps", "w2_ekf_dps", "d1_mNm", "d2_mNm",
                "dq1_ley_deg", "dq2_ley_deg", "inov_gz_dps", "ctl_modo", "ekf_valido", "ctl_aplicando"]


# firmware 1.4: plan en el instante de la muestra y lo que se sumó a los pulsos
COLUMNAS_PLAN = ["q1_plan_deg", "q2_plan_deg", "ff1_deg", "ff2_deg", "dq1_aplicada_deg", "dq2_aplicada_deg"]


def sufijo_condicion(info):
    """Sufijo de la carpeta según la corrección, la prealimentación y la carga."""
    ctl = (info or {}).get("correccion_imu") or {}
    modo, ff = ctl.get("modo"), bool(ctl.get("prealimentacion"))
    suf = {(1, False): "_sombra", (2, False): "_conIMU", (0, True): "_ff",
           (1, True): "_ffsombra", (2, True): "_ffIMU"}.get((modo, ff), "")
    carga = (info or {}).get("carga_g") or 0
    if carga:
        suf += "_c%dg" % round(carga)
    return suf


def carpeta_base():
    descargas = os.path.join(os.path.expanduser("~"), "Downloads")
    base = descargas if os.path.isdir(descargas) else os.getcwd()
    return os.path.join(base, config.IMU_CARPETA, "etapa1")


def _limpio(nombre):
    nombre = re.sub(r"[^\w\-]+", "_", nombre, flags=re.UNICODE).strip("_")
    return nombre[:40] or "corrida"


def guardar_corrida(run, base=None):
    """Guarda la corrida y devuelve la carpeta creada (o None si no tiene IMU)."""
    imu = run.get("imu")
    if not imu:
        return None
    base = base or carpeta_base()
    sufijo = sufijo_condicion(run.get("info"))
    carpeta = os.path.join(base, "%s_%s%s" % (time.strftime("%Y-%m-%d_%H-%M-%S"), _limpio(run["name"]), sufijo))
    os.makedirs(carpeta, exist_ok=True)

    con_ekf = all(len(s) >= 23 for s in imu)
    con_plan = con_ekf and all(len(s) >= 29 for s in imu)
    with open(os.path.join(carpeta, "imu.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLUMNAS_IMU + (COLUMNAS_EKF if con_ekf else []) + (COLUMNAS_PLAN if con_plan else []))
        for s in imu:
            t, q1, q2, ax, ay, az, gx, gy, gz, rumbo, roll, pitch, cal = s[:13]
            fila = ["%.5f" % t, "%.3f" % q1, "%.3f" % q2, "%.2f" % ax, "%.2f" % ay, "%.2f" % az,
                    "%.4f" % gx, "%.4f" % gy, "%.4f" % gz, "%.4f" % rumbo, "%.4f" % roll, "%.4f" % pitch,
                    (cal >> 6) & 3, (cal >> 4) & 3, (cal >> 2) & 3, cal & 3]
            if con_ekf:
                q1h, q2h, w1h, w2h, d1, d2, dl1, dl2, inov, est = s[13:23]
                fila += ["%.3f" % q1h, "%.3f" % q2h, "%.2f" % w1h, "%.2f" % w2h, "%.1f" % d1, "%.1f" % d2,
                         "%.3f" % dl1, "%.3f" % dl2, "%.2f" % inov, est & 3, (est >> 2) & 1, (est >> 3) & 1]
            if con_plan:
                fila += ["%.3f" % v for v in s[23:29]]
            w.writerow(fila)

    with open(os.path.join(carpeta, "telemetria.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_mm", "y_mm", "theta1_deg", "theta2_deg"])
        for s in run.get("samples") or []:
            w.writerow(["%.4f" % s[0], "%.3f" % s[1], "%.3f" % s[2], "%.4f" % s[3], "%.4f" % s[4]])

    info = dict(run.get("info") or {})
    info["referencia_mm"] = [[round(p[0], 3), round(p[1], 3)] for p in (run.get("reference") or [])]
    info["muestras_imu"] = len(imu)
    with open(os.path.join(carpeta, "info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    return carpeta
