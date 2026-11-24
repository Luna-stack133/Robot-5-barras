"""Genera data/runs.csv (una fila por corrida) y data/exclusions.csv (log de exclusiones).

Uso: python tabla_corridas.py   (requiere results/ de analisis_imu.py, analisis_video.py y tray_video.py)
"""
from rutas import *
import csv, json, os, re
from indice import indice

RAIZ_DATA = os.path.join(RAIZ, "data")
SES_DOE = {"S2": 1, "S3": 2}       # sesiones del DOE (bloques); S1 = piloto/calibración, S4 = diagnóstico
POS_XY = {"A": (75, 280), "B": (170, 220), "C": (75, 200), "D": (0, 150), "V1": (25, 240), "V2": (150, 250)}
DELTA = {"L1": 4, "L2": 12, "L3": 36}
ROL = {"excitacion": "DOE", "golpes": "tap_test", "trayectoria": "trajectory", "carga": "payload_100g",
       "bloqueado": "locked_arms", "piloto": "pilot", "piloto_golpes": "pilot_tap", "reposo": "rest_noise",
       "calibracion": "kinematic_calibration"}

def leer(n):
    p = ruta_res(n)
    return {r["nombre"]: r for r in csv.DictReader(open(p, encoding="utf-8"))} if os.path.exists(p) else {}

VC, TV, IM = leer("video_corridas.csv"), leer("tray_video.csv"), leer("imu_corridas.csv")
FALLAS = leer("tray_video_fallas.csv")
R = indice()
orden = {}
filas, excl = [], []
for r in sorted(R, key=lambda r: (r["sesion"], r["fecha"], r["hora"])):
    orden[r["sesion"]] = orden.get(r["sesion"], 0) + 1
    rol = ROL.get(r["tipo"], r["tipo"])
    if r["tipo"] == "excitacion" and r["postura"] in ("V1", "V2"): rol = "validation_posture"
    if r["tipo"] == "carga" and r["carga_g"] == 0: rol = "payload_control_0g"
    if r["tipo"] == "trayectoria":
        tv = TV.get(r["nombre"])
        rol = "trajectory_train" if tv and tv["conjunto"] == "ajuste" else "trajectory_test"
        if r["nombre"] in FALLAS: rol = "trajectory_test"
    # video asociado y validez del video
    vid, vid_ok, vid_mot = "", "", ""
    if r["nombre"] in VC:
        v = VC[r["nombre"]]; vid = v["video"]
        rs = float(v["r_sinc"]) if v["r_sinc"] not in ("", "nan") else float("nan")
        vid_ok = str(rs >= 0.75) if rs == rs else "False"
        vid_mot = "" if vid_ok == "True" else ("sin video sincronizable" if rs != rs else "sincronización r = %.2f < 0.75" % rs)
    if r["nombre"] in TV:
        t = TV[r["nombre"]]; vid = t["video"]; vid_ok = t["video_valido"]; vid_mot = t["motivo_video"]
    if r["nombre"] in FALLAS:
        f = FALLAS[r["nombre"]]; vid = f["video"]; vid_ok = "False"; vid_mot = "seguimiento del marcador insuficiente (46 % de cuadros); reconstrucción fallida"
    x, y = (re.findall(r"-?\d+", r["xy"]) + ["", ""])[:2] if r["xy"] else ("", "")
    fila = dict(run_id=r["nombre"][:19], folder="data/raw/%s/Lecturas/%s" % (r["sesion"], r["nombre"]),
                session_folder=r["sesion"], doe_session=SES_DOE.get(r["sesion"], ""), **{"pass": r["pasada"]},
                order_in_session=orden[r["sesion"]], replicate=r["replica"], role=rol, posture=r["postura"],
                x_mm=x, y_mm=y, level=r["nivel"] if r["tipo"] != "trayectoria" else "",
                delta_mm=DELTA.get(r["nivel"], "") if r["tipo"] in ("excitacion", "carga") else "",
                trajectory=r["nivel"] if r["tipo"] == "trayectoria" else "", vmax_mm_s=r["vmax"], amax_mm_s2=r["amax"],
                payload_g=r["carga_g"], date=r["fecha"], time=r["hora"].replace("-", ":"),
                video_tracking_file=("data/raw/video_tracking/" + vid) if vid else "", video_valid=vid_ok,
                video_exclusion_reason=vid_mot, excluded=str(not r["valida"]), exclusion_reason=r["motivo"] if not r["valida"] else "",
                fn_gyro_hz=IM.get(r["nombre"], {}).get("fn", ""), zeta_gyro=IM.get(r["nombre"], {}).get("zeta", ""))
    filas.append(fila)
    if not r["valida"]:
        excl.append(dict(run_id=fila["run_id"], role=rol, posture=r["postura"], level=fila["level"], scope="run (all sensors)",
                         reason=r["motivo"], action="sensitivity only" if "sensibilidad" in r["motivo"] else "excluded; cell re-run"))
    elif vid_ok == "False":
        excl.append(dict(run_id=fila["run_id"], role=rol, posture=r["postura"], level=fila["level"], scope="video metrics only",
                         reason=vid_mot, action="gyroscope kept"))
os.makedirs(RAIZ_DATA, exist_ok=True)
with open(os.path.join(RAIZ_DATA, "runs.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, list(filas[0].keys())); w.writeheader(); w.writerows(filas)
with open(os.path.join(RAIZ_DATA, "exclusions.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, list(excl[0].keys())); w.writeheader(); w.writerows(excl)
import collections
print("runs.csv:", len(filas), "corridas;", dict(collections.Counter(f["role"] for f in filas)))
print("exclusions.csv:", len(excl), "registros;", dict(collections.Counter((e["scope"], e["role"]) for e in excl)))
