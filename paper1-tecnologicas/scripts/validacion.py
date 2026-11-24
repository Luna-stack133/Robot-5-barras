"""Comparación pareada de modelos en las corridas de prueba: ajuste % del giro (n = 20) y RMSE de posición en video
(n = 13), con IC95 % de la diferencia y prueba de Wilcoxon de rangos con signo. Salida: results/validacion_pareada.csv"""
from rutas import *
import csv, json, numpy as np
from scipy import stats
J = json.load(open(ruta_res("identificacion_ff0.5.json")))
a = np.array([d["fit"] for d in J["test"]]); b = np.array([d["fit"] for d in J["test_const"]]); g = np.array([d["fit_rig"] for d in J["test"]])
TV = [d for d in csv.DictReader(open(ruta_res("tray_video.csv"))) if d["conjunto"] == "prueba" and d["video_valido"] == "True"]
V = {k: np.array([float(d[k]) for d in TV]) for k in ("rmse_modelo", "rmse_const", "rmse_rigido")}
out = []
def par(nombre, x, y, mejor_mayor=True):
    d = x - y; n = len(d); h = stats.t.ppf(.975, n - 1) * d.std(ddof=1) / np.sqrt(n)
    gana = int((d > 0).sum() if mejor_mayor else (d < 0).sum())
    out.append(dict(comparison=nombre, n=n, mean_a=x.mean(), sd_a=x.std(ddof=1), mean_b=y.mean(), sd_b=y.std(ddof=1),
                    mean_diff=d.mean(), ci95_low=d.mean() - h, ci95_high=d.mean() + h, a_better_in=gana, wilcoxon_p=stats.wilcoxon(d).pvalue))
par("gyro fit %: M(q) vs constant inertia", a, b); par("gyro fit %: M(q) vs rigid+delay", a, g); par("gyro fit %: constant inertia vs rigid+delay", b, g)
par("video RMSE mm: M(q) vs constant inertia", V["rmse_modelo"], V["rmse_const"], False)
par("video RMSE mm: M(q) vs rigid+delay", V["rmse_modelo"], V["rmse_rigido"], False)
with open(ruta_res("validacion_pareada.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, list(out[0].keys())); w.writeheader(); w.writerows(out)
for o in out: print("%-45s n=%2d  dif %.2f [%.2f, %.2f]  mejor en %d  p=%.1e" % (o["comparison"], o["n"], o["mean_diff"], o["ci95_low"], o["ci95_high"], o["a_better_in"], o["wilcoxon_p"]))
