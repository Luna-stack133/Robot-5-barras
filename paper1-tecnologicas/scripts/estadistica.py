"""ANOVA del DOE (log f_n y ζ; postura × nivel + sesión como bloque), tabla de celdas e IC95 %, sensibilidad.
Salidas: results/doe_celdas.csv, results/doe_anova.csv, results/doe_sensibilidad.csv"""
from rutas import *
import csv, numpy as np
import doe

def guardar(n, filas):
    with open(ruta_res(n), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, list(filas[0].keys())); w.writeheader(); w.writerows(filas)

F, Z = doe.celda("fn"), doe.celda("zeta")
guardar("doe_celdas.csv", [dict(posture=p, level=l, n=F[(p, l)]["n"], fn_mean_hz=F[(p, l)]["media"], fn_ci95_hz=F[(p, l)]["ic"],
        fn_cv_pct=F[(p, l)]["cv"], zeta_mean=Z[(p, l)]["media"], zeta_ci95=Z[(p, l)]["ic"]) for p in "ABCD" for l in ("L1", "L2", "L3")])
filas = []
for var, lg in (("fn", True), ("zeta", False)):
    A, dfe, N = doe.anova(var, lg)
    for k, v in A.items():
        filas.append(dict(response="log(fn)" if lg else "zeta", source=k, df=v["df"], df_error=dfe, F=v["F"], p=v["p"], partial_eta2=v["eta2p"]))
guardar("doe_anova.csv", filas)
for f in filas: print("%-8s %-16s F(%d,%d) = %7.2f  p = %.2e  η²p = %.3f" % (f["response"], f["source"], f["df"], f["df_error"], f["F"], f["p"], f["partial_eta2"]))

# Sensibilidad: (1) cambiar la réplica D-L1 12:50:23 por la repetición extra 15:38:47; (2) sin la postura D
base = list(doe.doe); sens = []
o = [r for r in base if "12-50-23" in r["nombre"]][0]; x = dict([r for r in doe.R if "15-38-47" in r["nombre"]][0])
x["replica"] = o["replica"]; x["valida"] = "True"
doe.doe[:] = [r if r is not o else x for r in base]
for k, v in doe.anova("fn", True)[0].items(): sens.append(dict(case="extra D-L1 repeat swapped in", source=k, F=v["F"], p=v["p"], partial_eta2=v["eta2p"]))
doe.doe[:] = [r for r in base if r["postura"] != "D"]; doe.POS = "ABC"
for k, v in doe.anova("fn", True)[0].items(): sens.append(dict(case="posture D dropped", source=k, F=v["F"], p=v["p"], partial_eta2=v["eta2p"]))
doe.doe[:] = base; doe.POS = "ABCD"
guardar("doe_sensibilidad.csv", sens)
