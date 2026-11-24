from rutas import *
import sys, csv, warnings; warnings.filterwarnings("ignore")
import numpy as np
from indice import indice
import imu_libre as L
R=indice()
llegadas=[]; corridas=[]
for r in R:
    if r["tipo"] in ("excitacion","carga","piloto"):
        info,out=L.excitacion(r["carpeta"])
        P=[o for o in out if o["punto"]=="P"]
        for k,o in enumerate(P):
            llegadas.append(dict(nombre=r["nombre"],k=k,**{x:o[x] for x in ("ta","pk","fn","zeta","R2","t2","resid")}))
        ok=[o for o in P if o["R2"]>=0.85]
        med=lambda key: float(np.nanmedian([o[key] for o in ok])) if ok else np.nan
        corridas.append(dict(r,n_lleg=len(P),n_ok=len(ok),pk=med("pk"),fn=med("fn"),zeta=med("zeta"),R2=med("R2"),t2=med("t2"),resid=med("resid")))
    elif r["tipo"] in ("golpes","piloto_golpes"):
        info,out=L.golpes(r["carpeta"])
        ok=[o for o in out if o["R2"]>=0.85]
        for k,o in enumerate(out): llegadas.append(dict(nombre=r["nombre"],k=k,ta=np.nan,pk=o["reb"],fn=o["fn"],zeta=o["zeta"],R2=o["R2"],t2=o["t2"],resid=np.nan))
        med=lambda key: float(np.nanmedian([o[key] for o in ok])) if ok else np.nan
        corridas.append(dict(r,n_lleg=len(out),n_ok=len(ok),pk=med("reb"),fn=med("fn"),zeta=med("zeta"),R2=med("R2"),t2=med("t2"),resid=np.nan))
def guardar(fn,rows):
    keys=list(rows[0].keys())
    with open(fn,"w",newline="") as f:
        w=csv.DictWriter(f,keys); w.writeheader(); w.writerows(rows)
guardar(ruta_res("imu_llegadas.csv"),llegadas)
guardar(ruta_res("imu_corridas.csv"),[{k:v for k,v in c.items() if k!="carpeta"} for c in corridas])
print(len(llegadas),len(corridas))
