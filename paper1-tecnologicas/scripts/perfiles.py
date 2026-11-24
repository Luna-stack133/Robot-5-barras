from rutas import *
import json, numpy as np, time
from identificacion import *
Dtr=[cargar(r) for r in train]
res=json.load(open(ruta_res("identificacion.json")))
p0=res["params"]
def costo(x,lib,basep):
    from identificar import residuos
    r=residuos(x,lib,basep,Dtr,sel); return float(r@r)
out={}
for fijo,grid in (("ff",[0.0,0.2,0.4,0.6,0.8,1.0]),("tau",[0.0,0.01,0.02,0.03,0.04,0.05])):
    lib=[k for k in libres if k!=fijo]; prof=[]
    for v in grid:
        basep=dict(base); basep[fijo]=v
        x0=[p0[k] for k in lib]; e=[esc[libres.index(k)] for k in lib]
        x,_=ajustar(Dtr,lib,basep,x0,e,sel); c=costo(x,lib,basep)
        prof.append(dict(valor=v,costo=c,**{k:float(a) for k,a in zip(lib,x)})); print(fijo,v,round(c,5),np.round(x,4),flush=True)
    out[fijo]=prof
# línea base rígida: mejor retardo en train
best=None
for d in np.arange(0,0.15,0.005):
    e=np.concatenate([(c["gz"]/m.K_GIRO-m.rigido(c,d))[sel(c)] for c in Dtr]); s=float(e@e)
    if best is None or s<best[1]: best=(d,s)
out["rigido_tau"]=best[0]; print("rigido tau",best)
json.dump(out,open(ruta_res("perfiles.json"),"w"),indent=1)
