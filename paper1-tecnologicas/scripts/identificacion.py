from rutas import *
import sys, json, math, numpy as np, time, csv
import modelo5b as m
from identificar import ajustar, unpack
from indice import indice
R=indice()
tray=[r for r in R if r["tipo"]=="trayectoria"]
train=[r for r in tray if r["sesion"]=="S2" and r["nivel"] in ("C100/1000","C200/2000")]
test=[r for r in tray if r not in train]
def cargar(r):
    c=m.preparar(m.leer_corrida(r["carpeta"])); c["meta"]=r; return c
def const_inercia(c, Mc):
    c=dict(c); n=len(c["t"]); c["M"]=np.repeat(Mc[None],n,0); c["dM1"]=np.zeros((n,2,2)); c["dM2"]=np.zeros((n,2,2)); return c
sel=lambda c: c["t"]>=0.0
import os
FF_FIJO=os.environ.get("FF_FIJO")
if FF_FIJO is None:
    libres=["K1","K2","B1","B2","ff","tau"]; base={"K":1.0,"B":0.05,"ff":0.6,"tau":0.02}
    x0=[1.3,0.9,0.04,0.06,0.6,0.023]; esc=[1,1,0.05,0.05,0.5,0.02]
else:
    libres=["K1","K2","B1","B2","tau"]; base={"K":1.0,"B":0.05,"ff":float(FF_FIJO),"tau":0.02}
    x0=[1.3,0.9,0.04,0.06,0.025]; esc=[1,1,0.05,0.05,0.02]
def fitpct(e,y): return 100*(1-np.linalg.norm(e)/np.linalg.norm(y-y.mean()))
def evaluar(datos,p):
    out=[]
    for c in datos:
        _,_,w3=m.simular(c,p); y=c["gz"]/m.K_GIRO; k=sel(c)
        e=(y-w3)[k]; rig=m.rigido(c,0.085)[k]
        out.append(dict(nombre=c["meta"]["nombre"],sesion=c["meta"]["sesion"],tray=c["meta"]["nivel"],
            fit=fitpct(e,y[k]),rmse=float(np.degrees(np.sqrt(np.mean(e**2)))),mae=float(np.degrees(np.mean(np.abs(e)))),
            fit_rig=fitpct(y[k]-rig,y[k]),rmse_rig=float(np.degrees(np.sqrt(np.mean((y[k]-rig)**2))))))
    return out
if __name__=="__main__":
    t0=time.time()
    Dtr=[cargar(r) for r in train]; Dte=[cargar(r) for r in test]
    print("train",[c["meta"]["nivel"] for c in Dtr],"test",len(Dte))
    x,cov=ajustar(Dtr,libres,base,x0,esc,sel); p=unpack(x,libres,base); sd=np.sqrt(np.diag(cov))
    print("M(q):",{k:round(v,4) for k,v in zip(libres,x)},"sd",np.round(sd,4),"%.0fs"%(time.time()-t0))
    # inercia constante: M en la postura central del área (C)
    q=m.ik if hasattr(m,"ik") else None
    import video_real as vr
    qc=vr.ik(0.075,0.200); Mc,_=m.masa(*qc)
    Dtr_c=[const_inercia(c,Mc) for c in Dtr]; Dte_c=[const_inercia(c,Mc) for c in Dte]
    xc,covc=ajustar(Dtr_c,libres,base,x,esc,sel); pc=unpack(xc,libres,base)
    print("M const:",{k:round(v,4) for k,v in zip(libres,xc)},"%.0fs"%(time.time()-t0))
    # jackknife por corrida
    jk=[]
    for i in range(len(Dtr)):
        xi,_=ajustar([c for j,c in enumerate(Dtr) if j!=i],libres,base,x,esc,sel); jk.append(xi)
    jk=np.array(jk); n=len(Dtr); sd_jk=np.sqrt((n-1)/n*np.sum((jk-jk.mean(0))**2,0))
    print("jackknife sd",np.round(sd_jk,4),"%.0fs"%(time.time()-t0))
    corr=cov/np.outer(sd,sd)
    res=dict(ff_fijo=FF_FIJO,params={**base,**dict(zip(libres,map(float,x)))},sd_jac=dict(zip(libres,map(float,sd))),sd_jk=dict(zip(libres,map(float,sd_jk))),
             corr=corr.tolist(),params_const={**base,**dict(zip(libres,map(float,xc)))},
             train=evaluar(Dtr,p),test=evaluar(Dte,p),train_const=evaluar(Dtr_c,pc),test_const=evaluar(Dte_c,pc),
             train_names=[r["nombre"] for r in train],test_names=[r["nombre"] for r in test])
    json.dump(res,open(ruta_res("identificacion%s.json"%("" if FF_FIJO is None else "_ff"+FF_FIJO)),"w"),indent=1)
    for k in ("train","test","train_const","test_const"):
        a=res[k]; print(k,"fit %.1f [%.1f–%.1f] rmse %.2f °/s | rígido fit %.1f"%(np.mean([z["fit"] for z in a]),min(z["fit"] for z in a),max(z["fit"] for z in a),np.mean([z["rmse"] for z in a]),np.mean([z["fit_rig"] for z in a])))
    print("%.0fs"%(time.time()-t0))
