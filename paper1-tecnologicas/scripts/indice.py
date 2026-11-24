from rutas import *
"""Índice de corridas del Paper 1: sesión, pasada, tipo, postura, nivel, réplica, validez."""
import glob, os, re, json
import numpy as np
BASE=DATOS
POST={(75,280):"A",(170,220):"B",(75,200):"C",(0,150):"D",(25,240):"V1",(150,250):"V2"}
DLV={4:"L1",12:"L2",36:"L3"}
def hora(nm): return nm[11:19]
def indice():
    rows=[]
    for d in sorted(glob.glob(BASE+"/S*/Lecturas/2026-*")):
        nm=os.path.basename(d); ses=d.split("/")[-3]
        if "Ir_a" in nm: continue
        info=json.load(open(d+"/info.json"))
        r=dict(carpeta=d,nombre=nm,sesion=ses,fecha=nm[:10],hora=hora(nm),tipo="",postura="",nivel="",xy="",
               vmax=info.get("vmax_mm_s"),amax=info.get("amax_mm_s2"),interrumpida=bool(info.get("interrumpida")),
               valida=True,motivo="",carga_g=0,pasada="",replica="")
        m=re.search(r"ciclos_(-?\d+)_(-?\d+)_d(\d+)",nm); g=re.search(r"Golpes_(-?\d+)_(-?\d+)",nm)
        if m:
            x,y,dd=abs(int(m.group(1))),int(m.group(2)),int(m.group(3)); r.update(tipo="excitacion",postura=POST.get((x,y),"(%d,%d)"%(x,y)),nivel=DLV[dd],xy="(%d, %d)"%(x,y))
            if "2_ciclos" in nm: r.update(tipo="piloto")
        elif g:
            x,y=abs(int(g.group(1))),int(g.group(2)); r.update(tipo="golpes",postura=POST.get((x,y),"(%d,%d)"%(x,y)),xy="(%d, %d)"%(x,y))
            if ses=="S1": r.update(tipo="reposo" if r["hora"]<"10-30" else "piloto_golpes")
            if ses=="S4": r.update(tipo="bloqueado")
        elif "Secuencia" in nm: r.update(tipo="calibracion")
        elif "Figura" in nm:
            fig="C" if "Cuadrado" in nm else "O"; r.update(tipo="trayectoria",nivel="%s%d/%d"%(fig,info["vmax_mm_s"],info["amax_mm_s2"]))
        rows.append(r)
    # pasadas y réplicas
    for r in rows:
        h=r["hora"]; s=r["sesion"]; f=r["fecha"]
        if s=="S2" and f=="2026-11-22": r["pasada"]=1 if h<"12-30" else 2
        if s=="S2" and f=="2026-11-23": r["pasada"]=2; r["motivo"]="repetición (23 nov)"
        if s=="S3": r["pasada"]=1 if h<"16-00-00" else 2
        if s in("S2","S3") and r["pasada"]: r["replica"]={("S2",1):1,("S2",2):2,("S3",1):3,("S3",2):4}[(s,r["pasada"])]
        if s=="S4" and r["tipo"]=="excitacion":
            r["tipo"]="carga"; r["carga_g"]=0 if r["hora"] in("17-50-49","17-51-29") else 100
    # exclusiones predefinidas
    for r in rows:
        h=r["hora"]
        if r["interrumpida"]: r.update(valida=False,motivo="corrida interrumpida")
        if r["sesion"]=="S2" and r["fecha"]=="2026-11-22" and r["xy"]=="(0, 146)": r.update(valida=False,motivo="postura desplazada 4 mm por corrida interrumpida (repetida el 23 nov)")
        if r["sesion"]=="S2" and r["fecha"]=="2026-11-22" and h>="13-06" and r["tipo"]=="excitacion": r.update(valida=False,motivo="velocidad 100/1000 en vez de 300/2000 (repetida el 23 nov)")
        if r["nombre"].startswith("2026-11-23_15-38-47"): r.update(valida=False,motivo="repetición extra: la réplica original (12:50:23) es válida; solo para sensibilidad")
    return rows
if __name__=="__main__":
    import collections
    R=indice()
    print(len(R)); print(collections.Counter((r["sesion"],r["tipo"],r["valida"]) for r in R))
    for r in R:
        if not r["valida"]: print(r["nombre"],r["motivo"])
