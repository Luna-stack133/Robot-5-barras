from rutas import *
import csv, numpy as np, sys
from indice import indice
from video_lib import *
import imu_libre as L
R=[r for r in indice() if r["tipo"] in ("excitacion","carga") ]
rows=[]; runs=[]
def asent(t,d,tol):
    a=np.where(d>tol)[0]
    return 0.0 if len(a)==0 else (t[a[-1]+1] if a[-1]+1<len(t) else np.nan)
for r in R:
    s=sincronizar(r["carpeta"],r["sesion"])
    if s is None: runs.append(dict(nombre=r["nombre"],video="",r_sinc=np.nan)); continue
    vp,lag,rs=s; V=cargar_video(vp)
    info,out=L.excitacion(r["carpeta"])
    res=[]
    for o in [o for o in out if o["punto"]=="P"]:
        ta=o["ta"]+lag
        w=(V["tv"]>=ta)&(V["tv"]<=ta+1.95)&V["ok"]
        if w.sum()<60: continue
        t=V["tv"][w]-ta; P=V["P"][w]
        rest=np.median(P[t>t[-1]-0.3],axis=0)
        E=P-rest; d=np.hypot(*E.T)
        u=np.linalg.svd(E,full_matrices=False)[2][0]; sgn=E@u
        kp=np.argmax(np.abs(sgn)); fit=L.ajustar(t[kp:]-t[kp],sgn[kp:]) or dict(fn=np.nan,zeta=np.nan,R2=np.nan)
        x=dict(nombre=r["nombre"],ta=o["ta"],pk_mm=float(d.max()),t1=float(asent(t,d,1.0)),t05=float(asent(t,d,0.5)),
               fn_v=fit["fn"],zeta_v=fit["zeta"],R2_v=fit["R2"],fn_imu=o["fn"],zeta_imu=o["zeta"],pk_gz=o["pk"])
        rows.append(x); res.append(x)
    med=lambda k: float(np.nanmedian([x[k] for x in res])) if res else np.nan
    runs.append(dict(nombre=r["nombre"],video=os.path.basename(vp),lag=lag,r_sinc=rs,n=len(res),pk_mm=med("pk_mm"),t1=med("t1"),t05=med("t05"),
                     fn_v=med("fn_v"),zeta_v=med("zeta_v"),fn_imu=med("fn_imu")))
    print(r["nombre"][11:60], os.path.basename(vp)[3:18], "r=%.2f n=%d pk %.2f mm t1 %.2f fnv %.2f fni %.2f"%(rs,len(res),med("pk_mm"),med("t1"),med("fn_v"),med("fn_imu")),flush=True)
for fn,rr in ((ruta_res("video_llegadas.csv"),rows),(ruta_res("video_corridas.csv"),runs)):
    keys=sorted(set().union(*[x.keys() for x in rr]),key=lambda k:list(rr[0].keys()).index(k) if k in rr[0] else 99)
    with open(fn,"w",newline="") as f: w=csv.DictWriter(f,keys); w.writeheader(); w.writerows(rr)
