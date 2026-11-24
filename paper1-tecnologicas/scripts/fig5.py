from rutas import *
import sys, json, csv, numpy as np, os
os.environ["FF_FIJO"]="0.5"
from estilo import *
import modelo5b as m, video_real as vr
from identificacion import cargar, const_inercia, test, sel
from doe import celda
J=json.load(open(ruta_res("identificacion_ff0.5.json"))); p=J["params"]; pc=J["params_const"]
MOD=json.load(open(ruta_res("modos_modelo.json")))
TV=list(csv.DictReader(open(ruta_res("tray_video.csv"))))
R=list(csv.DictReader(open(ruta_res("imu_corridas.csv"))))
fig=plt.figure(figsize=(7.16,4.6)); gs=fig.add_gridspec(2,3,height_ratios=[1,1.05],hspace=0.55,wspace=0.42)
# (a) traza de prueba: el círculo O200/2000 de S3 (no visto en el ajuste)
r=[r for r in test if r["nivel"]=="O200/2000" and r["sesion"]=="S3"][0]
c=cargar(r); _,_,w=m.simular(c,p); qc=vr.ik(0.075,0.200); Mc,_=m.masa(*qc); _,_,wc=m.simular(const_inercia(c,Mc),pc)
y=np.degrees(c["gz"]/m.K_GIRO); k=sel(c)
ax=fig.add_subplot(gs[0,:]); t=c["t"][k]
ax.plot(t,y[k],color=GRAY,lw=1.6,label="measured (IMU gyro)")
ax.plot(t,np.degrees(w[k]),color=COL["A"],lw=1.0,label="M(q) model")
ax.plot(t,np.degrees(wc[k]),color=COL["B"],lw=0.9,ls=(0,(3,1.5)),label="constant-inertia model")
fit=lambda a:100*(1-np.linalg.norm(y[k]-a)/np.linalg.norm(y[k]-y[k].mean()))
ax.set_title("(a) Unseen test run: circle, 200 mm/s, 2000 mm/s² (session 2). Fit: M(q) %.1f %%, constant inertia %.1f %%"%(fit(np.degrees(w[k])),fit(np.degrees(wc[k]))),fontsize=7.5)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Distal angular rate [°/s]"); ejes(ax); ax.legend(frameon=False,ncol=3,loc="lower right",fontsize=6.5)
ax.set_xlim(t[0],t[-1])
# (b) ajuste % emparejado en las 20 de prueba
ax=fig.add_subplot(gs[1,0])
a=np.array([d["fit"] for d in J["test"]]); b=np.array([d["fit"] for d in J["test_const"]]); rg=np.array([d["fit_rig"] for d in J["test"]])
for i in range(len(a)): ax.plot([0,1,2],[rg[i],b[i],a[i]],color=GRID,lw=0.7,zorder=1)
for j,(v,cc) in enumerate(((rg,GRAY),(b,COL["B"]),(a,COL["A"]))):
    ax.plot(np.full(len(v),j)+np.linspace(-0.08,0.08,len(v)),v,"o",ms=2.8,color=cc,zorder=2)
    ax.errorbar(j+0.2,v.mean(),yerr=1.96*v.std(ddof=1)/np.sqrt(len(v)),fmt="_",ms=10,color=INK,capsize=2,lw=1,zorder=3)
    ax.text(j+0.27,v.mean(),"%.1f"%v.mean(),fontsize=6.3,va="center")
ax.set_xticks([0,1,2]); ax.set_xticklabels(["rigid +\n85 ms","constant\ninertia","M(q)"]); ax.set_ylabel("Gyro fit [%] (20 test runs)")
ax.set_title("(b)"); ejes(ax); ax.set_xlim(-0.4,2.6)
# (c) RMSE de posición por video
ax=fig.add_subplot(gs[1,1])
T=[d for d in TV if d["conjunto"]=="prueba" and d["video_valido"]=="True"]
cols=[("rmse_rigido",GRAY,"rigid +\n85 ms"),("rmse_const",COL["B"],"constant\ninertia"),("rmse_modelo",COL["A"],"M(q)")]
V=np.array([[float(d[c_]) for c_,_,_ in cols] for d in T])
for row in V: ax.plot([0,1,2],row,color=GRID,lw=0.7,zorder=1)
for j,(c_,cc,lab) in enumerate(cols):
    v=V[:,j]; ax.plot(np.full(len(v),j)+np.linspace(-0.08,0.08,len(v)),v,"o",ms=2.8,color=cc,zorder=2)
    ax.errorbar(j+0.2,v.mean(),yerr=1.96*v.std(ddof=1)/np.sqrt(len(v)),fmt="_",ms=10,color=INK,capsize=2,lw=1,zorder=3)
    ax.text(j+0.27,v.mean(),"%.2f"%v.mean(),fontsize=6.3,va="center")
ax.set_xticks([0,1,2]); ax.set_xticklabels([c_[2] for c_ in cols]); ax.set_ylabel("Effector position RMSE [mm]\n(video, %d test runs)"%len(V))
ax.set_title("(c)"); ejes(ax); ax.set_xlim(-0.4,2.6)
# (d) f_n predicho (modo dominante en el giro) vs medido
ax=fig.add_subplot(gs[1,2]); fn=celda("fn")
dom=lambda P,key="modos": max(MOD[P][key],key=lambda d:d["vis_gz"])["fn"]
for L in ("L1","L2","L3"):
    xs=[fn[(P,L)]["media"] for P in "ABCD"]; ys=[dom(P) for P in "ABCD"]
    ax.plot(xs,ys,"o",ms=4.5,color=LV[L],label="DOE "+L,zorder=3)
for P in "ABCD": ax.text(fn[(P,"L3")]["media"]-0.06,dom(P)-0.17,P,fontsize=6.5,ha="right")
def med(P,carga=None,tipo="excitacion"):
    v=[float(r["fn"]) for r in R if r["tipo"]==tipo and r["valida"]=="True" and r["postura"]==P and (carga is None or r["carga_g"]==carga) and (tipo!="excitacion" or r["nivel"]=="L2")]
    return np.mean(v)
for P in ("V1","V2"):
    ax.plot(med(P),dom(P),MK[P],ms=5,mfc="white",mec=COL[P],mew=1.1,zorder=4); ax.text(med(P)+(0.06 if P=="V1" else -0.08),dom(P)+(-0.05 if P=="V1" else 0.12),P,fontsize=6.3,ha="left" if P=="V1" else "right")
for P in ("A","D"):
    xm=med(P,"100",tipo="carga"); ax.plot(xm,dom(P,"modos_100g"),"*",ms=6.5,color=INK,zorder=4); ax.text(xm+0.08,dom(P,"modos_100g")-(0.22 if P=="A" else 0.1),P+" + 100 g",fontsize=6.3)
ax.plot([1.4,4],[1.4,4],color=GRAY,lw=0.6,ls="--"); ax.set_xlim(1.4,4); ax.set_ylim(1.2,4)
ax.set_xlabel("Measured f$_n$ [Hz] (gyro)"); ax.set_ylabel("Model f$_n$, gyro-dominant mode [Hz]"); ax.set_title("(d)"); ejes(ax)
ax.plot([],[],"v",mfc="white",mec=GRAY,ms=5,label="validation posture (L2)"); ax.plot([],[],"*",color=INK,ms=6,label="100 g payload (L2)")
ax.legend(frameon=False,fontsize=5.8,loc="lower right",handletextpad=0.2)
guardar(fig,"fig5_validacion")
print("carga A",med("A","100","carga"),"D",med("D","100","carga"))
