from rutas import *
import csv, numpy as np
from estilo import *
from doe import celda, doe
R=list(csv.DictReader(open(ruta_res("imu_corridas.csv"))))
VC={r["nombre"]:r for r in csv.DictReader(open(ruta_res("video_corridas.csv")))}
fn=celda("fn"); ze=celda("zeta")
G=[r for r in R if r["tipo"]=="golpes" and r["valida"]=="True"]
fig,axs=plt.subplots(1,3,figsize=(7.16,2.5),gridspec_kw=dict(width_ratios=[1,1,1.25]))
xs=np.arange(4); off={"L1":-0.22,"L2":0,"L3":0.22}
for k,(ax,C,lab) in enumerate(((axs[0],fn,"Natural frequency f$_n$ [Hz]"),(axs[1],ze,"Damping ratio ζ [–]"))):
    for L in ("L1","L2","L3"):
        m=[C[(p,L)]["media"] for p in "ABCD"]; h=[C[(p,L)]["ic"] for p in "ABCD"]
        ax.errorbar(xs+off[L],m,yerr=h,fmt="o",ms=4.5,color=LV[L],ecolor=LV[L],elinewidth=1,capsize=2,label={"L1":"L1 (Δ = 4 mm)","L2":"L2 (12 mm)","L3":"L3 (36 mm)"}[L])
        for i,p in enumerate("ABCD"):
            v=[float(r["fn" if k==0 else "zeta"]) for r in doe if r["postura"]==p and r["nivel"]==L]
            ax.plot(np.full(len(v),i+off[L])+0.07,v,".",ms=2,color=LV[L],alpha=0.6)
    gv=[[float(r["fn" if k==0 else "zeta"]) for r in G if r["postura"]==p] for p in "ABCD"]
    ax.plot(xs+0.42,[np.mean(v) for v in gv],"D",ms=4,mfc="white",mec=INK,mew=0.8,label="tap test")
    ax.set_xticks(xs); ax.set_xticklabels(list("ABCD")); ax.set_xlim(-0.5,3.65); ax.set_xlabel("Posture"); ax.set_ylabel(lab); ejes(ax)
axs[0].set_title("(a)"); axs[1].set_title("(b)")
axs[0].legend(frameon=False,fontsize=6,loc="lower left")
ax=axs[2]
for p in "ABCD":
    x=[];y=[]
    for r in doe:
        if r["postura"]!=p or r["nombre"] not in VC: continue
        v=VC[r["nombre"]]; pk=float(v["pk_mm"])
        if np.isfinite(pk) and float(v["r_sinc"])>=0.75: x.append(pk); y.append(float(r["fn"]))
    ax.plot(x,y,MK[p],ms=4,color=COL[p],mec="white",mew=0.5,ls="none",label=p)
    o=np.argsort(x); 
    if len(x)>3:
        cf=np.polyfit(np.log(np.array(x)),y,1); xx=np.linspace(min(x),max(x),20); ax.plot(xx,np.polyval(cf,np.log(xx)),color=COL[p],lw=0.8)
        ax.text(max(x)*1.08,np.polyval(cf,np.log(max(x))),p,color=INK,fontsize=7,fontweight="bold",va="center")
ax.set_xscale("log"); ax.set_xlabel("Peak deviation at the effector [mm] (video)"); ax.set_ylabel("f$_n$ from gyro [Hz]")
ax.set_title("(c)"); ejes(ax); ax.set_xlim(0.15,30)
fig.tight_layout(w_pad=1.0)
guardar(fig,"fig4_mapa_fn_zeta")
