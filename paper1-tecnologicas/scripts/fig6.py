from rutas import *
import csv, numpy as np
from estilo import *
from scipy.signal import welch
from indice import indice
I=indice(); R=list(csv.DictReader(open(ruta_res("imu_corridas.csv"))))
fig,axs=plt.subplots(1,2,figsize=(7.16,2.6),gridspec_kw=dict(width_ratios=[1.2,1]))
ax=axs[0]
def psd(r):
    d=np.genfromtxt(r["carpeta"]+"/imu.csv",delimiter=",",names=True); g=d["gz"]-np.median(d["gz"])
    f,P=welch(g,fs=100,nperseg=256,nfft=1024); return f,P,np.abs(g).max()
act=[r for r in I if r["tipo"]=="golpes" and r["valida"] and r["postura"]=="A"]
blq=[r for r in I if r["tipo"]=="bloqueado"]
for rs,cc,lab in ((act,COL["A"],"servos enabled, tap at A (n=%d)"),(blq,INK,"servos off, arms locked by jig, tap (n=%d)")):
    for i,r in enumerate(rs):
        f,P,mx=psd(r); ax.semilogy(f,P,color=cc,lw=0.7,alpha=0.85,label=lab%len(rs) if i==0 else None)
ax.axvspan(1,5,color="#f1f0ec",zorder=0); ax.text(3,2e-5,"1–5 Hz band",ha="center",fontsize=6.3,color=GRAY)
ax.set_xlim(0,30); ax.set_xlabel("Frequency [Hz]"); ax.set_ylabel("Gyro PSD [(°/s)²/Hz]"); ejes(ax)
ax.set_title("(a) Locked-arm control (servos off)"); ax.legend(frameon=False,fontsize=6,loc="upper right")
ax=axs[1]
def fns(P,carga,tipo):
    return [float(r["fn"]) for r in R if r["tipo"]==tipo and r["valida"]=="True" and r["postura"]==P and r["carga_g"]==carga and (tipo=="carga" or r["nivel"]=="L2")]

# Predicción registrada ANTES de la sesión 4 (23 nov, 17:40) con el modelo previo K = diag(1.335, 0.865):
# razón de frecuencias con/sin 100 g = 0.952 (A) y 0.919 (D). Se registró en valor absoluto sobre la única
# réplica válida disponible entonces (S2 pasada 1: A 3.36 Hz → 3.20; D 1.96 Hz → 1.80). Aquí la razón se
# aplica a la línea base final (media DOE L2, n = 4), que es la comparación que se reporta en el texto.
RAZON={"A":0.952,"D":0.919}
base={P:np.mean(fns(P,"0","excitacion")) for P in "AD"}
pred={P:RAZON[P]*base[P] for P in "AD"}
for j,P in enumerate("AD"):
    for k,(v,mk,lab) in enumerate(((fns(P,"0","excitacion"),"o","no payload, DOE L2 (n=%d)"),(fns(P,"0","carga"),"s","no payload, same-session control (n=%d)"),(fns(P,"100","carga"),"*","100 g at effector (n=%d)"))):
        x=j*1.0+(k-1)*0.22+np.linspace(-0.04,0.04,len(v))
        ax.plot(x,v,mk,ms=4.5 if mk!="*" else 6.5,color=COL[P] if k<2 else INK,mfc=COL[P] if k==0 else ("white" if k==1 else INK),mec=COL[P] if k<2 else INK,ls="none")
        ax.hlines(np.mean(v),j+(k-1)*0.22-0.09,j+(k-1)*0.22+0.09,color=INK,lw=1)
    ax.hlines(pred[P],j+0.13,j+0.31,color="#e34948",lw=1.2,ls=(0,(2,1)))
    ax.text(j+0.33,pred[P],"pre-registered\nratio × DOE mean",fontsize=5.8,va="center",color="#3d3d3a")
ax.plot([],[],"o",color=GRAY,ms=4,label="no payload, DOE L2"); ax.plot([],[],"s",mfc="white",mec=GRAY,ms=4,label="no payload, same-session control")
ax.plot([],[],"*",color=INK,ms=6,label="100 g fixed at effector")
ax.set_xticks([0,1]); ax.set_xticklabels(["A (75, 280)","D (0, 150)"]); ax.set_xlim(-0.45,1.75); ax.set_ylabel("f$_n$ from gyro [Hz]")
ax.set_title("(b) Payload test (level L2)"); ejes(ax); ax.legend(frameon=False,fontsize=6,loc="center right")
fig.tight_layout(w_pad=1.5); guardar(fig,"fig6_diagnostico")
for P in "AD": print(P, np.mean(fns(P,"0","excitacion")), fns(P,"0","carga"), np.mean(fns(P,"100","carga")))
