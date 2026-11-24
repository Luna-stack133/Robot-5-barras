from rutas import *
import sys, csv, json, numpy as np, os, warnings; warnings.filterwarnings("ignore")
os.environ["FF_FIJO"]="0.5"
from estilo import *
from video_lib import cargar_video, BASE
import imu_libre as L
from scipy.signal import welch
import modelo5b as m
from indice import indice
VC={r["nombre"]:r for r in csv.DictReader(open(ruta_res("video_corridas.csv")))}
p=json.load(open(ruta_res("identificacion_ff0.5.json")))["params"]
MOD=json.load(open(ruta_res("modos_modelo.json")))
R=[r for r in indice() if r["tipo"]=="excitacion" and r["valida"] and r["postura"] in "ABCD" and r["nivel"]=="L2"]
fig,axs=plt.subplots(2,4,figsize=(7.16,3.6),gridspec_kw=dict(height_ratios=[1.25,1]))
tg=np.arange(0,1.9,1/60); U={}; SH=[]
for j,pos in enumerate("ABCD"):
    rr=sorted([r for r in R if r["postura"]==pos],key=lambda r:r["replica"])
    ax=axs[0,j]; reps=[]; mods=[]; psds=[]
    for r in rr:
        vc=VC[r["nombre"]]; V=cargar_video(BASE+"/video_tracking/"+vc["video"]); lag=float(vc["lag"])
        info,out=L.excitacion(r["carpeta"]); P_arr=[o for o in out if o["punto"]=="P"]
        # modelo: simular la corrida
        c=m.preparar(m.leer_corrida(r["carpeta"])); q,_,_=m.simular(c,p); Pm=np.array([m.fk(a,b)[2] for a,b in q])*1000
        # ajuste fino del desfase video↔modelo (solo para la figura): correlación de rapidez sobre toda la corrida
        g=np.arange(c["t"][0]+0.5,c["t"][-1]-0.5,0.01); vm=np.linalg.norm(np.gradient(np.column_stack([np.interp(g,c["t"],Pm[:,k]) for k in range(2)]),0.01,axis=0),axis=1)
        ok=V["ok"]; best=(-1,0)
        for dx in np.arange(-0.35,0.351,0.005):
            Pv=np.column_stack([np.interp(g+lag+dx,V["tv"][ok],V["P"][ok][:,k]) for k in range(2)]); vv=np.linalg.norm(np.gradient(Pv,0.01,axis=0),axis=1)
            rr_=np.corrcoef(vm,vv)[0,1]
            if rr_>best[0]: best=(rr_,dx)
        lag2=lag+best[1]; SH.append(best[1])
        tr=[]; tm=[]
        for o in P_arr:
            ta=o["ta"]
            wm=(c["t"]>=ta)&(c["t"]<=ta+1.95); tmm=c["t"][wm]-ta; Em=Pm[wm]-np.median(Pm[wm][tmm>tmm[-1]-0.3],0)
            if U.get(pos) is None:
                u=np.linalg.svd(Em,full_matrices=False)[2][0]; U[pos]=u if u[1]>0 else -u
            u=U[pos]; tm.append(np.interp(tg,tmm,Em@u))
            w=(V["tv"]>=ta+lag2)&(V["tv"]<=ta+lag2+1.95)&V["ok"]
            if w.sum()<60: continue
            t=V["tv"][w]-ta-lag2; P=V["P"][w]; rest=np.median(P[t>t[-1]-0.3],0); E=P-rest
            tr.append(np.interp(tg,t,E@u))
            # PSD del giro (IMU) en la ventana
        reps.append(np.mean(tr,0)); mods.append(np.mean(tm,0))
        d=np.genfromtxt(r["carpeta"]+"/imu.csv",delimiter=",",names=True); seg=[]
        for o in P_arr:
            w=(d["t_s"]>=o["ta"])&(d["t_s"]<o["ta"]+1.9); seg.append(d["gz"][w]-d["gz"][w].mean())
        f,Pxx=welch(np.concatenate(seg),fs=100,nperseg=128,noverlap=64,nfft=1024); psds.append(Pxx)
    reps=np.array(reps); 
    ax.fill_between(tg,reps.min(0),reps.max(0),color=COL[pos],alpha=0.18,lw=0)
    for k,y in enumerate(reps): ax.plot(tg,y,color=COL[pos],lw=0.6,alpha=0.9)
    ax.plot(tg,np.mean(mods,0),color=INK,lw=1.0,ls="--")
    ax.axhline(0,color=GRAY,lw=0.4)
    fn=[float(VC[r["nombre"]]["fn_v"]) for r in rr]
    ax.set_title("%s  (%s)"%(pos,{"A":"75, 280","B":"170, 220","C":"75, 200","D":"0, 150"}[pos]),fontsize=7.5)
    ax.text(0.97,0.95,"f$_n$ video %.2f Hz"%np.median(fn),transform=ax.transAxes,ha="right",va="top",fontsize=6.3)
    ax.set_xlim(0,1.9); ejes(ax)
    if j==0: ax.set_ylabel("Displacement along main\naxis [mm] (video, L2)")
    ax.set_xlabel("Time after stop [s]")
    a2=axs[1,j]; P=np.array(psds); k=(f<=12)
    for y in P: a2.semilogy(f[k],y[k],color=COL[pos],lw=0.6)
    for md in MOD[pos]["modos"]:
        a2.axvline(md["fn"],color=INK,ls=(0,(2,2)),lw=0.7)
    a2.set_xlim(0,8); ejes(a2); a2.set_xlabel("Frequency [Hz]")
    if j==0: a2.set_ylabel("Gyro PSD [(°/s)²/Hz]")
axs[0,0].plot([],[],color=GRAY,lw=0.6,label="replicate (mean of 5 stops)"); axs[0,0].plot([],[],color=INK,ls="--",lw=1,label="identified model")

axs[1,0].plot([],[],color=INK,ls=(0,(2,2)),lw=0.7,label="model natural frequencies (gyro PSD rows)")
fig.tight_layout(h_pad=0.6,w_pad=0.4,rect=(0,0,1,0.94))
h=axs[0,0].get_legend_handles_labels(); h2=axs[1,0].get_legend_handles_labels()
fig.legend(h[0]+h2[0],h[1]+h2[1],loc="upper center",ncol=3,frameon=False,fontsize=6.5)
guardar(fig,"fig3_respuesta_libre"); print("desfase extra video-modelo [ms]:",np.round(np.array(SH)*1000))
