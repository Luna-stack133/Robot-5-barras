from rutas import *
import json, glob, os, re, numpy as np
from scipy.optimize import curve_fit
BASE=DATOS
POST={(75,280):"A",(170,220):"B",(75,200):"C",(0,150):"D",(25,240):"V1",(150,250):"V2",(0,146):"D*"}
DLV={4:"Δ1",12:"Δ2",36:"Δ3"}

def leer(d):
    imu=np.genfromtxt(d+"/imu.csv",delimiter=",",names=True)
    tel=np.genfromtxt(d+"/telemetria.csv",delimiter=",",names=True)
    info=json.load(open(d+"/info.json"))
    return imu,tel,info

def modelo(t,A,s,w,ph,c): return A*np.exp(-s*t)*np.cos(w*t+ph)+c

def ajustar(t,g):
    """Ajusta oscilación amortiguada a g(t) (t desde 0). Devuelve fn, zeta, R2 o None."""
    if len(t)<40: return None
    gg=g-np.mean(g[-20:])
    # frecuencia inicial por FFT
    n=len(gg); F=np.abs(np.fft.rfft(gg*np.hanning(n),4096)); fr=np.fft.rfftfreq(4096,np.median(np.diff(t)))
    k=(fr>1.2)&(fr<8); f0=fr[k][np.argmax(F[k])]
    best=None
    for ph in (0,np.pi/2,np.pi,3*np.pi/2):
        try:
            p,_=curve_fit(modelo,t,g,p0=[np.max(np.abs(gg)),3.0,2*np.pi*f0,ph,np.mean(g[-20:])],maxfev=20000)
        except Exception: continue
        r=g-modelo(t,*p); R2=1-np.sum(r**2)/np.sum((g-g.mean())**2)
        if p[1]>0 and p[2]>0 and (best is None or R2>best[2]): best=(p,None,R2)
    if best is None: return None
    p,_,R2=best; s,w=p[1],p[2]; wn=np.hypot(s,w); return dict(fn=wn/2/np.pi, zeta=s/wn, R2=R2)

def t_bajo(t,g,thr=2.0):
    a=np.where(np.abs(g)>=thr)[0]
    return 0.0 if len(a)==0 else (t[a[-1]+1] if a[-1]+1<len(t) else np.nan)

def excitacion(d):
    imu,tel,info=leer(d)
    t=imu["t_s"]; gz=imu["gz"]
    P=np.array(info["referencia_mm"][0]); tx,ty=tel["x_mm"],tel["y_mm"]; tt=tel["t_s"]
    pos=np.column_stack([tx,ty])
    quieto=np.r_[False,np.hypot(*np.diff(pos,axis=0).T)<0.05]
    # inicios de tramos quietos
    out=[]
    i=1
    while i<len(tt):
        if quieto[i] and not quieto[i-1]:
            j=i
            while j+1<len(tt) and quieto[j+1]: j+=1
            ta=tt[i-1]; tb=tt[j] if j+1<len(tt) else t[-1]
            punto="P" if np.hypot(*(pos[i]-P))<0.6 else "Q"
            w=(t>=ta)&(t<=min(tb,ta+1.95))
            tw=t[w]-ta; g=gz[w]
            if len(tw)>30:
                pk=np.max(np.abs(g))
                kp=np.argmax(np.abs(g))
                fit=ajustar(tw[kp:]-tw[kp],g[kp:])
                fin=np.sqrt(np.mean(g[tw>tw[-1]-0.3]**2))
                out.append(dict(punto=punto,ta=ta,dur=tw[-1],pk=pk,t2=t_bajo(tw,g,2.0),resid=fin,**(fit or dict(fn=np.nan,zeta=np.nan,R2=np.nan))))
            i=j+1
        else: i+=1
    # última llegada: la telemetría se corta al llegar; el IMU sigue ~2.4 s
    if len(tt)>2 and np.hypot(*(pos[-1]-P))<0.6 and (not out or abs(out[-1]["ta"]-tt[-1])>0.5):
        ta=tt[-1]; w=(t>=ta)&(t<=ta+1.95); tw=t[w]-ta; g=gz[w]
        if len(tw)>30:
            kp=np.argmax(np.abs(g)); fit=ajustar(tw[kp:]-tw[kp],g[kp:])
            out.append(dict(punto="P",ta=ta,dur=tw[-1],pk=np.max(np.abs(g)),t2=t_bajo(tw,g,2.0),
                            resid=np.sqrt(np.mean(g[tw>tw[-1]-0.3]**2)),**(fit or dict(fn=np.nan,zeta=np.nan,R2=np.nan))))
    return info,out

def golpes(d):
    imu,tel,info=leer(d); t=imu["t_s"]; g=imu["gz"]-np.median(imu["gz"])
    out=[];k=0
    while k<len(t):
        if abs(g[k])>15:
            j=k+np.argmax(np.abs(g[k:k+30]))
            w=(t>t[j]+0.05)&(t<t[j]+0.5); k2=np.where(w)[0]
            if len(k2):
                kr=k2[np.argmax(np.abs(g[k2]))]
                s=(t>=t[kr])&(t<=t[kr]+1.6)
                fit=ajustar(t[s]-t[kr],g[s])
                out.append(dict(pk=abs(g[j]),reb=abs(g[kr]),t2=t_bajo(t[(t>=t[j])&(t<t[j]+2.5)]-t[j],g[(t>=t[j])&(t<t[j]+2.5)]),**(fit or dict(fn=np.nan,zeta=np.nan,R2=np.nan))))
            k=j+150
        else: k+=1
    return info,out
