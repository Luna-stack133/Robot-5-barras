from rutas import *
import sys, os, glob, json, re, datetime, numpy as np, warnings
warnings.filterwarnings("ignore")
import video_real as vr
BASE=DATOS
_cache={}
REF_MODELO=None  # la sincronización contra el modelo se probó y se descartó: hace inestables las ventanas de ajuste
def cargar_video(path):
    if path in _cache: return _cache[path]
    v=np.genfromtxt(path,delimiter=",",names=True)
    G,Pv,rms=vr.rectificar(v)
    tv=v["t_s"]; ok=v["ok"]>0.5
    nm=os.path.basename(path); m=re.search(r"(\d{8})_(\d{6})",nm)
    t0=datetime.datetime.strptime(m.group(1)+m.group(2),"%Y%m%d%H%M%S")
    d=dict(path=path,tv=tv,P=Pv,ok=ok,rms=rms,t0=t0,ses=nm[:2],lado=np.nanmedian(v["lado_px"]))
    _cache[path]=d; return d
def videos_sesion(ses):
    return sorted(glob.glob(BASE+"/video_tracking/%s_*_marcador.csv"%ses))
def rapidez(t,P,suav=5):
    v=np.hypot(np.gradient(P[:,0],t),np.gradient(P[:,1],t))
    return np.convolve(v,np.ones(suav)/suav,mode="same")
def sincronizar(carpeta, ses, off=22.0, ventana=25.0):
    """Encuentra el video y el desfase: t_video = t_corrida + lag. Devuelve (video, lag, r)."""
    tel=np.genfromtxt(carpeta+"/telemetria.csv",delimiter=",",names=True)
    info=json.load(open(carpeta+"/info.json"))
    T0=datetime.datetime.strptime(info["fecha"],"%Y-%m-%d %H:%M:%S")
    tt=tel["t_s"]; Pc=np.column_stack([tel["x_mm"],tel["y_mm"]])
    if REF_MODELO:
        # referencia = efector simulado con el modelo identificado (incluye el atraso real del servo);
        # sincronizar contra el comando sesgaría el desfase en el retardo efectivo (40–90 ms)
        import modelo5b as m
        c=m.preparar(m.leer_corrida(carpeta)); q,_,_=m.simular(c,REF_MODELO)
        Pm=np.array([m.fk(a,b)[2] for a,b in q])*1000; tt=c["t"]; Pc=Pm
    g=np.arange(max(tt[0],0),tt[-1],0.01); Pg=np.column_stack([np.interp(g,tt,Pc[:,k]) for k in range(2)])
    vc=rapidez(g,Pg,9)
    best=None
    for vp in videos_sesion(ses):
        V=cargar_video(vp)
        pred=(T0-V["t0"]).total_seconds()-off
        if pred< -ventana or pred>V["tv"][-1]+ventana: continue
        okm=V["ok"]
        vv=rapidez(V["tv"][okm],V["P"][okm],5)
        for lag in np.arange(pred-ventana,pred+ventana,0.02):
            a=np.interp(g+lag,V["tv"][okm],vv,left=np.nan,right=np.nan)
            k=np.isfinite(a)
            if k.sum()<0.9*len(g): continue
            r=np.corrcoef(a[k],vc[k])[0,1]
            if best is None or r>best[2]: best=(vp,lag,r)
    if best is None: return None
    vp,lag,r=best
    # refinamiento fino
    V=cargar_video(vp); okm=V["ok"]; vv=rapidez(V["tv"][okm],V["P"][okm],5)
    for l2 in np.arange(lag-0.05,lag+0.05,0.002):
        a=np.interp(g+l2,V["tv"][okm],vv,left=np.nan,right=np.nan); k=np.isfinite(a)
        if k.sum()<0.9*len(g): continue
        r2=np.corrcoef(a[k],vc[k])[0,1]
        if r2>best[2]: best=(vp,l2,r2)
    return best
