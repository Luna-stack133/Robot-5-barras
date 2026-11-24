from rutas import *
import csv, numpy as np, itertools
from scipy import stats
R=[r for r in csv.DictReader(open(ruta_res("imu_corridas.csv")))]
def f(x):
    try: return float(x)
    except: return np.nan
doe=[r for r in R if r["tipo"]=="excitacion" and r["valida"]=="True" and r["postura"] in "ABCD" and r["replica"]]
POS="ABCD"; NIV=["L1","L2","L3"]
def celda(var):
    out={}
    for p,l in itertools.product(POS,NIV):
        v=np.array([f(r[var]) for r in doe if r["postura"]==p and r["nivel"]==l])
        reps=sorted(int(r["replica"]) for r in doe if r["postura"]==p and r["nivel"]==l)
        n=np.isfinite(v).sum(); m=np.nanmean(v); s=np.nanstd(v,ddof=1)
        h=stats.t.ppf(0.975,n-1)*s/np.sqrt(n) if n>1 else np.nan
        out[(p,l)]=dict(n=n,reps=reps,media=m,sd=s,ic=h,cv=100*s/m)
    return out
def anova(var, log=False):
    y=[];X=[]
    rows=[r for r in doe if np.isfinite(f(r[var]))]
    for r in rows:
        y.append(np.log(f(r[var])) if log else f(r[var]))
    y=np.array(y)
    def dummies(key,levels):   # codificación de efectos (suma cero): tipo III válido con interacción
        return np.array([[1.0 if r[key]==lv else (-1.0 if r[key]==levels[0] else 0.0) for lv in levels[1:]] for r in rows])
    Dp=dummies("postura",list(POS)); Dl=dummies("nivel",NIV); Ds=dummies("sesion",["S2","S3"])
    Dpl=np.hstack([Dp[:,[i]]*Dl[:,[j]] for i in range(Dp.shape[1]) for j in range(Dl.shape[1])])
    one=np.ones((len(y),1))
    def rss(*blocks):
        Xm=np.hstack([one]+list(blocks)); b,*_=np.linalg.lstsq(Xm,y,rcond=None); e=y-Xm@b; return e@e, Xm.shape[1]
    full,pf=rss(Ds,Dp,Dl,Dpl); dfe=len(y)-pf; mse=full/dfe
    res={}
    for nm,(red) in {"sesion (bloque)":(Dp,Dl,Dpl),"postura":(Ds,Dl,Dpl),"nivel":(Ds,Dp,Dpl),"postura×nivel":(Ds,Dp,Dl)}.items():
        r2,pr=rss(*red); ss=r2-full; df=pf-pr; F=(ss/df)/mse; p=stats.f.sf(F,df,dfe); eta=ss/(ss+full)
        res[nm]=dict(df=df,F=F,p=p,eta2p=eta)
    return res,dfe,len(y)
if __name__=="__main__":
    for var in ("fn","zeta"):
        C=celda(var)
        print("==",var)
        for p in POS:
            print(p," | ".join("%s: %.3f ± %.3f (n=%d, CV %.1f%%)"%(l,C[(p,l)]["media"],C[(p,l)]["ic"],C[(p,l)]["n"],C[(p,l)]["cv"]) for l in NIV))
        res,dfe,N=anova(var, log=(var=="fn"))
        print("N=%d dfe=%d"%(N,dfe))
        for k,v in res.items(): print("  %-15s df=%d F=%.1f p=%.2g eta2p=%.2f"%(k,v["df"],v["F"],v["p"],v["eta2p"]))
