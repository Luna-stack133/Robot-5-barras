from rutas import *
import sys, json, numpy as np
import modelo5b as m, video_real as vr
P={"A":(75,280),"B":(170,220),"C":(75,200),"D":(0,150),"V1":(25,240),"V2":(150,250)}
def jef(t1,t2,e=1e-6):
    p0=np.array(m.fk(t1,t2)[2]); J=np.zeros((2,2))
    for k,d in enumerate(([e,0],[0,e])): J[:,k]=(np.array(m.fk(t1+d[0],t2+d[1])[2])-p0)/e
    return J
def modos(x,y,K1,K2,B1,B2,mx=0.0):
    t=vr.ik(x/1000,y/1000); M,J3=m.masa(*t); Je=jef(*t); M=M+mx*Je.T@Je
    K=np.diag([K1,K2]); B=np.diag([B1,B2]); Mi=np.linalg.inv(M)
    A=np.block([[np.zeros((2,2)),np.eye(2)],[-Mi@K,-Mi@B]]); ev,V=np.linalg.eig(A)
    out=[]
    for k in range(4):
        if ev[k].imag>0:
            wn=abs(ev[k]); z=-ev[k].real/wn; v=V[:2,k]; vis_g=abs(J3@v)/np.linalg.norm(v); vis_e=np.linalg.norm(Je@v)/np.linalg.norm(v)
            out.append(dict(fn=wn/2/np.pi,zeta=z,vis_gz=float(vis_g),vis_ef=float(vis_e)))
    rel=[M[0,0]/m.J_ROTOR,M[1,1]/m.J_ROTOR]
    return sorted(out,key=lambda d:d["fn"]),rel
if __name__=="__main__":
    p=json.load(open(ruta_res("identificacion_ff0.5.json")))["params"]
    res={}
    for k,(x,y) in P.items():
        md,rel=modos(x,y,p["K1"],p["K2"],p["B1"],p["B2"]); md1,_=modos(x,y,p["K1"],p["K2"],p["B1"],p["B2"],0.1)
        res[k]=dict(modos=md,modos_100g=md1,rel_inercia=rel)
        print(k," | ".join("%.2f Hz ζ%.2f gz%.2f ef%.3f"%(d["fn"],d["zeta"],d["vis_gz"],d["vis_ef"]) for d in md),"| 100g:"," ".join("%.2f"%d["fn"] for d in md1),"| J/Jrot %.0f %.0f"%tuple(rel))
    json.dump(res,open(ruta_res("modos_modelo.json"),"w"),indent=1)
