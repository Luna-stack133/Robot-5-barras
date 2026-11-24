from rutas import *
import sys, json, numpy as np, csv, os, tempfile, warnings; warnings.filterwarnings("ignore")
import modelo5b as m, video_real as vr
from video_lib import sincronizar, cargar_video
import os; os.environ["FF_FIJO"]="0.5"
from identificacion import cargar, const_inercia, train
from indice import indice
R=[r for r in indice() if r["tipo"]=="trayectoria"]
res=json.load(open(ruta_res("identificacion_ff0.5.json")))
p=res["params"]; pc=res["params_const"]
qc=vr.ik(0.075,0.200); Mc,_=m.masa(*qc)
tau_rig=json.load(open(ruta_res("perfiles.json")))["rigido_tau"] if os.path.exists(ruta_res("perfiles.json")) else 0.06
# exclusión decidida al revisar la alineación (no por el error del modelo): ver README, pregunta 1
EXCL_VIDEO_TRAY={"2026-11-23_16-15-55":"desalineación de 58 mm tras la sincronización"}
out=[]; FALLAS=[]
for r in R:
    s=sincronizar(r["carpeta"],r["sesion"])
    if s is None: continue
    vp,lag,rs=s
    v=np.genfromtxt(vp,delimiter=",",names=True)
    c=cargar(r); dur=c["t"][-1]-c["t"][0]
    k=(v["t_s"]>=lag+c["t"][0]-1.0)&(v["t_s"]<=lag+c["t"][-1]+1.0)
    tmp=tempfile.NamedTemporaryFile(suffix=".csv",delete=False).name
    with open(vp) as f: hdr=f.readline().strip()
    np.savetxt(tmp,np.array([list(x) for x in v[k]]),delimiter=",",fmt="%.4f",header=hdr,comments="")
    try: rec=vr.reconstruir(tmp,r["carpeta"],verbose=False)
    except Exception as e:
        print(r["nombre"],"ERROR",e); FALLAS.append(dict(nombre=r["nombre"],video=os.path.basename(vp),error=str(e))); continue
    os.unlink(tmp)
    tv,Pv=rec["t"],rec["P"]
    def efector(q): return np.array([m.fk(a,b)[2] for a,b in q])*1000
    qm,_,_=m.simular(c,p); Pm=efector(qm)
    cc=const_inercia(c,Mc); qk,_,_=m.simular(cc,pc); Pk=efector(qk)
    td=c["t"]-tau_rig; q1=np.interp(td,c["t"],c["q1c"]); q2=np.interp(td,c["t"],c["q2c"]); Pr=efector(np.column_stack([q1,q2]))
    w=(tv>=c["t"][0]+0.3)&(tv<=c["t"][-1])
    def err(Pp):
        Pi=np.column_stack([np.interp(tv[w],c["t"],Pp[:,i]) for i in range(2)]); E=Pv[w]-Pi; E=E-E.mean(0)
        return float(np.sqrt(np.mean(np.sum(E**2,1)))), float(np.mean(np.hypot(*E.T)))
    e_m,a_m=err(Pm); e_k,a_k=err(Pk); e_r,a_r=err(Pr)
    conj="ajuste" if r in train else "prueba"
    mot=EXCL_VIDEO_TRAY.get(r["nombre"][:19],"")
    out.append(dict(nombre=r["nombre"],sesion=r["sesion"],tray=r["nivel"],conjunto=conj,video=os.path.basename(vp),video_valido=(mot==""),motivo_video=mot,r_sinc=rec["info"]["r_sinc"],r_vel=rs,
                    rmse_modelo=e_m,rmse_const=e_k,rmse_rigido=e_r,mae_modelo=a_m,mae_const=a_k,mae_rigido=a_r))
    print("%s %-10s %-6s r=%.2f | modelo %.2f  const %.2f  rígido %.2f mm"%(r["nombre"][11:19],r["nivel"],conj,rec["info"]["r_sinc"],e_m,e_k,e_r),flush=True)
with open(ruta_res("tray_video.csv"),"w",newline="") as f:
    wr=csv.DictWriter(f,list(out[0].keys())); wr.writeheader(); wr.writerows(out)

with open(ruta_res("tray_video_fallas.csv"),"w",newline="") as f:
    wr=csv.DictWriter(f,["nombre","video","error"]); wr.writeheader(); wr.writerows(FALLAS)
