from rutas import *
import sys, numpy as np
from estilo import *
import modelo5b as m, video_real as vr
from r5b import kinematics as kin
P={"A":(75,280),"B":(170,220),"C":(75,200),"D":(0,150),"V1":(25,240),"V2":(150,250)}
fig,ax=plt.subplots(figsize=(3.5,3.5))
# espacio de trabajo útil (calidad > 0.3)
xs=np.arange(-220,371,3); ys=np.arange(0,420,3); Q=np.full((len(ys),len(xs)),np.nan)
for i,y in enumerate(ys):
    for j,x in enumerate(xs):
        try:
            q=kin.quality_xy(x,y)
            Q[i,j]=q if q is not None else np.nan
        except Exception: pass
ax.contourf(xs,ys,np.where(Q>0.3,1,np.nan),levels=[0.5,1.5],colors=["#f1f0ec"])
ax.contour(xs,ys,np.nan_to_num(Q,nan=0),levels=[0.3],colors=[GRAY],linewidths=0.6,linestyles="--")
# mecanismo en C
t=vr.ik(0.095,0.175); (e1,e2,ef)=m.fk(*t); e1=np.array(e1)*1000; e2=np.array(e2)*1000; ef=np.array(ef)*1000
M1=np.array([0,0]); M2=np.array([150,0])
for a,b,lw in ((M1,e1,3),(M2,e2,3),(e1,ef,2.2),(e2,ef,2.2)):
    ax.plot([a[0],b[0]],[a[1],b[1]],color=INK,lw=lw,solid_capstyle="round",zorder=3)
for p in (M1,M2): ax.add_patch(plt.Rectangle(p-[14,14],28,28,color="#d9d8d2",zorder=2,ec=INK,lw=0.6))
for p in (e1,e2): ax.plot(*p,"o",ms=4,mfc="white",mec=INK,mew=0.8,zorder=4)
# IMU sobre el distal derecho
u=(ef-e2)/np.linalg.norm(ef-e2); pi=ef-u*45
ax.plot(*pi,"s",ms=5,color="#e34948",zorder=5); ax.annotate("IMU (BNO055)\non right distal",pi,xytext=(pi[0]+45,pi[1]+5),fontsize=6.5,arrowprops=dict(arrowstyle="-",lw=0.5,color=INK))
ax.plot(*ef,"o",ms=7,mfc="#cfcfcf",mec=INK,mew=0.8,zorder=5)
ax.annotate("effector + Z axis\n(20 mm video marker)",ef,xytext=(-195,195),fontsize=6.5,arrowprops=dict(arrowstyle="-",lw=0.5,color=INK))
for k,(x,y) in P.items():
    val=k.startswith("V")
    ax.plot(x,y,MK[k],ms=6,mfc="white" if val else COL[k],mec=COL[k],mew=1.2,zorder=6)
    ax.text(x+7,y+6,k,fontsize=7,fontweight="bold",color=INK,zorder=7)
ax.text(0,-28,"servo 1",ha="center",fontsize=6.5); ax.text(150,-28,"servo 2",ha="center",fontsize=6.5)
ax.text(75,-48,"JMC iHSV57, STEP/DIR from ESP32-S3",ha="center",fontsize=6.5,color=GRAY)
ax.annotate("",xy=(0,-12),xytext=(150,-12),arrowprops=dict(arrowstyle="<->",lw=0.5)); ax.text(75,-10,"D = 150",ha="center",va="bottom",fontsize=6)
ax.set_aspect("equal"); ax.set_xlim(-200,350); ax.set_ylim(-60,410)
ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ejes(ax)
ax.plot([],[],"o",color=GRAY,label="DOE postures (filled)"); ax.plot([],[],"o",mfc="white",mec=GRAY,label="validation postures")
ax.plot([],[],ls="--",color=GRAY,lw=0.6,label="usable workspace"); ax.legend(loc="upper left",frameon=False,fontsize=6.5)
guardar(fig,"fig1_mecanismo")
